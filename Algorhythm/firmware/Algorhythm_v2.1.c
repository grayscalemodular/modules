/****************************************************************************
*
*	Algorhythm
*   Copyright (c) 2013 Grayscale, LLC
*
*	Version History
*
*	1.6		8/12/16
*		- turned off LEDs on power down interrupt.
*
*	1.5		4/24/14
*		- re-did yesterdays stuff.  See notes in clockISR.
*
*	1.4		4/23/14
*		- added more pacing for clock interrupt
*
*	1.3		4/09/2014
*		- fixed a bug with double reset when reset occurs when clock is low.
*
*	1.2		3/27/2014
*		- shortened hold timer to 750ms from 1000ms and altered LED flashing
*		  so that if an LED is off, the first flash is on.  Also resets flash timer.
*
*	1.1		3/18/2014
*		- put minimum limits on link pulse so that linked modules wouldn't stop
*		  with fast clock rates.
*
*	1.0		12/xx/2013	2nd Prototype
*
*	0.1		10/18/2013	Prototype
*
*****************************************************************************/

#ifdef __DEBUG
#pragma config PWRTEN = OFF
#pragma config DEBUG = ON
#else
#pragma config PWRTEN = ON
#pragma config DEBUG = OFF
#endif
#pragma config FOSC = INTIO2, PLLCFG = OFF, FCMEN = OFF, IESO = OFF
#pragma config WDTEN = OFF, SOSCSEL = DIG
#pragma config BOREN = SBORDIS, BORV = 2
#pragma config MCLRE = ON
#pragma config STVREN = ON
#pragma config XINST = ON
#pragma config CCP2MX = PORTBE
#pragma config RETEN = OFF


/****************************************************************************
*
*	Defines
*
****************************************************************************/

#include <p18f66k22.h>
#include <stdlib.h>
#include <string.h>

#define LED_DIM_COUNTS			(unsigned int) 300
#define MIN_LED_PULSE			(unsigned char) 25
// trigger pulse length is for all output pulses
#define TRIGGER_PULSE			(unsigned char) 10
#define LINK_PULSE				(unsigned char) 8
#define HOLD_THRESHOLD			(unsigned int) 750
#define SWITCH_CHECK_INTERVAL	(unsigned char) 10
#define LINK_CHECK_INTERVAL		(unsigned char) 5

// LED BLINK RATE is on a 2 ms timer, so value is halved
#define LED_BLINK_RATE			(unsigned char) 125
#define OFF	0
#define ON	1
#define DIM 1
#define NORMAL 	0
#define TRUE	1
#define FALSE	0

/****************************************************************************
*
*	EEPROM address defines
*
*	EEPROM exists within the controller between the
*	addresses of 0x0000 and 0x03FF.  It is used for non-volatile
*	settings that are stored at power down and restored at power up.
*
****************************************************************************/

#define EEFLAG_ADDR 0x00			// 2 bytes 0xa55a if programmed
#define ENABLED_STEP_ADDR 0x02		// 8 bytes for enabled steps
#define ENABLED_GATE_ADDR 0x0a		// 8 bytes for enabled gates
#define PATTERN_SETTING_ADDR 0x12	// 1 byte for pattern setting
#define LOOP_SETTING_ADDR 0x13		// 1 byte for loop setting
#define LOOP_MODE_ADDR 0x14			// 1 byte for loop mode
#define SEQ_MODE_ADDR 0x15			// 1 byte for seq mode
#define GATE_MODE_ADDR 0x16			// 1 byte for gate mode


/****************************************************************************
*
*	Globals
*
****************************************************************************/

// timing variables
volatile union {
	unsigned char d[2];
	unsigned int 	i;
} genDelay;

//unsigned int genDelay;
volatile unsigned char LEDTriggerTimer;
unsigned char triggerPulseTimer;
unsigned char linkPulseTimer;
unsigned char startPulseTimer;
unsigned char stopPulseTimer;
unsigned char patternPulseTimer;
unsigned char LEDflashTimer;
unsigned char linkCheckInterval;

// LED control variables

// These variables do not hold numeric values.  They are all bitwise, using bits 0-3.
volatile unsigned char LEDstatuses;		// row 1
volatile unsigned char LEDflashBits;	// row 1 flashing control
volatile unsigned char LEDflashFlag;
// the index of each of these is the current view. This way, each view is virtual and simulataneous
//  until actually displayed.
unsigned char LEDstepsR2[4] = {0, 0, 0, 0};			// row 2
unsigned char LEDstepsR3[4] = {0, 0, 0, 0};			// row 3
unsigned char LEDstepsR2Dim[4] = {0, 0, 0, 0};		// row 2 dim
unsigned char LEDstepsR3Dim[4] = {0, 0, 0, 0};		// row 3 dim
unsigned char LEDstepsR2Flags;		// used for consistent ISR communication in case view changes
unsigned char LEDstepsR3Flags;		//  between the occurance of CCP5ISR and CCP6ISR
unsigned char LEDstepsR2DimFlags;
unsigned char LEDstepsR3DimFlags;
volatile unsigned char LEDcolumn; 		// currently illuminated LED column bitwise
// numerical variables
volatile unsigned char LEDcolumnCount; 	// currently illuminated LED column numerical

enum _statusLED{
	LEDSTART,
	LEDLOOP,
	LEDSEQ,
	LEDGATE
};

// step related variables
unsigned char currentStep;
unsigned char enabledStep[9];
unsigned char resetRequest;
unsigned char linkStopAllRequest;
unsigned char stepCounter;
//unsigned char patternCounter;
unsigned char patternSetting;
unsigned char loopCounter;
unsigned char loopSetting;
unsigned char gateSetting[9];

// switch checking
unsigned char currentPortD;
unsigned char previousPortD;
unsigned char currentPortA;
unsigned char previousPortA;
unsigned int switchLoopHoldTimer;
unsigned int switchSeqHoldTimer;
unsigned int switchGateHoldTimer;
unsigned char ignoreRelease = 0;
unsigned char switchCheckInterval;

// modes
volatile unsigned char startMode;
volatile unsigned char runMode;			// with startMode = 1 and runMode = 0 == armed mode
unsigned char loopMode;
unsigned char seqMode;
unsigned char gateMode;
unsigned char linkMode;
unsigned char invertedMode = 0;			// panel inversion, not implemented at this time

enum _linkRole {
	LINKNONE,
	LINKMASTER,
	LINKMIDDLE,
	LINKLAST
} linkRole;

enum _view {
	VIEWGRID,
	VIEWLOOP,
	VIEWPATTERN,
	VIEWGATE
} currentView = VIEWGRID;

unsigned char clockFlag = 0;
unsigned char clockDisableFlag = 0;
unsigned char linkOutMinFlag = 0;
unsigned char linkInputFlag = 0;
unsigned char linkStopFlag = 0;
unsigned char endRequest = 0;

/****************************************************************************
*
*	Function Prototypes
*
****************************************************************************/
void DI(void);
void EI(void);
void ISRhighvector(void);
void ISRlowvector(void);
void highISR(void);
void lowISR(void);
void main(void);
void initialize(void);
void initT0(void);
void setSeed(void);
void initT3(void);
void initT5(void);
void initPortBInterrupts(void);
void initIO(void);
void initNonVolatileValues(void);
void saveSettings(void);
void CCP4ISR(void);
void CCP5ISR(void);
void CCP6ISR(void);
void clockISR(void);
void modeISR(void);
void resetISR(void);
void stateISR(void);
void setStatusLED(enum _statusLED sLED, unsigned char on, unsigned char flash);
void setStepLED(enum _view view, unsigned char stepLED, unsigned char on, unsigned char dim);
void genDelayFunc(unsigned int counts);
void pollSwitches(void);
void advanceStep(void);
void clockLow(void);
void start(void);
void stop(void);
void setPatternOut(unsigned char on);
void setStepOut(unsigned char step, unsigned char on);
void sendLinkStopPulse(void);
void sendLinkPulse(void);
void sendStartPulse(void);
void sendStopPulse(void);
void initPowerFail(void);
void initLinkMode(void);
void changeView(enum _view newView);
void displayGrid(void);
void displayLoop(void);
void displayPattern(void);
void displayGate(void);
void pollLinkSignals(void);
unsigned char readEEchar(unsigned int address);
unsigned int readEEint(unsigned int address);
void writeEEchar(unsigned int address, unsigned char data);
void writeEEint(unsigned int address, unsigned int data);

/****************************************************************************
*
*	Macros
*
****************************************************************************/
void DI(void)
{
	_asm
	BCF INTCON,6,0 			// low priority interrupts
	BCF INTCON,7,0 			// high priority interrups
	_endasm
}

void EI(void)
{
	_asm
	BSF INTCON,7,0 			// high priority interrupts
	BSF INTCON,6,0 			// low priority interrups
	_endasm
}

/****************************************************************************
*
* Interrupt Vectors
*
****************************************************************************/

#pragma code highvector = 0x08
void ISRhighvector(void)
{
	_asm
	GOTO	highISR
	_endasm
}
#pragma code lowvector = 0x18
void ISRlowvector(void)
{
	_asm
	GOTO	lowISR
	_endasm
}
#pragma code


/****************************************************************************
*
* Interrupt Handlers
*
****************************************************************************/

#pragma interrupt highISR
void highISR(void)
{
	if(PIR4bits.CCP6IF)
		CCP6ISR();
	if(PIR4bits.CCP5IF)
		CCP5ISR();
	if(PIR6bits.CMP3IF)
		saveSettings();
	if(INTCONbits.INT0IF)
		clockISR();
}

#pragma interruptlow lowISR
void lowISR(void)
{
	if(PIR4bits.CCP4IF)
		CCP4ISR();
	if(INTCON3bits.INT1IF)
		modeISR();
	if(INTCON3bits.INT2IF)
		resetISR();
	if(INTCON3bits.INT3IF)
		stateISR();
}

/****************************************************************************
*
* Function:	main
*
* Description:  Main loop.  Nothing much happens here due event driven nature
* 	of things.
*
****************************************************************************/

void main(void)
{
	initialize();
	while(1);

/* 	// internal test clock
	while(1)
	{
		genDelayFunc(500);
		if(startMode)
			advanceStep();
		genDelayFunc(500);
		if(runMode)
			clockLow();
	}
*/
}

/****************************************************************************
*
* Function: initialize
*
* Description:	  Initializes the hardware and associated global variables
*
****************************************************************************/

void initialize(void)
{
	unsigned char c;
	// CONFIG registers using internal clock at 16 MHz
	OSCCON = 0b01110010;
	OSCCON2 = 0;
	RCONbits.IPEN = 1; 	// make sure dual priority interrupts are used

	initIO();			// initialize processor pins
	initPowerFail();	// intiallize the comparator for power fail detection

	// Map the timers
	CCPTMRS1 = 0b00010101; 	// Both CCP5 and CCP6 will use Timer 5
							// CCP4 will use Timer 3

	// init the timers
	initT0();			// initialize timer 0 for random number seed generation
	initT5();			// initialize timer 5/CCP5/CCP6 for LED control
	initT3(); 			// initialize timer 3/CCP4 for general purposes
						//  software timers

	initLinkMode();		// determine if module is linked and re-initialize pins accordingly

	// check to see if the contents of EEPROM are valid.  If yes, read in stored settings.
	//  If not, initialize to default values
	initNonVolatileValues();

	previousPortA = 0x00;	// initialize for pollSwithes
	previousPortD = 0x00;

	currentStep = 0;		// in armed mode, current step is zero
	stepCounter = 1;
	loopCounter = 1;
	initPortBInterrupts();

	EI();

}

/****************************************************************************
*
* Function: initT0
*
* Description: Timer 0 is started.  Timer 0 is used in free-running mode to
* 		assist with random number generation.
*
****************************************************************************/
void initT0(void)
{
	T0CON = 0b10000001;		// start timer 0 free running at 1usec clock rate
}

void setSeed(void)
{
	union {
		unsigned char bytes[2];
		unsigned int w;
	} value;

	value.bytes[0] = TMR0L;
	value.bytes[1] = TMR0H;
	srand(value.w);
}

/****************************************************************************
*
* Function: initT3
*
* Description:	  Initializes timer 3 and CCP4 for general purpose software
*			timing at 1 mS intervals.
*
*			Timer 3 runs at 1 usec clock source
*
****************************************************************************/

void initT3(void)
{
	// init timing variables
	genDelay.i = 0;
	LEDTriggerTimer = 0;
	linkPulseTimer = 0;
	triggerPulseTimer = 0;
	startPulseTimer = 0;
	stopPulseTimer = 0;
	patternPulseTimer = 0;
	switchCheckInterval = SWITCH_CHECK_INTERVAL;
	linkCheckInterval = LINK_CHECK_INTERVAL;
	switchLoopHoldTimer = 0;
	switchSeqHoldTimer = 0;
	switchGateHoldTimer = 0;

	// Init CCP4 (page 245)
	CCP4CON = 0b00001011;	// Compare mode with special event trigger
	CCPR4 = 0x03e8;			// Compare register = 1000d = 0x03e8 = 1 ms
	IPR4bits.CCP4IP = 0;	// low priority interrupt
	PIR4bits.CCP4IF = 0;	// clear interrupt flag
	PIE4bits.CCP4IE = 1;	// Enable the interrupt

	// Init Timer 3 (page 211)
	TMR3H = 1;				// this will offset 500uS ahead of timer 5 so that IRQs are not
	TMR3L = 0xf4;			//  all at the same time.
	T3CON = 0b00100001;		// prescale of 4 (16MHz /4 /4 = 1MHz), and start timer

}



/****************************************************************************
*
* Function: initT5
*
* Description:	  Initializes timer 5 and CCP5 and CCP6 for background LED
*			muxing and brightness management.  At the completion of
*			initialization, the timer is started and the associated interrupts
*			are enabled (the global interrupt setting is not modified).
*
*			LED Management:  The colums of the LEDs are sequenced at each
*			interrupt of CCP5.  During the ISR, the appropriate row content
*			is output, turning on/off LEDs according to the control array in
*			memory.  CCP6 will interrupt prior to the next column sequence
*			to turn off any LEDs specified for a lower brightness, giving
*			those LEDs a lower duty cycle.
*
*			Timer 5 runs at 1 usec clock source and interrupt is 2ms.
*
****************************************************************************/

void initT5(void)
{
	// init all LED control variables

	LEDstatuses = 0;
	LEDflashBits = 0;
	LEDcolumn = 1;
	LEDcolumnCount = 0;
	LEDflashFlag = 0;
	LEDflashTimer = LED_BLINK_RATE;

	// Init CCP5 (page 245)
	CCP5CON = 0b00001011;	// Compare mode with special event trigger
	CCPR5 = 0x07d0;			// Compare register = 2000d = 0x07d0 = 2 ms
	IPR4bits.CCP5IP = 1; 	// high priority
	PIR4bits.CCP5IF = 0;	// clear interrupt flag
	PIE4bits.CCP5IE = 1;	// Enable the interrupt

	// Init CCP6
	CCP6CON = 0b00001010;	// Compare mode only
	CCPR6 = LED_DIM_COUNTS;	// Compare register, value defined elsewhere
	IPR4bits.CCP6IP = 1; 	// high priority
	PIR4bits.CCP6IF = 0;	// clear interrupt flag
	PIE4bits.CCP6IE = 1;	// Enable the interrupt

	// Init Timer 5 (page 211)
	TMR5H = 0;
	TMR5L = 0;
	T5CON = 0b00100001;		// prescale of 4 (16MHz /4 /4 = 1MHz), and start timer

}

/****************************************************************************
*
* Function:	initPortBInterrupts
*
* Description:  INT0-3 correspond to port B bits 0-3.
*
****************************************************************************/
void initPortBInterrupts(void)
{
	// the clock input interrupt INT0 is triggered on both edges
	if(PORTB & 1)			// if signal is high, enable falling edge
	{
		INTCON2 &= ~0x40;	// set for falling edge
		clockFlag = 0;
	}
	else
	{
		INTCON2 |= 0x40;	// set for rising edge
		clockFlag = 1;
	}
	INTCON |= 0x10;			// enable clock edge interrupt
							// note this interrupt is fixed at high priority

	// the other interrups 1,2 and 3 are rising edge triggered and low priority

	INTCON2 |= 0b00111000;
	INTCON2 &= 0b11111101;	// INT1 has low priority
	INTCON3 = 0b00111000;	// enable 1 2 and 3, low priority
	resetRequest = FALSE;
	linkStopAllRequest = FALSE;
	clockDisableFlag = FALSE;
}

/****************************************************************************
*
* Function: initio
*
* Description: Initialize the processor i/o ports
*
****************************************************************************/
void initIO(void)
{

	LATA = 0b00000000;
	TRISA = 0b00001111;
							// bit 7: n/a
							// bit 6: n/a
							// bit 5: n/c
							// bit 4: n/c
							// bit 3: in - SW Start
							// bit 2: in - SW Loop
							// bit 1: in - SW Seq
							// bit 0: in - SW Gate
    LATB =  0b00000000;
	TRISB = 0b11001111;
							// bit 7: background debug
							// bit 6: background debug
							// bit 5: n/c
							// bit 4: n/c
							// bit 3: in - J STATE
							// bit 2: in - J RESET
							// bit 1: in - J MODE
							// bit 0: in - J CLOCKI
	LATC =  0b00000000;
	TRISC = 0b00000000;
							// bit 7: out - J8
							// bit 6: out - J7
							// bit 5: out - J6
							// bit 4: out - J5
							// bit 3: out - J4
							// bit 2: out - J3
							// bit 1: out - J2
							// bit 0: out - J1
	LATD = 0xFF;
	TRISD = 0xFF;			// all inputs
							// bit 7: in - SW 8
							// bit 6: in - SW 7
							// bit 5: in - SW 6
							// bit 4: in - SW 5
							// bit 3: in - SW 4
							// bit 2: in - SW 3
							// bit 1: in - SW 2
							// bit 0: in - SW 1

	LATE = 0b00000000;		// LED control
	TRISE = 0;
							// bit 7: n/c
							// bit 6: out - LED Row 1
							// bit 5: out - LED Row 2
							// bit 4: out - LED Row 3
							// bit 3: out - LED Col 4
							// bit 2: out - LED Col 3
							// bit 1: out - LED Col 2
							// bit 0: out - LED Col 1

	ANCON0 = 0;				// in order to use port F, comparators must be off
	ANCON1 = 0;				// (LATF is unaffected, but will turn off comparators anyway)

	LATF = 0;
	TRISF = 0b01100000;		//
							// bit 7: L PULLDOWNS (always leave on and low)
							// bit 6: L STOPALL	 (initially input)
							// bit 5: L LINKBACK (initially input)
							// bit 4: L LINK O
							// bit 3: J STOP O
							// bit 2: J START O
							// bit 1: J PATT O
							// bit 0: doesn't exist

	LATG = 0;
	TRISG = 0b00001111;		// bit 4: unused
							// bit 3: P FAIL IN (comparator input)
							// bit 2: L LINK IN
							// bit 1: L SENSE DNSTREAM
							// bit 0: L SENSE UPSTREAM
	ANCON2 = 0b00000010;	// comparator input must be set as analog input

}

/****************************************************************************
*
* Function: initNonVolatileValues
*
* Description: If the EEPROM has been previously programmed, the power up values
*		are read from the EEPROM.  Otherwise, factory defaults are programmed
*		and loaded.
*
*		NOTE:  Interrupts must not be enabled prior to calling.
*
****************************************************************************/
void initNonVolatileValues(void)
{
	unsigned int flag;
	unsigned char c;

	flag = readEEint(EEFLAG_ADDR);
	if(flag != 0xa55a)
	{
		// store factory defaults for first time power up

		writeEEint(EEFLAG_ADDR,0xa55a);
		for(c = 0; c < 8; c++)
		{
			writeEEchar((ENABLED_STEP_ADDR + c), TRUE);
			writeEEchar((ENABLED_GATE_ADDR + c), TRUE);
		}
		writeEEchar(PATTERN_SETTING_ADDR, 8);
		writeEEchar(LOOP_SETTING_ADDR, 1);
		writeEEchar(LOOP_MODE_ADDR, TRUE);
		writeEEchar(SEQ_MODE_ADDR, TRUE);
		writeEEchar(GATE_MODE_ADDR, TRUE);
	}

	// read saved values from EEPROM

	for(c = 0; c < 8; c++)
	{
		enabledStep[(c + 1)] = readEEchar(ENABLED_STEP_ADDR + c);
		gateSetting[(c + 1)] = readEEchar(ENABLED_GATE_ADDR + c);
	}

	patternSetting = readEEchar(PATTERN_SETTING_ADDR);
	loopSetting = readEEchar(LOOP_SETTING_ADDR);
	loopMode = readEEchar(LOOP_MODE_ADDR);
	seqMode = readEEchar(SEQ_MODE_ADDR);
	gateMode = readEEchar(GATE_MODE_ADDR);
	// other mode initialization
	startMode = FALSE;
	runMode = FALSE;

	// initialize the leds/views
	displayGrid();
	displayGate();
	displayPattern();
	displayLoop();

	setStatusLED(LEDSTART, FALSE, FALSE);
	setStatusLED(LEDLOOP, loopMode, FALSE);
	setStatusLED(LEDSEQ, seqMode, FALSE);
	setStatusLED(LEDGATE, gateMode, FALSE);
}

/****************************************************************************
*
* Function: saveSettings
*
* Description: Save non-volatile settings.  This occurs on power down interrupt.
*		Since this occurs on high priority interrupts, interrupts are not enabled
*		during EE write process.
*
****************************************************************************/
void saveSettings(void)
{
	unsigned char c;
	LATE = 0;
	for(c = 0; c < 8; c++)
	{
		writeEEchar((ENABLED_STEP_ADDR + c), enabledStep[(c + 1)]);
		writeEEchar((ENABLED_GATE_ADDR + c), gateSetting[(c + 1)]);
	}
	writeEEchar(PATTERN_SETTING_ADDR, patternSetting);
	writeEEchar(LOOP_SETTING_ADDR, loopSetting);
	writeEEchar(LOOP_MODE_ADDR, loopMode);
	writeEEchar(SEQ_MODE_ADDR, seqMode);
	writeEEchar(GATE_MODE_ADDR, gateMode);

	while(CMSTAT & 0x80);	// as long as voltage is falling or below threshold, stay here.
							// if the voltage bounces back without a reset, then normal
							// operation will ensue.

	PIR6 &= ~0x04;			// clear interrupt flag
}

/****************************************************************************
*
* Function: CCP4ISR
*
* Description: Interrupt service routine for general purpose timing. 1mS.
*
****************************************************************************/
void CCP4ISR(void)
{
	// clear IF
	PIR4bits.CCP4IF = 0;

	if(LEDTriggerTimer != 0)
	{
		LEDTriggerTimer--;
		if(LEDTriggerTimer == 0)
		{
			if(!gateMode)
			{
				if(currentStep <= patternSetting)
					setStepLED(VIEWGRID, currentStep, enabledStep[currentStep], DIM);
				else
					setStepLED(VIEWGRID, currentStep, OFF, NORMAL);
			}
			if(!gateSetting[currentStep])
			{
				setStepLED(VIEWGATE, currentStep, gateSetting[currentStep], DIM);
			}
		}
	}

	if(triggerPulseTimer != 0)
	{
		triggerPulseTimer--;
		if(triggerPulseTimer == 0)
		{
			if(!gateMode)
				setPatternOut(FALSE);
			if(gateSetting[currentStep] == 0)	// individual trigger mode
				setStepOut(currentStep, OFF);
		}
	}
	if(patternPulseTimer != 0)
	{
		patternPulseTimer--;
		if(patternPulseTimer == 0)
		{
			setPatternOut(OFF);
		}
	}

	if(startPulseTimer != 0)
	{
		startPulseTimer--;
		if(startPulseTimer == 0)
		{
			LATF &= ~0x04;		// set output low
		}
	}
	if(stopPulseTimer != 0)
	{
		stopPulseTimer--;
		if(stopPulseTimer == 0)
		{
			LATF &= ~0x08;		// set output low
		}
	}
	if(linkPulseTimer != 0)
	{
		linkPulseTimer--;
		if(linkPulseTimer == 0)
		{
			if(linkOutMinFlag == TRUE)			// implemented for high speed clocks
			{
				linkOutMinFlag = FALSE;			// (sequence of clearing important)
				if(clockDisableFlag)			// if this flag is set, a pulse overrun was about to occur
				{
					sendLinkPulse();			// send pulse now
					clockDisableFlag = FALSE;	// allow clock to run
				}
			}
			else
			{
				LATF &= ~0x70;			// set all link outputs low
				TRISFbits.TRISF6 = 1;	// set stopall back to input
				linkOutMinFlag = TRUE;
				linkPulseTimer = LINK_PULSE;
			}
		}
	}

	if(genDelay.i != 0)
		genDelay.i--;

	switchCheckInterval--;
	if(switchCheckInterval == 0)
	{
		pollSwitches();
		switchCheckInterval = SWITCH_CHECK_INTERVAL;
	}

	if(switchGateHoldTimer != 0)
	{
		switchGateHoldTimer--;
		if(switchGateHoldTimer == 0)
		{
			ignoreRelease |= 0x01;		// bitwise flag
			changeView(VIEWGATE);
		}
	}
	if(switchSeqHoldTimer != 0)
	{
		switchSeqHoldTimer--;
		if(switchSeqHoldTimer == 0)
		{
			ignoreRelease |= 0x02;		// bitwise flag
			changeView(VIEWPATTERN);
		}
	}
	if(switchLoopHoldTimer != 0)
	{
		switchLoopHoldTimer--;
		if(switchLoopHoldTimer == 0)
		{
			ignoreRelease |= 0x04;		// bitwise flag
			changeView(VIEWLOOP);
		}
	}

	if(linkRole != LINKNONE)
	{
		linkCheckInterval--;
		if(linkCheckInterval == 0)
		{
			pollLinkSignals();
			linkCheckInterval = LINK_CHECK_INTERVAL;
		}
	}
	INTCONbits.INT0IE = 1;	 			// This routine has had a chance to
										// to execute, so allow clock input.
}

/****************************************************************************
*
* Function: CCP5ISR
*
* Description: Interrupt service routine for CCP5 used for LED on/off
*
****************************************************************************/
void CCP5ISR(void)
{
	unsigned char LEDport;
	// clear IF
	PIR4bits.CCP5IF = 0;
	LEDcolumnCount++;
	if(LEDcolumnCount == 4)
	{
		LEDcolumnCount = 0;
		LEDcolumn = 1;			// set bit 0

		LEDstepsR2Flags = LEDstepsR2[(unsigned char)currentView];
		LEDstepsR3Flags = LEDstepsR3[(unsigned char)currentView];
		LEDstepsR2DimFlags = LEDstepsR2Dim[(unsigned char)currentView];
		LEDstepsR3DimFlags = LEDstepsR3Dim[(unsigned char)currentView];
	}
	else
	{
		LEDcolumn = LEDcolumn << 1;			// shift left to next column
	}
	LEDport = LEDcolumn;					// LEDs are preset off
	if((LEDcolumn & LEDflashBits) != 0)
	{
		// LED is in flashing mode
		if((LEDstatuses & LEDflashFlag & LEDcolumn) != 0)
		{
			LEDport |= 0x40;
		}
	}
	else
	{
		// LED is not in flashing mode
		if((LEDstatuses & LEDcolumn) != 0)
		{
			LEDport |= 0x40;
		}
	}
	if((LEDstepsR2Flags & LEDcolumn) != 0)
	{
		LEDport |= 0x20;
	}
	if((LEDstepsR3Flags & LEDcolumn) != 0)
	{
		LEDport |= 0x10;
	}
	LATE = LEDport;				// update port output

	if(LEDflashTimer-- == 0)
	{
		LEDflashTimer = LED_BLINK_RATE;
		LEDflashFlag = ~LEDflashFlag;		// Alternates between 0x00 and 0xff
	}
}

/****************************************************************************
*
* Function: CCP6ISR
*
* Description: Interrupt service routine for CCP6 used for turning off LEDs
*		early (before next column) to dim them.
*
****************************************************************************/
void CCP6ISR(void)
{
	// clear IF
	PIR4bits.CCP6IF = 0;
	if((LEDstepsR2DimFlags & LEDcolumn) != 0)
	{
		LATE &= ~0x20;
	}
	if((LEDstepsR3DimFlags & LEDcolumn) != 0)
	{
		LATE &= ~0x10;
	}
}

/****************************************************************************
*
* Function: clockISR
*
* Description:  Clock input interrupt service routine.  This is triggered on
*		both edges of the clock signal
*
****************************************************************************/
void clockISR(void)
{
	// the clock input interrupt INT0 is triggered on both edges
	if(!clockDisableFlag)		// for overrun of link pulse
	{
		if(!clockFlag)			// if signal is high, enable falling edge
		{
			INTCON2 &= ~0x40;	// set for falling edge
			if(startMode)
			{
				advanceStep();
			}
			clockFlag = 1;
			//
			// When the clock is fed audio frequencies, it overwhelms the
			//  other functions due to it having the highest priority interrupt.
			//  Ideally, the clock input should be moved to a lower priority interrupt
			//  input (hardware change) because INT0 is fixed at high priority.
			//  For a software fix, the interrupt is disabled
			//  until the 1 ms interrupt has had a chance to execute, where it is re-enabled.
			//  This will also allow the other low priority interrupts time to execute.
			// This is placed on the rising edge so that the falling edge, presummed not
			// as critical, is delayed first.
			INTCONbits.INT0IE = 0;	// disable interrupt until 1ms (CCP4ISR) has
									//  had a chance to execute. Reenabled there.
		}
		else
		{
			INTCON2 |= 0x40;	// set for rising edge
			if(runMode)
			{
				clockLow();
			}
			clockFlag = 0;
		}
	}
	INTCONbits.INT0IF = 0;		// clear interrupt flag
}

/****************************************************************************
*
* Function:	modeISR
*
* Description: Link input interrupt service routine.  This is triggered on the
*		rising edge of the link signal
*
****************************************************************************/
void modeISR(void)
{
	INTCON3bits.INT1IF = 0;		// clear interrupt flag
	if(seqMode == TRUE)
	{
		seqMode = FALSE;
	}
	else
	{
		seqMode = TRUE;
	}
	if(currentView != VIEWPATTERN)
	{
		setStatusLED(LEDSEQ, seqMode, FALSE);
	}
}

/****************************************************************************
*
* Function:  resetISR
*
* Description:  Reset input interrupt service routine.  This is triggered on
*		the rising edge of the reset signal.
*
****************************************************************************/
void resetISR(void)
{
	INTCON3bits.INT2IF = 0;		// clear interrupt flag
	if(startMode == FALSE)
		return;					// ignore if not running, else it will reset
								//  when it starts
	// Normally when the reset is connected to an Algorhythm, it is sychronous
	//  rising shortly after the clock rises (in response to an output turning on).
	//  In this case the current step is still in progress.
	//  If the clock is already low, the current step has already completed
	//  and some of the same functions handled in  clockLow need to be
	//  handled here.
	if((PORTB & 0x01) == 0)  	// is clock already low
	{
		stepCounter = 1;
		loopCounter = 1;
		resetRequest = FALSE;	// probably already cleared... just to make sure

		if(loopMode)
		{
			// loop mode won't stop on endRequest unless it is in link mode.
			if(linkRole != LINKNONE)
			{
				sendLinkPulse();
				stop();
			}
		}
		else		// end mode
		{
			stop();
		}
	}
	else // clock is high, handle reset in clockLow.
	{
		resetRequest = TRUE;
	}
}

/****************************************************************************
*
* Function:	stateISR
*
* Description: State input signal interrupt service routine.  It is triggered
*		on the rising edge of the State signal.  It is the same as pressing
*		the start/stop pushbutton.
*
****************************************************************************/
void stateISR(void)
{
	INTCON3bits.INT3IF = 0;		// clear interrupt flag
	if(startMode)
	{
		stop();
	}
	else
	{
		start();
		if(linkMode != LINKNONE)
			linkStopAllRequest = TRUE;
	}
}

/****************************************************************************
*
* Function: setStatusLED
*
* Description:  The status LEDs are the top row of LEDs that show either
*		Start/Stop status, or the status of the pushbuttons.
*
* Parameters:  The LED to be switched on or off, and either ON or OFF constant
*	LEDSTART,
*	LEDLOOP,
*	LEDSEQ,
*	LEDGATE
*
****************************************************************************/

void setStatusLED(enum _statusLED sLED, unsigned char on, unsigned char flash)
{
	unsigned char LEDbits;
	unsigned char c;
	// enum is 0 through 3
	LEDbits = 1;
	c = (unsigned char) sLED;
	LEDbits = LEDbits << c;
	if(on)
	{
		if(flash)
		{
			LEDflashBits |= LEDbits;
			if(LEDstatuses & LEDbits)
			{
				// if the LED is already on, start the first flash with it off
				LEDflashFlag = 0;
				LEDflashTimer = LED_BLINK_RATE;	// restart timer
			}
			else
			{
				// if the LED is already off, start the first flash with it on
				LEDflashFlag = 0xff;
				LEDflashTimer = LED_BLINK_RATE;	// restart timer
			}
		}
		else
			LEDflashBits &= ~LEDbits;
		LEDstatuses |= LEDbits;

	}
	else
	{
		LEDstatuses &= ~LEDbits;
		LEDflashBits &= ~LEDbits;
	}
}

/****************************************************************************
*
* Function: setStepLED
*
* Description:  The step LEDs are turned on dimly if the switch is engaged
*		and turn on full brightness if the step is active.
*
* Parameters:  	view:  The view that should be updated or changed
*				stepLED: The step LED to be switched on or off (1-8)
*				on: the ON or OFF constant (1 or 0)
*				dim: DIM or NORMAL constant or TRUE or FALSE
*
****************************************************************************/

void setStepLED(enum _view view, unsigned char stepLED, unsigned char on, unsigned char dim)
{
	unsigned char LEDbits;
	unsigned char numericalView;

	numericalView = (unsigned char) view;		// convert from enum to number

	if(stepLED > 4)
	{
		stepLED -= 4;							// make 5-8, 1-4
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED - 1);				// shift operation bit into correct position
		if(on)
		{
			LEDstepsR3[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				LEDstepsR3Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				LEDstepsR3Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			LEDstepsR3[numericalView] &= ~LEDbits;			// clear the LED bit
			LEDstepsR3Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
	else
	{
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED - 1);
		if(on)
		{
			LEDstepsR2[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				LEDstepsR2Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				LEDstepsR2Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			LEDstepsR2[numericalView] &= ~LEDbits;			// clear the LED bit
			LEDstepsR2Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
}

void genDelayFunc(unsigned int counts)
{
	genDelay.i = counts;
	// avoids problem with change in the background while checking.
	while (genDelay.d[1] | genDelay.d[0]);
}

/****************************************************************************
*
* Function: pollSwitches
*
* Description: This routine is called every 10ms to check for changes in the
*		switches. If a change is detected from the last time this routine
*		was called, the context for the switch is updated.
*
****************************************************************************/
void pollSwitches(void)
{
	unsigned char shiftbits, diffbits;
	unsigned char s;

	// Port D are the step switches
	currentPortD = PORTD;					// read PORTD only once to avoid discontinuity in reading
											// if switch is bouncing
	if(currentPortD != previousPortD)
	{
		diffbits = currentPortD ^ previousPortD;
		previousPortD = currentPortD;
		shiftbits = 1;
		for(s = 1; s < 9; s++)
		{
			if((diffbits & shiftbits) == 0)		// shortcut to reloop, only detect changing switches
			{
				shiftbits = shiftbits << 1;
				continue;
			}
			if((currentPortD & shiftbits) == 0)	// only recognize switch release per Wes
			{
				// update depending on view
				switch(currentView)
				{
					case VIEWGRID:
						if(s <= patternSetting)
						{
							if(enabledStep[s] == FALSE)
							{
								// turn on step
								setStepLED(VIEWGRID, s, ON, DIM);
								enabledStep[s] = TRUE;
							}
							else
							{
								// turn off step
								setStepLED(VIEWGRID, s, OFF, NORMAL);
								enabledStep[s] = FALSE;
								// if switching off at current step
								if(currentStep == s && runMode)
								{
									setPatternOut(OFF);
									setStepOut(currentStep, OFF);
								}
							}
						}
						break;
					case VIEWLOOP:
						loopSetting = s;
						displayLoop();
						break;
					case VIEWPATTERN:
						patternSetting = s;

						// although unverified, reports of lights being on after pattern
						//  has changed.  Probably because clock pulse comes along between
						//  when the pattern is being selected and the display being updated.
						// therefore, clock pulse is disabled until finished.
						_asm
						BCF INTCON,7,0 			// disable high priority interrups
						_endasm

						displayPattern();
						displayGrid();			// changing the pattern will affect which steps are enabled

						_asm
						BSF INTCON,7,0 			// enable high priority interrups
						_endasm

						break;
					case VIEWGATE:
						if(gateSetting[s] == FALSE)
						{
							// turn on gate
							setStepLED(VIEWGATE, s, ON, DIM);
							gateSetting[s] = TRUE;
						}
						else
						{
							// turn off gate -- trigger mode
							setStepLED(VIEWGATE, s, OFF, NORMAL);
							gateSetting[s] = FALSE;
						}
						break;
				}
			}
			shiftbits = shiftbits << 1;
		}
	}
	// Port A are the status/mode switches
	currentPortA = PORTA & 0x0f;
	if(currentPortA != previousPortA)
	{
		diffbits = currentPortA ^ previousPortA;
		previousPortA = currentPortA;
		shiftbits = 1;
		for(s = 0; s < 4; s++)
		{
			if((diffbits & shiftbits) == 0)		// shortcut to reloop, only detect changing switches
			{
				shiftbits = shiftbits << 1;
				continue;
			}
			if(currentPortA & shiftbits)
			{
				// switch is now on
				switch(s)
				{
					case 0:				// GATE switch on
						switchGateHoldTimer = HOLD_THRESHOLD;
						break;
					case 1:				// SEQ switch on
						switchSeqHoldTimer = HOLD_THRESHOLD;
						break;
					case 2:	  			// LOOP switch on
						switchLoopHoldTimer = HOLD_THRESHOLD;
						break;
				}
			}
			else
			{
				// switch is now off
				if((ignoreRelease & shiftbits) == 0)		// see else below
				{
					// ignoreRelease is off, which means hold timer didn't expire
					//  check view.  ignoreRelease should never be set for Start/stop
					switch(s)
					{
						case 0:			// Trig Mode
							switchGateHoldTimer = 0;		// clear the timer
							if(currentView == VIEWGATE)
							{
								changeView(VIEWGRID);
							}
							else
							{
								if(gateMode == TRUE)
								{
									gateMode = FALSE;
									setStatusLED(LEDGATE, OFF, FALSE);
								}
								else
								{
									gateMode = TRUE;
									setStatusLED(LEDGATE, ON, FALSE);
								}
							}
							break;
						case 1:			// Rand Mode
							switchSeqHoldTimer = 0;			// clear the timer
							if(currentView == VIEWPATTERN)
							{
								changeView(VIEWGRID);
							}
							else
							{
								if(seqMode == TRUE)
								{
									seqMode = FALSE;
									setStatusLED(LEDSEQ, OFF, FALSE);
								}
								else
								{
									seqMode = TRUE;
									setStatusLED(LEDSEQ, ON, FALSE);
								}
							}
							break;
						case 2:			// End Mode
							switchLoopHoldTimer = 0;		// clear the timer
							if(currentView == VIEWLOOP)
							{
								changeView(VIEWGRID);
							}
							else
							{
								if(loopMode == TRUE)
								{
									loopMode = FALSE;
									setStatusLED(LEDLOOP, OFF, FALSE);
								}
								else
								{
									loopMode = TRUE;
									setStatusLED(LEDLOOP, ON, FALSE);
								}
							}
							break;
						case 3:				// START switch is now released
							if(startMode)
							{
								stop();
							}
							else
							{
								start();
								if(linkMode != LINKNONE)
									linkStopAllRequest = TRUE;
							}
							break;
					}
				}
				else
				{
					ignoreRelease &= ~shiftbits;			// clear the ignore release flag bit
				}
			}
			shiftbits = shiftbits << 1;
		} // end of loop
	}
}

/****************************************************************************
*
* Function:  advanceStep
*
* Description: advanceStep is called by the interrupt in response to a
*		rising clock pulse being detected.
*
****************************************************************************/
void advanceStep(void)
{
	unsigned int random;
	unsigned char c;

	if(runMode == FALSE)			// check if armed
	{
		runMode = TRUE;
		sendStartPulse();
		if(linkStopAllRequest)		// already preconditioned with link mode
		{
			// this is also checked in clockLow
			sendLinkStopPulse();	// this will stop any other linked modules
			linkStopAllRequest = FALSE;
		}
	}

	// stepCounter and loopCounter are advanced on Clock Low

	if(seqMode)
	{
		currentStep = stepCounter;
	}
	else	// random mode
	{
		// changing the seed doesn't change the pattern of 3 bits, it still
		// repeats every 64 rands.  All changing the seed does is change where in
		// in the pattern rand() starts.  So a read from TMR0 is used to shake
		// things up.
		random = rand();					// get random integer 0 - 32768
		random = random >> 3;
		c = random;
		c ^= TMR0L;							// XOR with timer 0
		c &= 0x07;							// clear all but 3 bits, 0-7
		c = c % patternSetting;				// modulo
		currentStep = c + 1;
	}

	if(enabledStep[currentStep])
	{
		setPatternOut(TRUE);
		if(!gateMode)
		{										// in trigger mode
			patternPulseTimer = TRIGGER_PULSE;	// set output low after time delay
		}
		// if the current step is enabled, LED full brightness
		setStepLED(VIEWGRID, currentStep, ON, NORMAL);
		// individual step output
		setStepOut(currentStep, ON);
	}
	else
	{
		// if the current step is disabled, LED is dimmed
		setStepLED(VIEWGRID, currentStep, ON, DIM);
		// if sequence mode, output even if step is off.  Skip if Random mode.
		if(seqMode == TRUE)
			setStepOut(currentStep, ON);
	}

	// pattern and gate view are also turned on
	setStepLED(VIEWPATTERN, currentStep, ON, NORMAL);
	setStepLED(VIEWGATE, currentStep, ON, NORMAL);
	// loop view
	setStepLED(VIEWLOOP, loopCounter, ON, NORMAL);

	// timers are set regardless.  Check in timer ISR to shut off LEDs
	LEDTriggerTimer = MIN_LED_PULSE;
	triggerPulseTimer = TRIGGER_PULSE;

}

/****************************************************************************
*
* Function: clockLow
*
* Description: This routine is the opposite of the routine above (advanceStep).
*		It is called when the clock input is detected low by the interrupt.
*		It performs the house keeping functions of turning off outputs
*		and checking for the end/loop mode.
*
****************************************************************************/
void clockLow(void)
{
	// turn off electrical outputs
	setPatternOut(OFF);
	setStepOut(currentStep, OFF);

	// turn off gate step LED or go back to DIM.
	setStepLED(VIEWGATE, currentStep, gateSetting[currentStep], DIM);

	// clean up display if pattern setting changed
	if(currentStep <= patternSetting)
	{
		setStepLED(VIEWPATTERN, currentStep, ON , DIM);
		setStepLED(VIEWGRID, currentStep, enabledStep[currentStep], enabledStep[currentStep]);
	}
	else
	{
		setStepLED(VIEWPATTERN, currentStep, OFF , NORMAL);
		setStepLED(VIEWGRID, currentStep, OFF, NORMAL);
	}

	// clean up display if loop setting changed
	if(loopCounter <= loopSetting)
		setStepLED(VIEWLOOP, loopCounter, ON, DIM);
	else
		setStepLED(VIEWLOOP, loopCounter, OFF, NORMAL);

	// determine the next step, so we know whether to stop here
	stepCounter++;
	if(stepCounter > patternSetting || resetRequest)
	{
		// end/loop mode is checked on falling edge of clock
		stepCounter = 1;
		loopCounter++;
		if(loopCounter > loopSetting || resetRequest)
		{
			resetRequest = FALSE;
			loopCounter = 1;
			if(loopMode)
			{
				// loop mode won't stop on endRequest unless it is in link mode.
				if(linkRole != LINKNONE)	// &&&
				{
					sendLinkPulse();
					stop();
				}
			}
			else		// end mode
			{
				stop();
			}
		}
	}
	// this is the preferred place to stop other modules, but is also checked
	// in advanceStep
	if(linkStopAllRequest)		// already preconditioned with link mode
	{
		sendLinkStopPulse();	// this will stop any other linked modules
		linkStopAllRequest = FALSE;
	}
}

/****************************************************************************
*
* Function:  start
*
* Description:  Starts the Algorhythm
*
****************************************************************************/
void start(void)
{
	startMode = TRUE;
	setStatusLED(LEDSTART, ON, FALSE);	// LED on
	// sendStartPulse();		deferred until first clock
	setSeed();					// set random number seed
}

/****************************************************************************
*
* Function:  stop
*
* Description:  Stops the Algorhythm and turns off any output and LEDs
*
****************************************************************************/
void stop(void)
{
	startMode = FALSE;
	runMode = FALSE;
	setStatusLED(LEDSTART, OFF, FALSE);
	if(currentStep <= patternSetting)
	{
		setStepLED(VIEWPATTERN, currentStep, ON , DIM);
		setStepLED(VIEWGRID, currentStep, enabledStep[currentStep], enabledStep[currentStep]);
	}
	else
	{
		setStepLED(VIEWPATTERN, currentStep, OFF , NORMAL);
		setStepLED(VIEWGRID, currentStep, OFF, NORMAL);
	}
	// clean up display if loop setting changed
	if(loopCounter <= loopSetting)
		setStepLED(VIEWLOOP, loopCounter, ON, DIM);
	else
		setStepLED(VIEWLOOP, loopCounter, OFF, NORMAL);

	setPatternOut(OFF);
	setStepOut(currentStep, OFF);
	currentStep = 0;
	stepCounter = 1;
	loopCounter = 1;
	resetRequest = FALSE;		// clear any pending resets
	sendStopPulse();
}

/****************************************************************************
*
* Function:  setPatternOut
*
* Description:  Sets the signal state on the pattern output jack.
*
****************************************************************************/
void setPatternOut(unsigned char on)
{
	if(on)
	{
		LATF |= 0x02;
	}
	else
	{
		LATF &= ~0x02;
	}
}

/****************************************************************************
*
* Function:  setStepOut
*
* Description: Sets the signal state on the selected Step output jack
*
****************************************************************************/
void setStepOut(unsigned char step, unsigned char on)
{
	unsigned char shiftbits = 1;
	unsigned char c;
	shiftbits = shiftbits << (step - 1);
	if(on)
		LATC |= shiftbits;
	else
		LATC &= ~shiftbits;
}

/****************************************************************************
*
* Function:  sendLinkStopPulse
*
* Description: Turns on the Stop All link output and starts the timer
*		that shuts it off.
*
****************************************************************************/
void sendLinkStopPulse(void)
{
	TRISFbits.TRISF6 = 0;
	LATFbits.LATF6 = 1;
	linkPulseTimer = LINK_PULSE; 	// timer interrupt will turn off pulse
}

/****************************************************************************
*
* Function:  sendLinkPulse
*
* Description: Turns on the link output and starts the timer that shuts it off.
*
****************************************************************************/
void sendLinkPulse(void)
{
	if(linkOutMinFlag == TRUE)
	{
		// there's been an overrun of the link pulse, pause until minimum
		//  logic 0 pulse has been sent.  Cleared in timer routine.
		clockDisableFlag = TRUE;
	}
	else
	{
		if(linkRole == LINKLAST)
		{
			LATF |= 0x20;					// linkback signal to first module
		}
		else
		{
			LATF |= 0x10;					// link signal to downstream module
		}
		linkPulseTimer = LINK_PULSE; 		// timer interrupt will turn off pulse
	}
}


/****************************************************************************
*
* Function:  sendStartPulse(void)
*
* Description:  Turns on the state output and starts the timer that shuts it off.
*
****************************************************************************/
void sendStartPulse(void)
{
	LATF |= 0x04;						// turn on pulse
	startPulseTimer = TRIGGER_PULSE; 	// timer interrupt will turn off pulse
}

/****************************************************************************
*
* Function:  sendStatePulse(void)
*
* Description:  Turns on the state output and starts the timer that shuts it off.
*
****************************************************************************/
void sendStopPulse(void)
{
	LATF |= 0x08;						// turn on pulse
	stopPulseTimer = TRIGGER_PULSE; 	// timer interrupt will turn off pulse
}

/****************************************************************************
*
* Function: initPowerFail
*
* Description: Initialize the comparator feature to generate an interrupt
*		when the unregulated power supply (+12) starts to fall.  The
*		interrupt will begin context saving.
*
****************************************************************************/
void initPowerFail(void)
{
	// program the comparator voltage reference module
	CVRCON = 0b10011010;		// set CVref to 4.0625V
	CM3CON = 0b10001100;		// enable comparator and interrupt on low to
								//  high transition
	IPR6 |= 0x04;				// set high priority
	PIR6 &= ~0x04;				// ensure IR flag is clear
	PIE6 |= 0x04;				// enable interrupt
}

/****************************************************************************
*
* Function: initLinkMode
*
* Description: Read the status of the link cable(s) and determine this
*		module's role in link mode.  Make adjustments to the IO pin funtions
*		based on the role.
*
****************************************************************************/
void initLinkMode(void)
{
	linkRole = LINKNONE;		// default
	// link connections are active low
	if(PORTG & 0x01)			// check upstream connection
	{
		// no upstream connection
		if(PORTG & 0x02)		// check downstream connection
		{
			linkRole = LINKNONE;	// no connections either way
		}
		else
		{
			linkRole = LINKMASTER;	// downstream connection makes this a Master
		}
	}
	else
	{
		// there is an upstream connection
		if(PORTG & 0x02)
		{
			linkRole = LINKLAST;	// no downstream connection
			TRISF &= ~0x20;			// make LINKBACK an output
		}
		else
		{
			linkRole = LINKMIDDLE;	// both upstream and downstream connection
		}
	}
}

/****************************************************************************
*
* Function: changeView
*
* Description: Changes the current view and updates the status LEDs
*
****************************************************************************/
void changeView(enum _view newView)
{
	if(currentView != newView)		// only if they are not the same
	{

		switch(currentView)			// reset LED on previous view to reflect mode
		{
			case VIEWGRID:
				break;
			case VIEWLOOP:
				setStatusLED(LEDLOOP, loopMode, FALSE);
				break;
			case VIEWPATTERN:
				setStatusLED(LEDSEQ, seqMode, FALSE);
				break;
			case VIEWGATE:
				setStatusLED(LEDGATE, gateMode, FALSE);
				break;
		}
		switch(newView)
		{
			case VIEWGRID:
				break;
			case VIEWLOOP:
				setStatusLED(LEDLOOP, TRUE, TRUE);
				break;
			case VIEWPATTERN:
				setStatusLED(LEDSEQ, TRUE, TRUE);
				break;
			case VIEWGATE:
				setStatusLED(LEDGATE, TRUE, TRUE);
				break;
		}
		currentView = newView;		// update the view
	}
}

/****************************************************************************
*
* Function: displayGrid
*
* Description: Updates the Grid view based on the enabled steps.  Doesn't
*		change view if not in Grid view.
*
****************************************************************************/
void displayGrid(void)
{
	unsigned char it;
	for(it = 1; it < 9; it++)
	{
		if(it <= patternSetting)
			// turn on or off depending on enabledStep
			setStepLED(VIEWGRID, it, enabledStep[it], DIM);
		else
			setStepLED(VIEWGRID, it, OFF, NORMAL);
	}
}


/****************************************************************************
*
* Function:	 displayLoop
*
* Description:  Displays the current loop setting in LOOPVIEW. Doesn't change
*		the view if not in loop view.
*
****************************************************************************/
void displayLoop(void)
{
	unsigned char it;
	for(it = 1; it < 9; it++)
	{
		if(it <= loopSetting)
			setStepLED(VIEWLOOP, it, ON, DIM);
		else
			setStepLED(VIEWLOOP, it, OFF, NORMAL);
	}
}


/****************************************************************************
*
* Function: displayPattern
*
* Description:	Displays the pattern length setting in Pattern view mode.
*
****************************************************************************/
void displayPattern(void)
{
	unsigned char it;
	for(it = 1; it < 9; it++)
	{
		if(it <= patternSetting)
			setStepLED(VIEWPATTERN, it, ON, DIM);
		else
			setStepLED(VIEWPATTERN, it, OFF, NORMAL);
	}
}

/****************************************************************************
*
* Function: displayGate
*
* Description:	updates the gate/trig view
*
****************************************************************************/
void displayGate(void)
{
	unsigned char it;
	for(it = 1; it < 9; it++)
	{
		// turn on or off depending on gateSetting
		setStepLED(VIEWGATE, it, gateSetting[it], DIM);
	}
}


/****************************************************************************
*
* Function:  pollLinkSignals
*
* Description:  This routine polls the various link inputs based on the role
*		of this module.
*
*		This routine needs to be called more often than
*		TRIGGER_PULSE so that pulses from other modules are not missed.
*
****************************************************************************/
void pollLinkSignals(void)
{
	if(linkRole != LINKMASTER)	// all modules except Master, listen to Link In
	{
		if(PORTGbits.RG2)
		{
			if(linkInputFlag == FALSE)
			{
				if(!startMode)
					start();
				linkInputFlag = TRUE;
			}
		}
		else
		{
			linkInputFlag = FALSE;
		}
	}
	else	// Link master only listens to linkback
	{
		if(PORTFbits.RF5)
		{
			if(linkInputFlag == FALSE)
			{
				if(!startMode)
					start();
				linkInputFlag = TRUE;
			}
		}
		else
		{
			linkInputFlag = FALSE;
		}
	}
	if(TRISFbits.TRISF6)		// if the stop all is currently an input, listen to it.
								// otherwise, ignore
	{
		if(PORTFbits.RF6)
		{
			if(linkStopFlag == FALSE)
			{
				linkStopFlag = TRUE;
				if(startMode)
					stop();
			}
		}
		else
		{
			linkStopFlag = FALSE;
		}
	}
}

/****************************************************************************
*
* Function: ReadEEchar
*
* Description:  Read a byte of EEPROM.  There are 1024 locations.
*
* Parameters Received: address location (0x0000 - 0x03ff)
* Parameters Returned: byte read
*
****************************************************************************/
unsigned char readEEchar(unsigned int address)
{
	EEADRH = ((address & 0xff00) >> 8);			// get upper byte of address and store
	EEADR = (unsigned char)(address & 0xff);	// get lower byte of address and store
	EECON1bits.EEPGD = 0;		// EERPOM memory selection
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;			// initiate read... self-clearing
	return EEDATA;
}

/****************************************************************************
*
* Function: ReadEEint
*
* Description:  Read an int (word) of EEPROM.  There are 1024 locations.
*
* Parameters Received: address location (0x0000 - 0x03ff)
* Parameters Returned: word read
*
****************************************************************************/
unsigned int readEEint(unsigned int address)
{
	unsigned int i;
	unsigned char c;

	i = 0;
	c = readEEchar(address++);	// read MSB
	i = c;
	c = readEEchar(address);	// read LSB
	i = i << 8;
	i = i | c;
	return i;
}

/****************************************************************************
*
* Function: WriteEEchar
*
* Description: Write a byte of EEPROM.
*
* Parameters Received: address location (0x0000 - 0x03ff)
*					   data byte to write
* Parameters Returned: none
*
****************************************************************************/
void writeEEchar(unsigned int address, unsigned char data)
{
	EEADRH = ((address & 0xff00) >> 8);			// get upper byte of address and store
	EEADR = (unsigned char)(address & 0xff);	// get lower byte of address and store
	EEDATA = data;				// store data
	EECON1bits.EEPGD = 0;		// EERPOM memory selection
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	// DI();
	PIR6bits.EEIF = 0;			// ensure clear flag
	EECON2 = 0x55;				// start sequence to begin writing
	EECON2 = 0xaa;
	EECON1bits.WR = 1;			// write has begun... self-clearing
	// EI();
	while (PIR6bits.EEIF == 0);	// wait for write to complete... could be 5 milliseconds
	EECON1bits.WREN = 0;
}

/****************************************************************************
*
* Function: WriteEEint
*
* Description:  Read an int (word) of EEPROM.  There are 1024 locations.
*
* Parameters Received: address location (0x0000 - 0x03ff)
* Parameters Returned: word read
*
****************************************************************************/
void writeEEint(unsigned int address, unsigned int data)
{
	writeEEchar(address++, ((data & 0xff00) >> 8));	// write MSB
	writeEEchar(address, (unsigned char) (data & 0xff)); // write LSB
}

/****************************************************************************
*
* Function:
*
* Description:
*
****************************************************************************/
