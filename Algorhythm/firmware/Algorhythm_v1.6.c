/****************************************************************************
*
*	Algorhythm
*   Copyright (c) Grayscale, LLC
*
*	Version History
*
*	2.1		3/11/17
*		- added OR output functions
*
*	2.0		4/21/16	- 6/26/16
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
#define DOUBLE_TAP_WINDOW		(unsigned char) 450

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
#define EEFLAG 0x5bb7

#define EEFLAG_ADDR 0x00			// 2 bytes of defined EEFLAG if programmed
#define CHAIN_ADDR 0x02
#define ORMODE64_ADDR 0x03
#define ORMODE32_ADDR 0x04
#define ORMODE16_ADDR 0x05
#define SEGMENTS_ADDR 0x06			// beginning of segment storage
// offsets within each segment
#define SEG_FLAGS_OFFSET	0x00
#define SEG_STEPS_OFFSET	0x01
#define SEG_GATES_OFFSET	0x02
#define SEG_PATTERN_OFFSET	0x03
#define SEG_LOOP_OFFSET		0x04

/****************************************************************************
*
*	Global Variables and Structs
*
****************************************************************************/

// timing variables
volatile union {
	unsigned char d[2];
	unsigned int 	i;
} genDelay;

volatile unsigned char LEDTriggerTimer;
unsigned char triggerPulseTimer;
unsigned char startPulseTimer;
unsigned char stopPulseTimer;

// LED control variables

// These variables do not hold numeric values.  They are all bitwise, using bits 0-3.
volatile unsigned char LEDstatuses;			// row 1
volatile unsigned char LEDstatusesDim;		// row 1 dim control

volatile unsigned char LEDstepsR2Flags;		// used for consistent ISR communication in case view changes
volatile unsigned char LEDstepsR3Flags;		//  between the occurance of CCP5ISR and CCP6ISR
volatile unsigned char LEDstatusesFlags;
volatile unsigned char LEDstatusesDimFlags;
volatile unsigned char LEDstepsR2DimFlags;
volatile unsigned char LEDstepsR3DimFlags;

volatile unsigned char LEDcolumn; 			// currently illuminated LED column bitwise
// numerical variables
volatile unsigned char LEDcolumnCount; 		// currently illuminated LED column numerical

enum _statusLED{					// used in parameter passing and local variables
	LEDSTART,
	LEDLOOP,
	LEDSEQ,
	LEDGATE
};

// switch checking
unsigned char currentPortD;
unsigned char previousPortD;
unsigned char currentPortA;
unsigned char previousPortA;
unsigned int switchHoldTimerA = 0;
unsigned int switchHoldTimerD = 0;
unsigned char switchHeldFlagsA = 0;
unsigned char switchHeldFlagsD = 0;
unsigned char ignoreReleaseA = 0;
unsigned char ignoreReleaseD = 0;
unsigned char switchCheckInterval;
unsigned int doubleTapTimer = 0;

enum _swcode {
	SW1,		// Order is sensitive and tied to code
	SW2,
	SW3,
	SW4,
	SW5,
	SW6,
	SW7,
	SW8,
	SW1HELD,
	SW2HELD,
	SW3HELD,
	SW4HELD,
	SW5HELD,
	SW6HELD,
	SW7HELD,
	SW8HELD,
	SWGATE,		// offset of 16 here
	SWSEQ,
	SWLOOP,
	SWSTART,
	SWGATEHELD,	// offset of 20 here
	SWSEQHELD,
	SWLOOPHELD,
	SWSTARTHELD
};

enum _swcode doubleTapSwitchcode;

// modes
volatile unsigned char startMode;
volatile unsigned char runMode;			// with startMode = 1 and runMode = 0 == armed mode
unsigned char resetRequest;

// output control
volatile unsigned char Output;

// rate limiting flags
unsigned char clockHigh = 0;
unsigned char executeClockLowFlag = 0;

enum _linkRole {
	LINKNONE,
	LINKMASTER,
	LINKMIDDLE,
	LINKLAST
} linkRole;							// still initialized to maintain hardware compatibility, but not used.

enum _segview {
	VIEWGRID,
	VIEWLOOP,
	VIEWPATTERN,
	VIEWGATE
};


enum _deviceview {
	VIEWOVERVIEW,
	VIEWCHAIN,
	VIEWMODERESPONSE,
	VIEWSEGMENT
} currentDeviceView = VIEWOVERVIEW;

unsigned char currentSegmentView;

// view storage for upper level views (which are the first three enumerations above).
//  Segment views are stored in segment struct.
unsigned char LEDdeviceStepsR2[3] = {0, 0};
unsigned char LEDdeviceStepsR3[3] = {0, 0};
unsigned char LEDdeviceStepsR2Dim[3] = {0, 0};
unsigned char LEDdeviceStepsR3Dim[3] = {0, 0};

enum _chain {
	C1X64,
	C2X32,
	C4X16,
	C8X8
} chainMode = C1X64;

unsigned char ORmode64;
unsigned char ORmode32;
unsigned char ORmode16;

// to save on memory, will use some bit fields for boolean flags
struct _segmentFlags {
	unsigned gateMode: 1;
	unsigned muted: 1;
	unsigned active: 1;
	unsigned seqMode: 1;
	unsigned loopMode: 1;
	unsigned modeResponse: 1;
};

// put the segment data, since it's large, into a different bank.
// the linker handles this without a special link script
// the order of elements is important.  Items at the beginning of the struct are saved in non-volatile memory.
// changes made here must also be made to EEPROM addresses/addressing.
#pragma udata bankA
struct _segment {
	struct _segmentFlags flags;
	unsigned char enabledStep;		// bitwise boolean values
	unsigned char gateSetting;		// bitwise boolean values
	unsigned char patternSetting;
	unsigned char loopSetting;
#define SAVESIZE	5
	// 5 bytes above here are stored in non-volatile memory
	unsigned char nextSegment;
	unsigned char currentStep;
	unsigned char stepCounter;
	unsigned char loopCounter;
	enum _segview currentView;
	unsigned char LEDstepsR2[4];		// row 2 (size is the number of segment views)
	unsigned char LEDstepsR3[4];		// row 3
	unsigned char LEDstepsR2Dim[4];		// row 2 dim
	unsigned char LEDstepsR3Dim[4];		// row 3 dim

} segment[8];
#pragma udata


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
void genDelayFunc(unsigned int counts);
void initialize(void);
void initT0(void);
void setSeed(void);
void initT3(void);
void initT5(void);
void initPortBInterrupts(void);
void initIO(void);
void initNonVolatileValues(void);
void saveSetting(unsigned char segmentNumber, unsigned char offset, unsigned char value);
void saveSettingFlags(unsigned char segmentNumber);
void saveSettings(void);
void CCP4ISR(void);
void CCP5ISR(void);
void CCP6ISR(void);
void clockISR(void);
void modeISR(void);
void resetISR(void);
void stateISR(void);
void setStatusLED(enum _statusLED sLED, unsigned char on, unsigned char dim);
void setStepLEDSegment(unsigned char localSegment, enum _segview view, unsigned char stepLED, unsigned char on, unsigned char dim);
void setStepLEDDevice(enum _deviceview view, unsigned char stepLED, unsigned char on, unsigned char dim);
void pollSwitches(void);
void onSwitch(enum _swcode switchcode);
void changeGridStep(unsigned char step);
void changeLoopSetting(unsigned char length);
void changePatternSetting(unsigned char pattern);
void changeGateSetting(unsigned char step);
void changeMuteSetting(unsigned char lSegment);
void changeModeResponse(unsigned char lSegment);
void changeChainSetting(enum _swcode chaining);
void changeORsetting(enum _swcode OR);
void advanceStep(void);
void clockLow(void);
void start(void);
void stop(void);
void resetChains(void);
void updateOutputs(void);
void setSegmentOut(unsigned char lSegment, unsigned char on);
void setOutputsAllOff(void);
void sendStartPulse(void);
void sendStopPulse(void);
void initPowerFail(void);
void changeSegmentView(enum _segview newView);
void displayGrid(unsigned char segmentNumber);
void displayLoop(unsigned char segmentNumber);
void displayPattern(unsigned char segmentNumber);
void displayGate(unsigned char segmentNumber);
void displayChain(void);
void displayOverview(void);
void updateOverview(void);
void displayModeResponse(void);
void displaySegment(unsigned char segmentNumber);
unsigned char readEEchar(unsigned int address);
unsigned int readEEint(unsigned int address);
void writeEEchar(unsigned int address, unsigned char data);
void writeEEint(unsigned int address, unsigned int data);
void initLinkMode(void);
unsigned char testBit(unsigned char dataByte, unsigned char bitNumber);
void setBit(unsigned char *dataByte, unsigned char bitNumber, unsigned char bitValue);


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
//	if(PIR6bits.CMP3IF)
//		saveSettings();
//	if(INTCONbits.INT0IF)
//		clockISR();
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
* Description:  Main loop. The clock routines were moved here from the high
*		priority interrupts.
*
****************************************************************************/

void main(void)
{
	initialize();
	while(1)
	{
		if((PORTB & 1) && clockHigh == FALSE)
		{
			clockHigh = TRUE;
			if(startMode)
				advanceStep();
		}
		if((PORTB & 1) == 0 && clockHigh == TRUE)
		{
			clockHigh = FALSE;
			if(runMode)
				clockLow();
		}
	}

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

void genDelayFunc(unsigned int counts)
{
	genDelay.i = counts;
	// avoids problem with change in the background while checking.
	while (genDelay.d[1] | genDelay.d[0]);
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
//	initPowerFail();	// intiallize the comparator for power fail detection

	// Map the timers
	CCPTMRS1 = 0b00010101; 	// Both CCP5 and CCP6 will use Timer 5
							// CCP4 will use Timer 3

	// init the timers
	initT0();			// initialize timer 0 for random number seed generation
	initT5();			// initialize timer 5/CCP5/CCP6 for LED control
	initT3(); 			// initialize timer 3/CCP4 for general purposes
						//  software timers

	initLinkMode();		// link deprecated in V2.  Used only to init signals.

	// check to see if the contents of EEPROM are valid.  If yes, read in stored settings.
	//  If not, initialize to default values
	initNonVolatileValues();

	previousPortA = 0x00;	// initialize for pollSwithes
	previousPortD = 0x00;

	stop();					// this will take care of a lot of initialization
	initPortBInterrupts();

	EI();
	displayOverview();

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
	triggerPulseTimer = 0;
	startPulseTimer = 0;
	stopPulseTimer = 0;
	switchCheckInterval = SWITCH_CHECK_INTERVAL;
	switchHoldTimerA = 0;
	switchHoldTimerD = 0;

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
	LEDstatusesDim = 0;
	LEDcolumn = 1;
	LEDcolumnCount = 0;

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
/*
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
*/
	INTCON &= ~0x12;
	// the other interrups 1,2 and 3 are rising edge triggered and low priority

	INTCON2 |= 0b00111000;
	INTCON2 &= 0b11111101;	// INT1 has low priority
	INTCON3 = 0b00111000;	// enable 1 2 and 3, low priority
	resetRequest = FALSE;
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

	// Link mode is not supported by this firmware, but signals should still be initialized to a default state

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
	unsigned int flag, address;
	unsigned char j, k, segsize;
	unsigned char *index;

	flag = readEEint(EEFLAG_ADDR);
	if(flag != EEFLAG)		// should change this if the storage scheme changes
	{
		// factory defaults for first time power up

		// chain mode already intialized in declaration.
		changeChainSetting(C1X64);		// initializes nextSegment
		ORmode64 = 0x01;
		ORmode32 = 0x11;
		ORmode16 = 0x55;

		for(j = 0; j < 8; j++)
		{
			segment[j].enabledStep = 0xff;			// set all steps enabled
			segment[j].gateSetting = 0xff;
			segment[j].patternSetting = 7;
			segment[j].loopSetting = 0;
			segment[j].flags.gateMode = TRUE;
			segment[j].flags.muted = FALSE;
			segment[j].flags.seqMode = TRUE;
			segment[j].flags.loopMode = TRUE;
			segment[j].flags.modeResponse = TRUE;
			if(j == 0)
				segment[j].flags.active = TRUE;
			else
				segment[j].flags.active = FALSE;
			// init each view to correspond with above
			for(k = 0; k < 4; k++)
			{
				segment[j].LEDstepsR2[k] = 0x0f;
				segment[j].LEDstepsR3[k] = 0x0f;
				segment[j].LEDstepsR2Dim[k] = 0x0f;
				segment[j].LEDstepsR3Dim[k] = 0x0f;
			}
			segment[j].currentView = VIEWGRID;
		}
		saveSettings();
	}
	else
	{
		// read saved values from EEPROM

		chainMode = readEEchar(CHAIN_ADDR);
		ORmode64 = readEEchar(ORMODE64_ADDR);
		ORmode32 = readEEchar(ORMODE32_ADDR);
		ORmode16 = readEEchar(ORMODE16_ADDR);

		address = SEGMENTS_ADDR;
		segsize = SAVESIZE;
		for(j = 0; j < 8; j++)
		{
			index = (unsigned char *) &segment[j];
			for(k = 0; k < segsize; k++)
			{
				*index++ = readEEchar(address++);
			}
		}
	}
	// initialize views based on settings
	switch(chainMode)
	{
		case C1X64:
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 2;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 4;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 6;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 0;
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 0;
			segment[3].flags.active = 0;
			segment[4].flags.active = 0;
			segment[5].flags.active = 0;
			segment[6].flags.active = 0;
			segment[7].flags.active = 0;
			break;
		case C2X32:
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 2;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 0;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 6;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 4;
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 0;
			segment[3].flags.active = 0;
			segment[4].flags.active = 1;
			segment[5].flags.active = 0;
			segment[6].flags.active = 0;
			segment[7].flags.active = 0;

			break;
		case C4X16:
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 0;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 2;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 4;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 6;
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 1;
			segment[3].flags.active = 0;
			segment[4].flags.active = 1;
			segment[5].flags.active = 0;
			segment[6].flags.active = 1;
			segment[7].flags.active = 0;
			break;
		case C8X8:
			segment[0].nextSegment = 0;
			segment[1].nextSegment = 1;
			segment[2].nextSegment = 2;
			segment[3].nextSegment = 3;
			segment[4].nextSegment = 4;
			segment[5].nextSegment = 5;
			segment[6].nextSegment = 6;
			segment[7].nextSegment = 7;
			segment[0].flags.active = 1;
			segment[1].flags.active = 1;
			segment[2].flags.active = 1;
			segment[3].flags.active = 1;
			segment[4].flags.active = 1;
			segment[5].flags.active = 1;
			segment[6].flags.active = 1;
			segment[7].flags.active = 1;
			break;
	}
	for(j = 0; j < 8; j++)
	{
		if(segment[j].flags.muted)
			setStepLEDDevice(VIEWOVERVIEW, j, OFF, NORMAL);
		else
			setStepLEDDevice(VIEWOVERVIEW, j, ON, DIM);
		for(k = 0; k < 8; k++)
		{
			if(k <= segment[j].loopSetting)
				setStepLEDSegment(j, VIEWLOOP, k, ON, DIM);
			else
				setStepLEDSegment(j, VIEWLOOP, k, OFF, NORMAL);
			if(k <= segment[j].patternSetting)
				setStepLEDSegment(j, VIEWPATTERN, k, ON, DIM);
			else
				setStepLEDSegment(j, VIEWPATTERN, k, OFF, NORMAL);
			if(k <= segment[j].patternSetting)
				setStepLEDSegment(j, VIEWGRID, k, testBit(segment[j].enabledStep, k), DIM);
			else
				setStepLEDSegment(j, VIEWGRID, k, OFF, NORMAL);
			setStepLEDSegment(j, VIEWGATE, k, testBit(segment[j].gateSetting, k), DIM);
		}
	}

	// other mode initialization
	startMode = FALSE;
	runMode = FALSE;
	// Stop is called later in initialization, which will take care of things like
	// currentStep, loopCounter, etc.

}

/****************************************************************************
*
* Function: saveSetting
*
* Description: Save a single setting for non-volatile storage.
*
****************************************************************************/
void saveSetting(unsigned char segmentNumber, unsigned char offset, unsigned char value)
{
	unsigned int address;
	address = SEGMENTS_ADDR + offset + (segmentNumber * SAVESIZE);
	writeEEchar(address, value);
}

/****************************************************************************
*
* Function: saveSettingFlags
*
* Description: Save the segment flags for non-volatile storage.
*
****************************************************************************/
void saveSettingFlags(unsigned char segmentNumber)
{
	unsigned int address;
	unsigned char * flagsptr;
	flagsptr = (unsigned char *) &segment[segmentNumber].flags;
	address = SEGMENTS_ADDR + SEG_FLAGS_OFFSET  + (segmentNumber * SAVESIZE);
	writeEEchar(address, *flagsptr);
}

/****************************************************************************
*
* Function: saveSettings
*
* Description: Save all settings in non-volatile memory
*
****************************************************************************/
void saveSettings(void)
{
	unsigned int flag, address;
	unsigned char j, k, segsize;
	unsigned char *index;

	writeEEchar(CHAIN_ADDR, chainMode);
	writeEEchar(ORMODE64_ADDR, ORmode64);
	writeEEchar(ORMODE32_ADDR, ORmode32);
	writeEEchar(ORMODE16_ADDR, ORmode16);

	address = SEGMENTS_ADDR;
	segsize = SAVESIZE;
	for(j = 0; j < 8; j++)
	{
		index = (unsigned char *) &segment[j];	// index points to beginning of segment
		for(k = 0; k < segsize; k++)
		{
			writeEEchar(address++, *index++);
		}
	}
	writeEEint(EEFLAG_ADDR, EEFLAG);
//	PIR6 &= ~0x04;			// clear interrupt flag
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
	unsigned char s, shiftbit, i, gateModeFlag;
	// clear interrupt flag
	PIR4bits.CCP4IF = 0;

	if(LEDTriggerTimer != 0)
	{
		LEDTriggerTimer--;
		if(LEDTriggerTimer == 0)
		{
			shiftbit = 1;
			for(i = 0; i < 8; i++)
			{
				if(segment[i].flags.gateMode == FALSE && testBit(segment[i].gateSetting, segment[i].currentStep) == FALSE)
				{
					if(segment[i].currentStep <= segment[i].patternSetting)
					{
						setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, testBit(segment[i].enabledStep, segment[i].currentStep), DIM);
						setStepLEDSegment(i, VIEWGATE, segment[i].currentStep, testBit(segment[i].gateSetting, segment[i].currentStep), DIM);
						setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, ON, DIM);
					}
					else
					{
						setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, OFF, NORMAL);
						setStepLEDSegment(i, VIEWGATE, segment[i].currentStep, OFF, NORMAL);
						setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, OFF, NORMAL);
					}
					if(segment[i].loopCounter <= segment[i].loopSetting)
						setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, ON, DIM);
					else
						setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, OFF, NORMAL);
				}
				if((LATC & shiftbit) == 0)
				{
					// output is off, turn off or dim LED based on muting
					if(segment[i].flags.muted)
						setStepLEDDevice(VIEWOVERVIEW, i, OFF, NORMAL);
					else
						setStepLEDDevice(VIEWOVERVIEW, i, ON, DIM);
				}
				shiftbit = shiftbit << 1;
			}
		}
	}

	if(triggerPulseTimer != 0)
	{
		triggerPulseTimer--;
		if(triggerPulseTimer == 0)
		{
			for(i = 0; i < 8; i++)
			{
				if(segment[i].flags.gateMode == FALSE && testBit(segment[i].gateSetting, segment[i].currentStep) == FALSE)
					setSegmentOut(i, OFF);
			}
			updateOutputs();
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

	if(genDelay.i != 0)
		genDelay.i--;

	switchCheckInterval--;
	if(switchCheckInterval == 0)
	{
		pollSwitches();
		switchCheckInterval = SWITCH_CHECK_INTERVAL;
	}

	if(switchHoldTimerA != 0)
	{
		switchHoldTimerA--;
		if(switchHoldTimerA == 0)
		{
			shiftbit = 1;
			for(s = 0; s < 4; s++)
			{
				if(shiftbit & switchHeldFlagsA)
				{
					ignoreReleaseA |= shiftbit;
					switchHeldFlagsA = 0;
					onSwitch((unsigned char)(s + 20));
					break;				// stop on first switch held
				}
				shiftbit = shiftbit << 1;
			}
		}
	}

	if(switchHoldTimerD != 0)
	{
		switchHoldTimerD--;
		if(switchHoldTimerD == 0)
		{
			shiftbit = 1;
			for(s = 0; s < 8; s++)
			{
				if(shiftbit & switchHeldFlagsD)
				{
					ignoreReleaseD |= shiftbit;
					switchHeldFlagsD = 0;
					onSwitch((unsigned char)(s + 8));
					break;				// stop on first switch held
				}
				shiftbit = shiftbit << 1;
			}
		}
	}

	if(doubleTapTimer != 0)
	{
		doubleTapTimer--;
		if(doubleTapTimer == 0 && currentDeviceView == VIEWCHAIN)
		{
			changeChainSetting(doubleTapSwitchcode);
		}
	}

	//INTCONbits.INT0IE = 1;	 			// This routine has had a chance to
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
		// reload the current view
		LEDstatusesFlags = LEDstatuses;
		LEDstatusesDimFlags = LEDstatusesDim;
		if(currentDeviceView == VIEWSEGMENT)
		{
			LEDstepsR2Flags = segment[currentSegmentView].LEDstepsR2[(unsigned char)segment[currentSegmentView].currentView];
			LEDstepsR3Flags = segment[currentSegmentView].LEDstepsR3[(unsigned char)segment[currentSegmentView].currentView];
			LEDstepsR2DimFlags = segment[currentSegmentView].LEDstepsR2Dim[(unsigned char)segment[currentSegmentView].currentView];
			LEDstepsR3DimFlags = segment[currentSegmentView].LEDstepsR3Dim[(unsigned char)segment[currentSegmentView].currentView];
		}
		else
		{
			LEDstepsR2Flags = LEDdeviceStepsR2[(unsigned char)currentDeviceView];
			LEDstepsR3Flags = LEDdeviceStepsR3[(unsigned char)currentDeviceView];
			LEDstepsR2DimFlags = LEDdeviceStepsR2Dim[(unsigned char)currentDeviceView];
			LEDstepsR3DimFlags = LEDdeviceStepsR3Dim[(unsigned char)currentDeviceView];
		}
	}
	else
	{
		LEDcolumn = LEDcolumn << 1;			// shift left to next column
	}
	LEDport = LEDcolumn;					// LEDs are preset off

	if((LEDstatusesFlags & LEDcolumn) != 0)
	{
		LEDport |= 0x40;
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
	if((LEDstatusesDimFlags & LEDcolumn) != 0)
	{
		LATE &= ~0x40;
	}
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
/*
	// the clock input interrupt INT0 is triggered on both edges
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
			//clockLow();
			executeClockLowFlag = TRUE;		// clockLow moved to forground and out of this high priority interrupt
		}
		clockFlag = 0;
	}
	INTCONbits.INT0IF = 0;		// clear interrupt flag
*/
}

/****************************************************************************
*
* Function:	modeISR
*
* Description: Mode input interrupt service routine.  This is triggered on the
*		rising edge of the mode signal
*
****************************************************************************/
void modeISR(void)
{
	unsigned char c;
	INTCON3bits.INT1IF = 0;		// clear interrupt flag

	for(c = 0; c < 8; c++)
	{
		if(segment[c].flags.modeResponse == TRUE)
		{
			if(segment[c].flags.seqMode == TRUE)
			{
				segment[c].flags.seqMode = FALSE;
			}
			else
			{
				segment[c].flags.seqMode = TRUE;
			}
			if(currentDeviceView == VIEWSEGMENT && currentSegmentView == c && segment[c].currentView != VIEWPATTERN)
			{
				setStatusLED(LEDSEQ, segment[c].flags.seqMode, FALSE);
			}
		}
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
	unsigned char i;
	INTCON3bits.INT2IF = 0;		// clear interrupt flag

	if(startMode == FALSE)
		return;					// ignore if not running. It will reset
								//  when it starts
	resetRequest = TRUE;
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

void setStatusLED(enum _statusLED sLED, unsigned char on, unsigned char dim)
{
	unsigned char LEDbits;
	unsigned char c;
	// enum is 0 through 3
	LEDbits = 1;
	c = (unsigned char) sLED;
	LEDbits = LEDbits << c;
	if(on)
	{
		LEDstatuses |= LEDbits;
		if(dim)
			LEDstatusesDim |= LEDbits;
		else
			LEDstatusesDim &= ~LEDbits;
	}
	else
	{
		LEDstatuses &= ~LEDbits;
		LEDstatusesDim &= ~LEDbits;
	}
}

/****************************************************************************
*
* Function: setStepLEDSegment
*
* Description:  The step LEDs are turned on dimly if the switch is engaged
*		and turn on full brightness if the step is active.
*
* Parameters:  	view:  The view that should be updated or changed
*				stepLED: The step LED to be switched on or off (0-7)
*				on: the ON or OFF constant (1 or 0)
*				dim: DIM or NORMAL constant or TRUE or FALSE
*
****************************************************************************/
void setStepLEDSegment(unsigned char localSegment, enum _segview view, unsigned char stepLED, unsigned char on, unsigned char dim)
{
	unsigned char LEDbits;
	unsigned char numericalView;

	numericalView = (unsigned char) view;		// convert from enum to number

	if(stepLED > 3)		// Row 3
	{
		stepLED -= 4;							// make 4-7, 0-3
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED);			// shift operation bit into correct position
		if(on)
		{
			segment[localSegment].LEDstepsR3[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				segment[localSegment].LEDstepsR3Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				segment[localSegment].LEDstepsR3Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			segment[localSegment].LEDstepsR3[numericalView] &= ~LEDbits;		// clear the LED bit
			segment[localSegment].LEDstepsR3Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
	else				// Row 2
	{
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED);
		if(on)
		{
			segment[localSegment].LEDstepsR2[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				segment[localSegment].LEDstepsR2Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				segment[localSegment].LEDstepsR2Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			segment[localSegment].LEDstepsR2[numericalView] &= ~LEDbits;		// clear the LED bit
			segment[localSegment].LEDstepsR2Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
}

/****************************************************************************
*
* Function: setStepLEDDevice
*
* Description:  The step LEDs are turned on dimly if the switch is engaged
*		and turn on full brightness if the step is active.
*
* Parameters:  	view:  The view that should be updated or changed
*				stepLED: The step LED to be switched on or off (0-7)
*				on: the ON or OFF constant (1 or 0)
*				dim: DIM or NORMAL constant or TRUE or FALSE
*
****************************************************************************/
void setStepLEDDevice(enum _deviceview view, unsigned char stepLED, unsigned char on, unsigned char dim)
{
	unsigned char LEDbits;
	unsigned char numericalView;

	numericalView = (unsigned char) view;		// convert from enum to number

	if(stepLED > 3)		// Row 3
	{
		stepLED -= 4;							// make 4-7, 0-3
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED);			// shift operation bit into correct position
		if(on)
		{
			LEDdeviceStepsR3[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				LEDdeviceStepsR3Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				LEDdeviceStepsR3Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			LEDdeviceStepsR3[numericalView] &= ~LEDbits;		// clear the LED bit
			LEDdeviceStepsR3Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
	else				// Row 2
	{
		LEDbits = 1;
		LEDbits = LEDbits << (stepLED);
		if(on)
		{
			LEDdeviceStepsR2[numericalView] |= LEDbits;			// set LED on bit
			if(dim)
				LEDdeviceStepsR2Dim[numericalView] |= LEDbits;	// set dimming bit
			else
				LEDdeviceStepsR2Dim[numericalView] &= ~LEDbits;	// clear dimming bit
		}
		else
		{
			LEDdeviceStepsR2[numericalView] &= ~LEDbits;		// clear the LED bit
			LEDdeviceStepsR2Dim[numericalView] &= ~LEDbits;		// clear dimming bit
		}
	}
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

	// Port D are the numbered step switches
	currentPortD = PORTD;					// read PORTD only once to avoid discontinuity in reading
											// if switch is bouncing
	if(currentPortD != previousPortD)
	{
		diffbits = currentPortD ^ previousPortD;
		previousPortD = currentPortD;
		shiftbits = 1;
		for(s = 0; s < 8; s++)
		{
			if((diffbits & shiftbits) == 0)		// shortcut to reloop, only detect changing switches
			{
				shiftbits = shiftbits << 1;
				continue;
			}
			if((currentPortD & shiftbits) != 0)	// action is only taken with switch release or timer expiration
			{
				switchHoldTimerD = HOLD_THRESHOLD;
				switchHeldFlagsD |= shiftbits;
			}
			else		// releasing switches
			{
				switchHeldFlagsD &= ~shiftbits;
				if(shiftbits & ignoreReleaseD)
				{
					ignoreReleaseD &= ~shiftbits;			// clear the ignore release flag bit
				}
				else
				{
					if(switchHeldFlagsD == 0)	// if no other keys are held, clear timer
					{
						switchHoldTimerD = 0;
					}
					onSwitch((unsigned char) s);
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
			if((currentPortA & shiftbits) != 0)	// action is only taken with switch release or timer expiration
			{
				switchHoldTimerA = HOLD_THRESHOLD;
				switchHeldFlagsA |= shiftbits;
			}
			else		// releasing switches
			{
				switchHeldFlagsA &= ~shiftbits;
				if(shiftbits & ignoreReleaseA)
				{
					ignoreReleaseA &= ~shiftbits;			// clear the ignore release flag bit
				}
				else
				{
					if(switchHeldFlagsA == 0)	// if no other keys are held, clear timer
					{
						switchHoldTimerA = 0;
					}
					onSwitch((unsigned char) (s + 16));
				}
			}
			shiftbits = shiftbits << 1;
		}
	}
}

/****************************************************************************
*
* Function:	onSwitch
*
* Description: takes the decoded key and acts upon it.
*
****************************************************************************/
void onSwitch(enum _swcode switchcode)
{
	switch(switchcode)
	{
		case SW1:
		case SW2:
		case SW3:
		case SW4:
		case SW5:
		case SW6:
		case SW7:
		case SW8:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					changeMuteSetting((unsigned char) switchcode);
					break;
				case VIEWCHAIN:
					// VIEWCHAIN mode has a double tap feature.
					if(doubleTapTimer)
					{
						if(switchcode == doubleTapSwitchcode)
						{
							// if here, indicates a second tap of the switch
							changeORsetting(switchcode);
						}
						doubleTapTimer = 0;
					}
					else
					{
						doubleTapTimer = DOUBLE_TAP_WINDOW;
						doubleTapSwitchcode = switchcode;
					}
					break;
				case VIEWMODERESPONSE:
					changeModeResponse((unsigned char) switchcode);
					break;
				case VIEWSEGMENT:
					switch(segment[currentSegmentView].currentView)
					{
						case VIEWGRID:
							changeGridStep((unsigned char) switchcode);
							break;
						case VIEWLOOP:
							changeLoopSetting((unsigned char) switchcode);
							break;
						case VIEWPATTERN:
							changePatternSetting((unsigned char) switchcode);
							break;
						case VIEWGATE:
							changeGateSetting((unsigned char) switchcode);
							break;
					}
					break;
			}
			break;
		case SW1HELD:
		case SW2HELD:
		case SW3HELD:
		case SW4HELD:
		case SW5HELD:
		case SW6HELD:
		case SW7HELD:
		case SW8HELD:
			switch(currentDeviceView)
			{
				case VIEWSEGMENT:
				case VIEWOVERVIEW:
				case VIEWCHAIN:
					displaySegment(switchcode - 8);
				 	break;
				case VIEWMODERESPONSE:
					break; 	// do nothing
			}
			displaySegment(switchcode - 8);
			break;
		case SWGATE:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					displayChain();
					break;
				case VIEWCHAIN:
					break;
				case VIEWMODERESPONSE:
					displayChain();
					break;
				case VIEWSEGMENT:
					switch(segment[currentSegmentView].currentView)
					{
						case VIEWGATE:
							// exits gate view and returns to grid view
							displaySegment(currentSegmentView);
							break;
						default:
							if(segment[currentSegmentView].flags.gateMode == TRUE)
							{
								segment[currentSegmentView].flags.gateMode = FALSE;
								setStatusLED(LEDGATE, OFF, NORMAL);
							}
							else
							{
								segment[currentSegmentView].flags.gateMode = TRUE;
								setStatusLED(LEDGATE, ON, NORMAL);
							}
							saveSettingFlags(currentSegmentView);
							break;
					}
			}
			break;
		case SWSEQ:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					displayModeResponse();
					break;
				case VIEWCHAIN:
					displayModeResponse();
					break;
				case VIEWMODERESPONSE:
					break;
				case VIEWSEGMENT:
					switch(segment[currentSegmentView].currentView)
					{
						case VIEWPATTERN:
							// exits the pattern view and returns to grid view
							displaySegment(currentSegmentView);
							break;
						default:
							if(segment[currentSegmentView].flags.seqMode == TRUE)
							{
								segment[currentSegmentView].flags.seqMode = FALSE;
								setStatusLED(LEDSEQ, OFF, NORMAL);
							}
							else
							{
								segment[currentSegmentView].flags.seqMode = TRUE;
								setStatusLED(LEDSEQ, ON, NORMAL);
							}
							saveSettingFlags(currentSegmentView);
							break;
					}
					break;
			}
			break;
		case SWLOOP:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					break;
				case VIEWCHAIN:
				case VIEWMODERESPONSE:
					// goto Overview
					currentDeviceView = VIEWOVERVIEW;
					setStatusLED(LEDLOOP, ON, NORMAL);
					setStatusLED(LEDSEQ, ON, DIM);
					setStatusLED(LEDGATE, ON, DIM);
					break;
				case VIEWSEGMENT:
					switch(segment[currentSegmentView].currentView)
					{
						case VIEWLOOP:
							// exits the loop view and returns to grid view
							displaySegment(currentSegmentView);
							break;
						default:
							if(segment[currentSegmentView].flags.loopMode == TRUE)
							{
								segment[currentSegmentView].flags.loopMode = FALSE;
								setStatusLED(LEDLOOP, OFF, FALSE);
							}
							else
							{
								segment[currentSegmentView].flags.loopMode = TRUE;
								setStatusLED(LEDLOOP, ON, NORMAL);
							}
							saveSettingFlags(currentSegmentView);
							break;
					}
					break;
			}
			break;
		case SWSTART:
			if(startMode)
				stop();
			else
				start();
			break;
		case SWGATEHELD:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					break;
				case VIEWCHAIN:
					break;
				case VIEWMODERESPONSE:
					break;
				case VIEWSEGMENT:
					changeSegmentView(VIEWGATE);
					break;
			}
			break;
		case SWSEQHELD:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					break;
				case VIEWCHAIN:
					break;
				case VIEWMODERESPONSE:
					break;
				case VIEWSEGMENT:
					changeSegmentView(VIEWPATTERN);
					break;
			}
			break;
		case SWLOOPHELD:
			switch(currentDeviceView)
			{
				case VIEWOVERVIEW:
					break;
				case VIEWCHAIN:
					break;
				case VIEWMODERESPONSE:
					break;
				case VIEWSEGMENT:
					changeSegmentView(VIEWLOOP);
					break;
			}
			break;
		case SWSTARTHELD:
			currentDeviceView = VIEWOVERVIEW;
			setStatusLED(LEDLOOP, ON, NORMAL);
			setStatusLED(LEDSEQ, ON, DIM);
			setStatusLED(LEDGATE, ON, DIM);
			break;
	}
}

// subroutines used above

void changeGridStep(unsigned char step)
{
	void *temp;
	if(step <= segment[currentSegmentView].patternSetting)				// if the pressed switch is not within pattern, ignore
	{
		if(testBit(segment[currentSegmentView].enabledStep, step) == FALSE)
		{
			// turn on step
			setStepLEDSegment(currentSegmentView, VIEWGRID, step, ON, DIM);
			temp = &segment[currentSegmentView].enabledStep;
			setBit(&segment[currentSegmentView].enabledStep, step, TRUE);
		}
		else
		{
			// turn off step
			setStepLEDSegment(currentSegmentView, VIEWGRID, step, OFF, NORMAL);
			setBit(&segment[currentSegmentView].enabledStep, step, FALSE);
			// if switching off at current step
			if(segment[currentSegmentView].currentStep == step && runMode && segment[currentSegmentView].flags.active)
			{
				setSegmentOut(currentSegmentView, OFF);
				updateOutputs();
			}
		}
		saveSetting(currentSegmentView, SEG_STEPS_OFFSET, segment[currentSegmentView].enabledStep);
	}
}

void changeLoopSetting(unsigned char length)
{
	segment[currentSegmentView].loopSetting = length;
	displayLoop(currentSegmentView);
	saveSetting(currentSegmentView, SEG_LOOP_OFFSET, segment[currentSegmentView].loopSetting);
}

void changePatternSetting(unsigned char pattern)
{
	segment[currentSegmentView].patternSetting = pattern;

	// although unverified, reports of lights being on after pattern
	//  has changed.  Probably because clock pulse comes along between
	//  when the pattern is being selected and the display being updated.
	// therefore, clock pulse is disabled until finished.
	_asm
	BCF INTCON,7,0 			// disable high priority interrups
	_endasm

	displayPattern(currentSegmentView);
	displayGrid(currentSegmentView);			// changing the pattern will affect which steps are enabled

	_asm
	BSF INTCON,7,0 			// enable high priority interrups
	_endasm
	saveSetting(currentSegmentView, SEG_PATTERN_OFFSET, segment[currentSegmentView].patternSetting);
}

void changeGateSetting(unsigned char step)
{
	if(testBit(segment[currentSegmentView].gateSetting, step) == FALSE)
	{
		// turn on step
		setStepLEDSegment(currentSegmentView, VIEWGATE, step, ON, DIM);
		setBit(&segment[currentSegmentView].gateSetting, step, TRUE);
	}
	else
	{
		// turn off step
		setStepLEDSegment(currentSegmentView, VIEWGATE, step, OFF, NORMAL);
		setBit(&segment[currentSegmentView].gateSetting, step, FALSE);
		// if switching off at current step
		if(segment[currentSegmentView].currentStep == step
			&& runMode
			&& segment[currentSegmentView].flags.active
			&& triggerPulseTimer == 0)
		{
			setSegmentOut(currentSegmentView, OFF);
			updateOutputs();
		}
	}
	saveSetting(currentSegmentView, SEG_GATES_OFFSET, segment[currentSegmentView].gateSetting);
}

void changeMuteSetting(unsigned char lSegment)
{
	if(segment[lSegment].flags.muted == TRUE)
	{
		segment[lSegment].flags.muted =	FALSE;
	}
	else
	{
		segment[lSegment].flags.muted = TRUE;
	}
	displayOverview();
	saveSettingFlags(lSegment);
}

void changeModeResponse(unsigned char lSegment)
{
	if(segment[lSegment].flags.modeResponse == TRUE)
	{
		segment[lSegment].flags.modeResponse =	FALSE;
	}
	else
	{
		segment[lSegment].flags.modeResponse = TRUE;
	}
	displayModeResponse();
	saveSettingFlags(lSegment);
}

void changeChainSetting(enum _swcode chaining)
{
	switch(chaining)
	{
		case SW1:
			chainMode = C1X64;
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 2;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 4;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 6;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 0;
			break;
		case SW5:
			chainMode = C2X32;
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 2;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 0;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 6;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 4;
			break;
		case SW3:
		case SW7:
			chainMode = C4X16;
			segment[0].nextSegment = 1;
			segment[1].nextSegment = 0;
			segment[2].nextSegment = 3;
			segment[3].nextSegment = 2;
			segment[4].nextSegment = 5;
			segment[5].nextSegment = 4;
			segment[6].nextSegment = 7;
			segment[7].nextSegment = 6;
			break;
		case SW8:
			chainMode = C8X8;
			segment[0].nextSegment = 0;
			segment[1].nextSegment = 1;
			segment[2].nextSegment = 2;
			segment[3].nextSegment = 3;
			segment[4].nextSegment = 4;
			segment[5].nextSegment = 5;
			segment[6].nextSegment = 6;
			segment[7].nextSegment = 7;
			break;
		// all other switches are ignored
	}
	if(runMode == TRUE)
	{
		// a reset request is generated so that LEDs and outputs are taken care of (i.e. not
		// leave an output on). A resetChains is called after the resetRequest is processed in the
		// clock routines.
		resetRequest = TRUE;
	}
	else
	{
		resetChains();
	}
	displayChain();
	writeEEchar(CHAIN_ADDR, chainMode);		// save in non-volatile memory
}

void changeORsetting(enum _swcode OR)
{
	switch(chainMode)
	{
		case C1X64:
			if(OR == SW1)				// all other switches ignored
			{
				if(ORmode64)			// toggle OR mode
					ORmode64 = 0;
				else
					ORmode64 = 0x01;
			}
			writeEEchar(ORMODE64_ADDR, ORmode64);		// save in non-volatile memory
			break;
		case C2X32:
			if(OR == SW1)
			{
				if(ORmode32 & 0x01)
					ORmode32 &= ~0x01;
				else
					ORmode32 |= 0x01;
			}
			if(OR == SW5)
			{
				if(ORmode32 & 0x10)
					ORmode32 &= ~0x10;
				else
					ORmode32 |= 0x10;
			}
			writeEEchar(ORMODE32_ADDR, ORmode32);
			break;
		case C4X16:
			if(OR == SW1)
			{
				if(ORmode16 & 0x01)
					ORmode16 &= ~0x01;
				else
					ORmode16 |= 0x01;
			}
			if(OR == SW3)
			{
				if(ORmode16 & 0x04)
					ORmode16 &= ~0x04;
				else
					ORmode16 |= 0x04;
			}
			if(OR == SW5)
			{
				if(ORmode16 & 0x10)
					ORmode16 &= ~0x10;
				else
					ORmode16 |= 0x10;
			}
			if(OR == SW7)
			{
				if(ORmode16 & 0x40)
					ORmode16 &= ~0x40;
				else
					ORmode16 |= 0x40;
			}
			writeEEchar(ORMODE16_ADDR, ORmode16);
			break;
	}
	displayChain();
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
	unsigned char c, i;

	if(runMode == FALSE)			// check if armed: startMode = true, runMode = false
	{
		runMode = TRUE;
		sendStartPulse();
	}
	if(resetRequest)
	{
		for(i = 0; i < 8; i++)
		{
			segment[i].currentStep = 0;
			segment[i].stepCounter = 0;
			segment[i].loopCounter = 0;
		}
		resetChains();
		resetRequest = FALSE;
	}
	for(i = 0; i < 8; i++)
	{
		if(segment[i].flags.active)
		{
			// stepCounter and loopCounter are advanced on Clock Low

			if(segment[i].flags.seqMode)
			{
				segment[i].currentStep = segment[i].stepCounter;
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
				c = c % (segment[i].patternSetting + 1);	// modulo
				segment[i].currentStep = c;
			}

			if(testBit(segment[i].enabledStep, segment[i].currentStep))
			{
				if(segment[i].flags.muted == FALSE)
				{
					// 1-8 jacks in this version are per segment, not per step
					setSegmentOut(i, ON);
//					setStepLEDDevice(VIEWOVERVIEW, i, ON, NORMAL);
				}
//				else
//				{
//					setStepLEDDevice(VIEWOVERVIEW, i, ON, DIM);
//				}
				// if the current step is enabled, LED full brightness
				setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, ON, NORMAL);
			}
			else
			{
				// if the current step is disabled, LED is dimmed
				setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, ON, DIM);
				// if sequence mode, output even if step is off.  Skip if Random mode.
				// &&& this is vague for Version 2. Will disable for now.
				//if(segment[i].flags.seqMode == TRUE && segment[i].flags.muted == FALSE)
				//	setSegmentOut(i, ON);
			}

			// update the views
			setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, ON, NORMAL);
			setStepLEDSegment(i, VIEWGATE, segment[i].currentStep, ON, NORMAL);
			setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, ON, NORMAL);
		}
	}
	// timers are set regardless.  Check in timer ISR to shut off LEDs
	updateOutputs();
	updateOverview();
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
	unsigned char i;
	// turn off electrical outputs
	setOutputsAllOff();

	for(i = 0; i < 8; i++)
	{
		// clean up display, Turn off LEDs for previous step
		if(segment[i].currentStep <= segment[i].patternSetting)
		{
			setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, ON , DIM);
			setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, testBit(segment[i].enabledStep, segment[i].currentStep), testBit(segment[i].enabledStep, segment[i].currentStep));
		}
		else
		{
			setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, OFF , NORMAL);
			setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, OFF, NORMAL);
		}

		// clean up display if loop setting changed
		if(segment[i].loopCounter <= segment[i].loopSetting)
			setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, ON, DIM);
		else
			setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, OFF, NORMAL);

		if(testBit(segment[i].gateSetting, segment[i].currentStep))
			setStepLEDSegment(i, VIEWGATE, segment[i].currentStep, ON, DIM);
		else
			setStepLEDSegment(i, VIEWGATE, segment[i].currentStep, OFF, NORMAL);

		if(segment[i].flags.muted)
			setStepLEDDevice(VIEWOVERVIEW, i, OFF, NORMAL);
		else
			setStepLEDDevice(VIEWOVERVIEW, i, ON, DIM);


		if(segment[i].flags.active)
		{
//			if(segment[i].flags.muted)
//				setStepLEDDevice(VIEWOVERVIEW, i, OFF, NORMAL);
//			else
//				setStepLEDDevice(VIEWOVERVIEW, i, ON, DIM);

			// determine the next step
			segment[i].stepCounter++;

			if(segment[i].stepCounter > segment[i].patternSetting)
			{
				//stepCounter = 1;
				segment[i].loopCounter++;
				if(segment[i].loopCounter > segment[i].loopSetting)
				{
					segment[i].loopCounter = 0;
					if(segment[i].flags.loopMode)
					{
						// check if we go to the next segment or stay in this one
						if(segment[i].nextSegment != i)
						{
							segment[i].flags.active = FALSE;		// current segment is no longer active
							segment[segment[i].nextSegment].flags.active = TRUE;	// set next segment as active
							if(segment[i].nextSegment < i)
							{
								// the next segment was already processed by this loop
								segment[segment[i].nextSegment].stepCounter = 0;
							}
							else
							{
								// since the segment hasn't been handled by this loop yet, set
								//  the stepCounter to 0xFF which will zero on increment
								segment[segment[i].nextSegment].stepCounter = 0xff;	// -1
							}
						}
						else
						{
							// the next segment is the same as this one, just zero stepCounter
							segment[i].stepCounter = 0;
						}
					}
					else		// end mode
					{
						// this segment is in end mode.  Set this segment as inactive.
						segment[i].flags.active = FALSE;
					}
				}
				else
				{
					// loop isn't done yet
					segment[i].stepCounter = 0;
				}
			} // end of if pattern setting
		} // end of if active
	} // end of looping through segments
	if(resetRequest)
	{
		for(i = 0; i < 8; i++)
		{
			segment[i].currentStep = 0;
			segment[i].stepCounter = 0;
			segment[i].loopCounter = 0;
		}
		resetRequest = FALSE;
		resetChains();
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
	unsigned char i;
	startMode = FALSE;
	runMode = FALSE;
	setStatusLED(LEDSTART, OFF, NORMAL);

	setOutputsAllOff();
	for(i = 0; i < 8; i++)
	{
		if(segment[i].currentStep <= segment[i].patternSetting)
		{
			setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, ON , DIM);
			setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, testBit(segment[i].enabledStep, segment[i].currentStep), testBit(segment[i].enabledStep, segment[i].currentStep));
		}
		else
		{
			setStepLEDSegment(i, VIEWPATTERN, segment[i].currentStep, OFF , NORMAL);
			setStepLEDSegment(i, VIEWGRID, segment[i].currentStep, OFF, NORMAL);
		}

		if(segment[i].loopCounter <= segment[i].loopSetting)
			setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, ON, DIM);
		else
			setStepLEDSegment(i, VIEWLOOP, segment[i].loopCounter, OFF, NORMAL);

		if(segment[i].flags.muted)
			setStepLEDDevice(VIEWOVERVIEW, i, OFF, NORMAL);
		else
			setStepLEDDevice(VIEWOVERVIEW, i, ON, DIM);

		segment[i].currentStep = 0;
		segment[i].stepCounter = 0;
		segment[i].loopCounter = 0;
	}
	// reset the "active" flags on each segment according to chain mode
	resetChains();

	resetRequest = FALSE;		// clear any pending resets
	sendStopPulse();
}

/****************************************************************************
*
* Function: resetChains
*
* Description:
*
****************************************************************************/
void resetChains(void)
{
	unsigned char it;
	switch(chainMode)
	{
		case C1X64:
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 0;
			segment[3].flags.active = 0;
			segment[4].flags.active = 0;
			segment[5].flags.active = 0;
			segment[6].flags.active = 0;
			segment[7].flags.active = 0;
			break;
		case C2X32:
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 0;
			segment[3].flags.active = 0;
			segment[4].flags.active = 1;
			segment[5].flags.active = 0;
			segment[6].flags.active = 0;
			segment[7].flags.active = 0;
			break;
		case C4X16:
			segment[0].flags.active = 1;
			segment[1].flags.active = 0;
			segment[2].flags.active = 1;
			segment[3].flags.active = 0;
			segment[4].flags.active = 1;
			segment[5].flags.active = 0;
			segment[6].flags.active = 1;
			segment[7].flags.active = 0;
			break;
		case C8X8:
			segment[0].flags.active = 1;
			segment[1].flags.active = 1;
			segment[2].flags.active = 1;
			segment[3].flags.active = 1;
			segment[4].flags.active = 1;
			segment[5].flags.active = 1;
			segment[6].flags.active = 1;
			segment[7].flags.active = 1;
			break;
	}
	for(it = 0; it < 8; it++)
	{
		if(segment[it].flags.muted)
			setStepLEDDevice(VIEWOVERVIEW, it, OFF, NORMAL);
		else
			setStepLEDDevice(VIEWOVERVIEW, it, ON, DIM);
	}

}

/****************************************************************************
*
* Function:  updateOutputs
*
* Description:  Sets the signal state on the segment output jacks based on the
*				logical OR of segment outputs and on the chaining configuration.
*
****************************************************************************/
void updateOutputs(void)
{
	unsigned char proposedOutput;
	proposedOutput = Output;
	switch(chainMode)
	{
		case C1X64:
			if(ORmode64)
			{
				if(Output & 0xff)
					proposedOutput |= 0xff;
			}
			break;
		case C2X32:
			if(ORmode32 & 0x01)
			{
				if(Output & 0x0f)
					proposedOutput |= 0x0f;
			}
			if(ORmode32 & 0x10)
			{
				if(Output & 0xf0)
					proposedOutput |= 0xf0;
			}
			break;
		case C4X16:
			if(ORmode16 & 0x01)
			{
				if(Output & 0x03)
					proposedOutput |= 0x03;
			}
			if(ORmode16 & 0x04)
			{
				if(Output & 0x0C)
					proposedOutput |= 0x0C;
			}
			if(ORmode16 & 0x10)
			{
				if(Output & 0x30)
					proposedOutput |= 0x30;
			}
			if(ORmode16 & 0x40)
			{
				if(Output & 0xC0)
					proposedOutput |= 0xC0;
			}
			break;
	}
	if(Output == 0)
	{
		LATF &= ~0x02;
	}
	else
	{
		LATF |= 0x02;
	}
	LATC = proposedOutput;
}

/****************************************************************************
*
* Function:  setSegmentOut
*
* Description: Sets the signal state on the selected segment output jack
*
****************************************************************************/
void setSegmentOut(unsigned char lSegment, unsigned char on)
{
	unsigned char shiftbits = 1;
	unsigned char c;
	shiftbits = shiftbits << (lSegment);
	if(on)
		Output |= shiftbits;
	else
		Output &= ~shiftbits;
}

/****************************************************************************
*
* Function:  setOutputsAllOff
*
* Description: Sets the signal state on the selected segment output jack
*
****************************************************************************/
void setOutputsAllOff(void)
{
	Output = 0;
	LATC = 0;
	LATF &= ~0x02;			// turn off pattern output
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
* Function:  sendStopPulse(void)
*
* Description:  Turns on the stop output and starts the timer that shuts it off.
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
// this is deprecated

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
* Function: changeSegmentView
*
* Description: Changes the current view and updates the status LEDs.  This is used
*		when switching directly from LOOP to PATTERN to GATE and vice versa.  Other
*		combinations go through displaySegment, which cleans up the status LEDs
*
*
****************************************************************************/
void changeSegmentView(enum _segview newView)
{
	if(segment[currentSegmentView].currentView != newView)		// only if they are not the same
	{
		switch(segment[currentSegmentView].currentView)			// reset LED on previous view to reflect mode
		{
			case VIEWLOOP:
				setStatusLED(LEDLOOP, segment[currentSegmentView].flags.loopMode, NORMAL);
				break;
			case VIEWPATTERN:
				setStatusLED(LEDSEQ, segment[currentSegmentView].flags.seqMode, NORMAL);
				break;
			case VIEWGATE:
				setStatusLED(LEDGATE, segment[currentSegmentView].flags.gateMode, NORMAL);
		}
		switch(newView)
		{
			case VIEWLOOP:
				setStatusLED(LEDLOOP, TRUE, DIM);
				break;
			case VIEWPATTERN:
				setStatusLED(LEDSEQ, TRUE, DIM);
				break;
			case VIEWGATE:
				setStatusLED(LEDGATE, TRUE, DIM);
				break;

		}
		segment[currentSegmentView].currentView = newView;		// update the view
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
void displayGrid(unsigned char segmentNumber)
{
	unsigned char it;
	for(it = 0; it < 8; it++)
	{
		if(it <= segment[segmentNumber].patternSetting)
			// turn on or off depending on enabledStep
			setStepLEDSegment(segmentNumber, VIEWGRID, it, testBit(segment[segmentNumber].enabledStep, it), DIM);
		else
			setStepLEDSegment(segmentNumber, VIEWGRID, it, OFF, NORMAL);
	}
}

/****************************************************************************
*
* Function:	 displayLoop
*
* Description:  Displays the current loop setting in LOOPVIEW.
*
*
****************************************************************************/
void displayLoop(unsigned char segmentNumber)
{
	unsigned char it;
	for(it = 0; it < 8; it++)
	{
		if(it <= segment[segmentNumber].loopSetting)
			setStepLEDSegment(segmentNumber, VIEWLOOP, it, ON, DIM);
		else
			setStepLEDSegment(segmentNumber, VIEWLOOP, it, OFF, NORMAL);
	}
	changeSegmentView(VIEWLOOP);
}

/****************************************************************************
*
* Function: displayPattern
*
* Description:	Displays the pattern length setting in Pattern view mode.
*
****************************************************************************/
void displayPattern(unsigned char segmentNumber)
{
	unsigned char it;
	for(it = 0; it < 8; it++)
	{
		if(it <= segment[segmentNumber].patternSetting)
			setStepLEDSegment(segmentNumber, VIEWPATTERN, it, ON, DIM);
		else
			setStepLEDSegment(segmentNumber, VIEWPATTERN, it, OFF, NORMAL);
	}
	changeSegmentView(VIEWPATTERN);
}

/****************************************************************************
*
* Function: displayGate
*
* Description:	updates the gate/trig view
*
****************************************************************************/
void displayGate(unsigned char segmentNumber)
{
	unsigned char it;
	for(it = 0; it < 8; it++)
	{
		// turn on or off depending on gateSetting
		setStepLEDSegment(segmentNumber, VIEWGATE, it, testBit(segment[segmentNumber].gateSetting, it), DIM);
	}
	changeSegmentView(VIEWGATE);
}

/****************************************************************************
*
* Function: displayChain
*
* Description:	Displays the current chain mode
*
****************************************************************************/
void displayChain(void)
{
	currentDeviceView = VIEWCHAIN;
	switch(chainMode)
	{
		case C1X64:
			if(ORmode64)
			{
				setStepLEDDevice(VIEWCHAIN, 0, ON, NORMAL);
				setStepLEDDevice(VIEWCHAIN, 1, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 2, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 3, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 4, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 5, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 6, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 7, ON, DIM);
			}
			else
			{
				setStepLEDDevice(VIEWCHAIN, 0, ON, NORMAL);
				setStepLEDDevice(VIEWCHAIN, 1, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 2, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 3, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 4, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 5, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 6, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 7, OFF, DIM);
			}
			break;
		case C2X32:
			setStepLEDDevice(VIEWCHAIN, 0, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 4, ON, NORMAL);
			if(ORmode32 & 0x01)
			{
				setStepLEDDevice(VIEWCHAIN, 1, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 2, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 3, ON, DIM);
			}
			else
			{
				setStepLEDDevice(VIEWCHAIN, 1, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 2, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 3, OFF, DIM);
			}
			if(ORmode32 & 0x10)
			{
				setStepLEDDevice(VIEWCHAIN, 5, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 6, ON, DIM);
				setStepLEDDevice(VIEWCHAIN, 7, ON, DIM);
			}
			else
			{
				setStepLEDDevice(VIEWCHAIN, 5, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 6, OFF, DIM);
				setStepLEDDevice(VIEWCHAIN, 7, OFF, DIM);

			}
			break;
		case C4X16:
			setStepLEDDevice(VIEWCHAIN, 0, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 2, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 4, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 6, ON, NORMAL);
			if(ORmode16 & 0x01)
				setStepLEDDevice(VIEWCHAIN, 1, ON, DIM);
			else
				setStepLEDDevice(VIEWCHAIN, 1, OFF, DIM);
			if(ORmode16 & 0x04)
				setStepLEDDevice(VIEWCHAIN, 3, ON, DIM);
			else
				setStepLEDDevice(VIEWCHAIN, 3, OFF, DIM);
			if(ORmode16 & 0x10)
				setStepLEDDevice(VIEWCHAIN, 5, ON, DIM);
			else
				setStepLEDDevice(VIEWCHAIN, 5, OFF, DIM);
			if(ORmode16 & 0x40)
				setStepLEDDevice(VIEWCHAIN, 7, ON, DIM);
			else
				setStepLEDDevice(VIEWCHAIN, 7, OFF, DIM);
			break;
		case C8X8:
			setStepLEDDevice(VIEWCHAIN, 0, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 1, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 2, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 3, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 4, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 5, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 6, ON, NORMAL);
			setStepLEDDevice(VIEWCHAIN, 7, ON, NORMAL);
			break;
	}
	setStatusLED(LEDLOOP, ON, DIM);
	setStatusLED(LEDSEQ, ON, DIM);
	setStatusLED(LEDGATE, ON, NORMAL);

}

/****************************************************************************
*
* Function: displayOverview
*
* Description:	Displays the overview of segments, showing which ones are muted
*
****************************************************************************/
void displayOverview(void)
{
	unsigned char it;
	currentDeviceView = VIEWOVERVIEW;
	updateOverview();
	setStatusLED(LEDLOOP, ON, NORMAL);
	setStatusLED(LEDSEQ, ON, DIM);
	setStatusLED(LEDGATE, ON, DIM);
}

/****************************************************************************
*
* Function: updateOverview
*
* Description: Updates the LEDs to the state of the output jacks.  If the
*		output jack is currenly off, then the dim vs. off state mirrors the
*		muting setting.
*
*	Note: this should be called after the outputs are updated.
*
****************************************************************************/
void updateOverview(void)
{
	unsigned char it;
	unsigned char shiftbit;
	shiftbit = 1;
	for(it = 0; it < 8; it++)
	{
		if(LATC & shiftbit)
		{
			// output is on, turn on LED
			setStepLEDDevice(VIEWOVERVIEW, it, ON, NORMAL);
		}
		else
		{
			// output is off, turn off or dim LED based on muting
			if(segment[it].flags.muted)
				setStepLEDDevice(VIEWOVERVIEW, it, OFF, NORMAL);
			else
				setStepLEDDevice(VIEWOVERVIEW, it, ON, DIM);
		}
		shiftbit = shiftbit << 1;
	}
}

/****************************************************************************
*
* Function: displayModeResponse
*
* Description:	Displays the overview of segments, showing which ones are muted
*
****************************************************************************/
void displayModeResponse(void)
{
	unsigned char it;
	currentDeviceView = VIEWMODERESPONSE;
	for(it = 0; it < 8; it++)
	{
		if(segment[it].flags.modeResponse)
			setStepLEDDevice(VIEWMODERESPONSE, it, ON, NORMAL);
		else
			setStepLEDDevice(VIEWMODERESPONSE, it, OFF, NORMAL);
	}
	setStatusLED(LEDLOOP, ON, DIM);
	setStatusLED(LEDSEQ, ON, NORMAL);
	setStatusLED(LEDGATE, ON, DIM);
}

/****************************************************************************
*
* Function: displaySegment
*
* Description:
*
****************************************************************************/
void displaySegment(unsigned char segmentNumber)
{
	// always enter a segment at the VIEWGRID level
	currentDeviceView = VIEWSEGMENT;
	currentSegmentView = segmentNumber;		// update global
	segment[segmentNumber].currentView = VIEWGRID;
//	displayGrid(segmentNumber);
	setStatusLED(LEDLOOP, segment[segmentNumber].flags.loopMode, NORMAL);
	setStatusLED(LEDSEQ, segment[segmentNumber].flags.seqMode, NORMAL);
	setStatusLED(LEDGATE, segment[segmentNumber].flags.gateMode, NORMAL);
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
	DI();
	PIR6bits.EEIF = 0;			// ensure clear flag
	EECON2 = 0x55;				// start sequence to begin writing
	EECON2 = 0xaa;
	EECON1bits.WR = 1;			// write has begun... self-clearing
	EI();
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
* Function: initLinkMode
*
* Description: Read the status of the link cable(s) and determine this
*		module's role in link mode.  Make adjustments to the IO pin funtions
*		based on the role.  Other than initializing pins, no other link
*		functions are supported in this firmware version.
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
* Function:  testBit
*
* Description:  To save on storage space in the segments, values such as
*		enabledStep are stored in a bitwise fashion. This routine tests the
*		designated bit and returns TRUE or FALSE.  The bitNumber is 0 - 7.
*
****************************************************************************/
unsigned char testBit(unsigned char dataByte, unsigned char bitNumber)
{
	unsigned char shiftbit = 1;
	shiftbit = shiftbit << bitNumber;
	if(dataByte & shiftbit)
		return TRUE;
	else
		return FALSE;
}

/****************************************************************************
*
* Function:  setBit
*
* Description:  To save on storage space in the segments, values such as
*		enabledStep are stored in a bitwise fashion.  This routine sets or
*		clears the designated bit.  The bitNumber is 0-7, and bitValue is
*		non-zero or zero.
*
****************************************************************************/
void setBit(unsigned char *dataByte, unsigned char bitNumber, unsigned char bitValue)
{
	unsigned char shiftbit = 1;
	shiftbit = shiftbit << bitNumber;
	if(bitValue)
		*dataByte |= shiftbit;
	else
		*dataByte &= ~shiftbit;

}

/****************************************************************************
*
* Function:
*
* Description:
*
****************************************************************************/
