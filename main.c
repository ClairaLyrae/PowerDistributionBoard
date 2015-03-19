////////////////////////////////////////////////////////////
//
//	Title:		Power Distribution Board
//	Author:		Claira Safi
//	Revised:	2/16/15
//	Device:		PIC24F32KA302
//
////////////////////////////////////////////////////////////

// Power Distribution Board

////////////////////////////////////////////////////////////
//	Includes/Config
////////////////////////////////////////////////////////////

#include "p24f32ka302.h"
#include <i2c.h>
#include <spi.h>

 int FBS __attribute__((space(prog), address(0xF80000))) = 0xF ;
 int FGS __attribute__((space(prog), address(0xF80004))) = 0x3 ;
 int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0x42;
 int FOSC __attribute__((space(prog), address(0xF80008))) = 0xDA ;
 int FWDT __attribute__((space(prog), address(0xF8000A))) = 0x5F ;
 int FPOR __attribute__((space(prog), address(0xF8000C))) = 0xFF ;
 int FICD __attribute__((space(prog), address(0xF8000E))) = 0xE3 ;
 int FDS __attribute__((space(prog), address(0xF80010))) = 0xDF ;

////////////////////////////////////////////////////////////
//	Operational Constants
////////////////////////////////////////////////////////////

#define XTFREQ          20000000         	  // On-board Crystal frequency
#define PLLMODE         1               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE        9600
#define BRGVAL          ((FCY/(BAUDRATE*16))-1)

#define AVDD			3.3						// Device voltage
#define ADCRES12		(float)(AVDD/4096)		// 12-bit ADC voltage resolution
#define ADCRES10		(float)(AVDD/1024)		// 10-bit ADC voltage resolution
#define ADCRES8			(float)(AVDD/256)		// 8-bit ADC voltage resolution

////////////////////////////////////////////////////////////
//	Pin Mapping
////////////////////////////////////////////////////////////

 // LED Debug Pins
#define LED1		_RA4
#define LED1_TRIS	TRISAbits.TRISA4
#define LED2		_RB5
#define LED2_TRIS	TRISBbits.TRISB5
#define LED3		_RB6
#define LED3_TRIS	TRISBbits.TRISB6

 // Misc Pins
#define ALARM		_RB7
#define ALARM_TRIS	TRISBbits.TRISB7
#define FAULT		_RB7
#define FAULT_TRIS	TRISBbits.TRISB7

// Current Rail Switches
#define BUSENA		_RA7
#define BUSENA_TRIS	TRISAbits.TRISA7
#define BUSENB		_RA6
#define BUSENB_TRIS	TRISAbits.TRISA6
#define BUSENC		_RB10
#define BUSENC_TRIS	TRISBbits.TRISB10
#define BUSEND		_RB11
#define BUSEND_TRIS	TRISBbits.TRISB11

// Cell voltage ADC inputs
#define VCELLA			_RA0
#define VCELLA_TRIS	TRISAbits.TRISA0 //AN0
#define VCELLA_AN		_ANSA0
#define VCELLB			_RA1
#define VCELLB_TRIS	TRISBbits.TRISA1 //AN1
#define VCELLB_AN		_ANSA1
#define VCELLC			_RB0
#define VCELLC_TRIS	TRISBbits.TRISB0 //AN2
#define VCELLC_AN		_ANSB0
#define VCELLD			_RB1
#define VCELLD_TRIS	TRISBbits.TRISB1 //AN3
#define VCELLD_AN		_ANSB1

// Auxillary Supply Current Sense
#define VBATT		_RB2
#define VBATT_TRIS	TRISBbits.TRISB2 //AN4
#define VBATT_AN	_ANSB2
#define ISENSEAUX		_RB3
#define ISENSEAUX_TRIS	TRISBbits.TRISB3 //AN4
#define ISENSEAUX_AN	_ANSB3

// Current Sense ADC inputs
#define ISENSEA			_RB12
#define ISENSEA_TRIS		TRISBbits.TRISB12 //AN12
#define ISENSEA_AN		_ANSB12
#define ISENSEB			_RB13
#define ISENSEB_TRIS		TRISBbits.TRISB13 //AN11
#define ISENSEB_AN		_ANSB13
#define ISENSEC			_RB14
#define ISENSEC_TRIS		TRISBbits.TRISB14 //AN10
#define ISENSEC_AN		_ANSB14
#define ISENSED			_RB15
#define ISENSED_TRIS		TRISBbits.TRISB15 //AN9
#define ISENSED_AN		_ANSB15

// I2C Slave Bus Pins
#define SCL			_RB8
#define SCL_TRIS	TRISBbits.TRISB8
#define SDA			_RB9
#define SDA_TRIS	TRISBbits.TRISB9

////////////////////////////////////////////////////////////
//	Constants
////////////////////////////////////////////////////////////

// Slave I2C Address (7-bit)
#define I2C_ADDRESS 0b11101100

////////////////////////////////////////////////////////////
//	Bitfields
////////////////////////////////////////////////////////////

static volatile struct DEVICECON {
	unsigned UNUSED : 8;
} DEVICECONbits;

#define I2C_STATE_IDLE	0
#define I2C_STATE_MEM	1
#define I2C_STATE_DATA	2

static volatile struct DEVICESTAT {
	unsigned I2CSTATE: 2;
} DEVICESTATbits;

////////////////////////////////////////////////////////////
//	I2C Register
////////////////////////////////////////////////////////////

// Register contents
#define REG_DEVICESTAT	0x00	// Device status bits	(<7:4> rail faults, <3> aux fault, <2> overcurrent, <1> cell unbalanced, <0> undercharged)
#define REG_DEVICECON	0x01	// Device control bits	(<7:4> output rail enable, <3> aux enable, <2> curr alerts, <1> cell alerts, <0> batt alerts)
#define REG_VBATT		0x02	// Total battery voltage (0.1V/bit, 0 to 25.5V)
#define REG_VCELLA		0x03	// Battery cell voltage (0.1V/bit, 0 to 25.5V)
#define REG_VCELLB		0x04	// Battery cell voltage (0.1V/bit, 0 to 25.5V)
#define REG_VCELLC		0x05	// Battery cell voltage (0.1V/bit, 0 to 25.5V)
#define REG_VCELLD		0x06	// Battery cell voltage (0.1V/bit, 0 to 25.5V)
#define REG_ISENSEA		0x07	// Output rail current (0.1A/bit, 0 to 25.5A)
#define REG_ISENSEB		0x08	// Output rail current (0.1A/bit, 0 to 25.5A)
#define REG_ISENSEC		0x09	// Output rail current (0.1A/bit, 0 to 25.5A)
#define REG_ISENSED		0x0A	// Output rail current (0.1A/bit, 0 to 25.5A)
#define REG_ISENSEAUX	0x0B	// Auxilary rail current (0.1A/bit, 0 to 25.5A)
#define REG_IALERT		0x0C	// Rail current alert level (0.1A/bit, 0 to 25.5A)
#define REG_ISHDWN		0x0D	// Rail current auto-shutdown level (0.1A/bit, 0 to 25.5A)
#define REG_VCALERT		0x0E	// Cell voltage alert level (0.1V/bit, 0 to 25.5V)
#define REG_VCSHDWN		0x0F	// Cell voltage auto-shutdown level (0.1V/bit, 0 to 25.5V)
#define REG_VBALERT		0x10	// Battery voltage alert level (0.1V/bit, 0 to 25.5V)
#define REG_VBSHDWN		0x11	// Battery voltage auto-shutdown level (0.1V/bit, 0 to 25.5V)

// Register for I2C addressing
#define REGISTER_SIZE		0x12
static volatile char register_data[REGISTER_SIZE];
static volatile unsigned register_pointer = 0;

// Register R/W masking bits (1 = writeable, 0 = read only)
static volatile char register_rw_mask[REGISTER_SIZE];
static volatile char register_changed[REGISTER_SIZE];
static volatile unsigned register_if;

// Setup bit R/W masking for registers
void inline initializeRegister()
{
	register_rw_mask[REG_DEVICECON] = 0xff;
}

void inline refreshRegister()
{
	if(register_changed[REG_DEVICECON])
	{
		register_changed[REG_DEVICECON] = 0;
	}
}

////////////////////////////////////////////////////////////
//	Functions
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
//	Initialization
////////////////////////////////////////////////////////////

// Initialize pin configurations and I/O features
inline void initializeIO()
{
    ANSA = 0x0000;
    ANSB = 0x0000;

	ISENSEA_TRIS = 1;
	ISENSEB_TRIS = 1;
	ISENSEC_TRIS = 1;
	ISENSED_TRIS = 1;
	ISENSEAUX_TRIS = 1;
	VBATT_TRIS = 1;
	VCELLA_TRIS = 1;
	VCELLB_TRIS = 1;
	VCELLC_TRIS = 1;
	VCELLD_TRIS = 1;
	BUSENA_TRIS = 0;
	BUSENB_TRIS = 0;
	BUSENC_TRIS = 0;
	BUSEND_TRIS = 0;
	ALARM_TRIS = 0;
	FAULT_TRIS = 0;
	SDA_TRIS = 0;
	SCL_TRIS = 0;
	LED1_TRIS = 0;
	LED2_TRIS = 0;
	LED3_TRIS = 0;

    LATA = 0;
    LATB = 0;
}

// Initialize SFR registers for peripherals
inline void initializeSFR()
{
    CLKDIVbits.RCDIV = 0;

	// ADC
	AD1CON1bits.ADON = 1;		// Enable
	AD1CON1bits.ADSIDL = 0;		// Dont stop in idle
	AD1CON1bits.FORM = 0b00;	// Absolute decimal
	AD1CON2bits.BUFM = 0;		// 16-word buffer (no split)
	AD1CON1bits.SSRC = 0b0111;	// Auto sample every X tad
	AD1CON1bits.ASAM = 1;		// Sample immediately after last conversion
	AD1CON2bits.CSCNA = 1;		// Scan inputs
	AD1CON2bits.SMPI = 0b1010;	// Interrupt every 10 conversions
	AD1CON2bits.ALTS = 0;		// Always use channel A
	AD1CON3bits.ADCS = 0b00111111;	// Tad = 64*Tcy
	AD1CON3bits.SAMC = 0b11111;		// Sample every 31 Tad
	AD1CHSbits.CH0NA = 0b000;	// Use AVss for negative input
	//AD1CHSbits.CH0SA = 0b00000;	// Not needed for scan (auto set)
	AD1CSSL = 0b0001111000111111;	// AN0-5, AN9-12
	AD1CSSH = 0b0000000000000000;
	_AD1IF = 0;
	_AD1IE = 1;

	// Timer 2
    TMR2 = 0;               // Clear timer 2
	T2CONbits.T32 = 0;		// Disable dual-timer
	T2CONbits.TCKPS = 0b00;
	T2CONbits.TON = 0;
    PR2 = 0x8800;           // Interrupt every 250ms
    _T2IF = 0;      // Clear interrupt flag
    _T2IE = 0;      // Set interrupt enable bit

	// I2C1 (Slave Device)
	I2C1CONbits.I2CEN = 1;	// I2C enable
	I2C1CONbits.I2CSIDL = 0;	// I2C stop in idle
	I2C1CONbits.SCLREL = 0;	// I2C clock release
	I2C1CONbits.STREN = 0;	// I2C clock stretch enable
	I2C1CONbits.IPMIEN = 0;	// I2C Peripheral Management
	I2C1CONbits.A10M = 0;	// I2C address size = 7 bit
	I2C1CONbits.SMEN = 0;	// I2C address size = 7 bit
	I2C1CONbits.SMEN = 0;	// I2C SMBUS enable
	I2C1MSK = 0x0000;	// I2C SMBUS enable
	I2C1BRG = 157;
	I2C1ADD = 0x54;
	_SI2C1IF = 0;
	_SI2C1IE = 1;

	// Timer 1
    TMR1 = 0;               // Clear timer 1
    PR1 = 152;           // Interrupt every 250ms
    _T1IF = 0;      // Clear interrupt flag
    _T1IE = 1;      // Set interrupt enable bit
	T1CONbits.TCKPS = 0b00;
	T1CONbits.TON = 1;
}

////////////////////////////////////////////////////////////
//	Main Program
////////////////////////////////////////////////////////////

// Main program loop
int main(void)
{
	initializeIO();
	initializeSFR();
	initializeRegister();
    while(1)
    {
		// Check for register changes and update parameters accordingly
		refreshRegister();
	}
	return 0;
}

////////////////////////////////////////////////////////////
// Interrupts
////////////////////////////////////////////////////////////

// Change Notification
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    _CNIF = 0;      // clear interrupt flag
}

// ADC interrupt (Save measurements to register)
void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
	register_data[REG_VCELLA] = ADC1BUF0;
	register_data[REG_VCELLB] = ADC1BUF1;
	register_data[REG_VCELLC] = ADC1BUF2;
	register_data[REG_VCELLD] = ADC1BUF3;
	register_data[REG_VBATT] = ADC1BUF4;
	register_data[REG_ISENSEAUX] = ADC1BUF5;
	register_data[REG_ISENSEA] = ADC1BUF6;
	register_data[REG_ISENSEB] = ADC1BUF7;
	register_data[REG_ISENSEC] = ADC1BUF8;
	register_data[REG_ISENSED] = ADC1BUF9;
    _AD1IF = 0;      // clear interrupt flag
}

// Serial port interrupt (I2C Slave command)
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void )
{
    unsigned char temp;
	// Address Matched
	if(!I2C1STATbits.D_A)
	{
		// Enter Read Mode
		if(!I2C1STATbits.R_W)
		{
			temp = I2C1RCV;     //dummy read
			DEVICESTATbits.I2CSTATE = I2C_STATE_MEM;	// Wait for RAM address
		}
		// Enter Write Mode (First byte)
		else if(I2C1STATbits.R_W)
		{
			temp = I2C1RCV;
			I2C1TRN = register_data[register_pointer];      //Read data from RAM & send data to I2C master device
			I2C1CONbits.SCLREL = 1; // Release SCL1 line
			while(I2C1STATbits.TBF);
		}
	}
	// Data bytes
	else
	{
		// Data Read
		if(I2C1STATbits.R_W)
		{
			if(++register_pointer >= REGISTER_SIZE)
				register_pointer = 0;
			temp = I2C1RCV;
			I2C1TRN = register_data[register_pointer];      //Read data from RAM & send data to I2C master device
			I2C1CONbits.SCLREL = 1; // Release SCL1 line
			while(I2C1STATbits.TBF);
		}
		// Data Write
		else
		{
			// First byte is register address
			if(DEVICESTATbits.I2CSTATE == I2C_STATE_MEM)
			{
				DEVICESTATbits.I2CSTATE = I2C_STATE_DATA;	// Allow data transfer
				register_pointer = I2C1RCV;
				if(register_pointer >= REGISTER_SIZE)
					register_pointer = 0;
			}
			// Next byte(s) are values to write into register
			else if(DEVICESTATbits.I2CSTATE == I2C_STATE_DATA)
			{
				// Mask input based on register R/W masking bits and write
				unsigned char mask = register_rw_mask[register_pointer];
				register_data[register_pointer] = (I2C1RCV & mask) | (register_data[register_pointer] & (mask ^ 0xffff));
				register_changed[register_pointer] = 1;
				if(++register_pointer >= REGISTER_SIZE)
					register_pointer = 0;
			}
		}
	}
    _SI2C1IF = 0;
}
