/*
===

	     COMIO: MCU Serial IO Protocol Shell

	   Version: 1.21.20130811212335 [MSP430]

	      Info: http://cogsmith.com/software/comio
	    Author: David Adam Coffey <dac@cogsmith.com>
	 Copyright: COGSMITH © 2012-2013
	   License: COGSMITH OPEN LICENSE AGREEMENT [COLA-1.2]
	LicenseURL: http://cogsmith.com/legal/licenses/cola

===
*/

// COMPILER = CCS 5.2.1
#include <stdlib.h>
#include "msp430g2553.h"

//

#define COMIO_VERSION "1.09.2012.1008"
#define MCU "MSP430G2553"

#define PIN0 0x01
#define PIN1 0x02
#define PIN2 0x04
#define PIN3 0x08
#define PIN4 0x10
#define PIN5 0x20
#define PIN6 0x40
#define PIN7 0x80

#define TX_PIN PIN1 // 0x02 = P1.1 = Timer0_A.OUT0
#define RX_PIN PIN2 // 0x04 = P1.2 = Timer0_A.CCI1A

#define ADC_PIN_TEMP 10

#define CLOCKRATE 1000000
#define BAUDRATE  9600
#define UARTSPEED CLOCKRATE/BAUDRATE

//

int VERBOSE = 0;

unsigned int  TX_DATA;
unsigned char RX_DATA;

int i=0;

//

char LTOATXT[] = "000000000";
void *LTOA (long l) {
	*LTOATXT=0;
	ltoa(l,LTOATXT);
	return &LTOATXT;
}

//

unsigned int ADC_ReadPin (char label[]) {
	if (label[2]=='0') { ADC10CTL1 = INCH_0; ADC10AE0 |= PIN0; }
	if (label[2]=='1') { ADC10CTL1 = INCH_1; ADC10AE0 |= PIN1; }
	if (label[2]=='2') { ADC10CTL1 = INCH_2; ADC10AE0 |= PIN2; }
	if (label[2]=='3') { ADC10CTL1 = INCH_3; ADC10AE0 |= PIN3; }
	if (label[2]=='4') { ADC10CTL1 = INCH_4; ADC10AE0 |= PIN4; }
	if (label[2]=='5') { ADC10CTL1 = INCH_5; ADC10AE0 |= PIN5; }
	if (label[2]=='6') { ADC10CTL1 = INCH_6; ADC10AE0 |= PIN6; }
	if (label[2]=='7') { ADC10CTL1 = INCH_7; ADC10AE0 |= PIN7; }

	ADC10CTL0 = REFON + ADC10ON + ADC10IE;
	for(i=0;i<999;i++);
	ADC10CTL0 |= ENC + ADC10SC;             							// Sampling and conversion start
	__bis_SR_register(CPUOFF + GIE);        							// LPM0 with interrupts enabled
	ADC10CTL1 = INCH_10;
    return ADC10MEM;
}

//

unsigned int ADC_ReadTemp () {
	ADC10CTL1 = INCH_10 + ADC10DIV_3;         							// Temp Sensor ADC10CLK/4
	ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
	for(i=0;i<999;i++);
	ADC10CTL0 |= ENC + ADC10SC;             							// Sampling and conversion start
	__bis_SR_register(CPUOFF + GIE);        							// LPM0 with interrupts enabled
    return ADC10MEM;
}

//

int IO_ReadPin (char label[]) {
	unsigned long pin = 0;

	if (label[2]=='0') { pin = PIN0; }
	if (label[2]=='1') { pin = PIN1; }
	if (label[2]=='2') { pin = PIN2; }
	if (label[2]=='3') { pin = PIN3; }
	if (label[2]=='4') { pin = PIN4; }
	if (label[2]=='5') { pin = PIN5; }
	if (label[2]=='6') { pin = PIN6; }
	if (label[2]=='7') { pin = PIN7; }

	if (label[0]=='1') {
		P1SEL &= ~pin;
		P1DIR &= ~pin;
		if ((P1IN & pin)==0) { return 1; } else { return 0; }
	}

	if (label[0]=='2') {
		P2SEL &= ~pin;
		P2DIR &= ~pin;
		if ((P2IN & pin)==0) { return 1; } else { return 0; }
	}
}

//

void IO_SetPin (char label[], int state) {
	char port = label[0];
	char pin  = label[2];
	char v = 0x00;

	if (pin=='0') { v=PIN0; }
	if (pin=='1') { v=PIN1; }
	if (pin=='2') { v=PIN2; }
	if (pin=='3') { v=PIN3; }
	if (pin=='4') { v=PIN4; }
	if (pin=='5') { v=PIN5; }
	if (pin=='6') { v=PIN6; }
	if (pin=='7') { v=PIN7; }

	if (port=='1') { P1DIR |= v; if (state==1) { P1OUT |= v; } else { P1OUT &= ~v; } }
	if (port=='2') { P2DIR |= v; if (state==1) { P2OUT |= v; } else { P2OUT &= ~v; } }
}

//

void TX_SendByte (unsigned char byte) {
    while (TACCTL0 & CCIE);		// Ensure last char got TX'd
    TACCR0 = TAR;				// Current state of TA counter
    TACCR0 += UARTSPEED;		// One bit time till first bit
    TACCTL0 = OUTMOD0 + CCIE;	// Set TXD on EQU0, Int
    TX_DATA = byte;             // Load global variable
    TX_DATA |= 0x100;           // Add mark stop bit to TXData
    TX_DATA <<= 1;              // Add space start bit
}

//

void TX_SendString (char *string) {
    while (*string) { TX_SendByte(*string++); }
}

//

void TX_SendLine (char *string) {
	TX_SendString(string);
	TX_SendString("\r\n");
}

//

int COMIO_IsPrintable (char c) { if (c>32&&c<127) { return 1; } return 0; }

//

char* COMIO_GetPinLabel (char c) {
	char* io = "0.0";
	if (c=='A'||c=='a'||c=='O'||c=='o') { io = "2.0"; }
	if (c=='B'||c=='b'||c=='P'||c=='p') { io = "2.1"; }
	if (c=='C'||c=='c'||c=='Q'||c=='q') { io = "2.2"; }
	if (c=='D'||c=='d'||c=='R'||c=='r') { io = "2.3"; }
	if (c=='E'||c=='e'||c=='S'||c=='s') { io = "2.4"; }
	if (c=='F'||c=='f'||c=='T'||c=='t') { io = "2.5"; }
	if (c=='G'||c=='g'||c=='U'||c=='u') { io = "1.0"; }
	if (c=='H'||c=='h'||c=='V'||c=='v') { io = "1.6"; }
	if (c=='I'||c=='i'||c=='W'||c=='w') { io = "1.3"; }
	if (c=='J'||c=='j'||c=='X'||c=='x') { io = "1.4"; }
	if (c=='K'||c=='k'||c=='Y'||c=='y') { io = "1.5"; }
	if (c=='L'||c=='l'||c=='Z'||c=='z') { io = "1.7"; }
	return io;
}

//

void COMIO_ParseIO_Read (char c) {
	int v = 0;

	char *label = "";
	label = COMIO_GetPinLabel(c);

	if (c>=0x4f&&c<=0x6f) { v = ADC_ReadPin(label); }
	if (c>=0x6a&&c<=0x7a) { v = IO_ReadPin(label);  }

	if (VERBOSE==1) {
		if (c>=0x4f&&c<=0x6f) { TX_SendString("ANALOG ");  }
		if (c>=0x6a&&c<=0x7a) { TX_SendString("DIGITAL "); }

		TX_SendString("READ P");
		TX_SendString(label);
		TX_SendString(" = ");
		TX_SendString(LTOA(v));
		TX_SendLine("");
	} else {
		if (c>=0x4f&&c<=0x6f) { TX_SendLine(LTOA(v)); }
		if (c>=0x6a&&c<=0x7a) { TX_SendLine(LTOA(v)); }
	}
}

//

void COMIO_ParseIO_Set (char c) {
	int v = 0;

	char *label = "";
	label = COMIO_GetPinLabel(c);

	if (c<0x50) { v=1; } else { v=0; }
	IO_SetPin(label,v);

	if (VERBOSE==1) {
		TX_SendString("SET P");
		TX_SendString(label);
		TX_SendString(" = ");
		if (v==1) { TX_SendString("ON"); } else { TX_SendString("OFF"); }
		TX_SendLine("");
	} else {
		if (v==1) { TX_SendLine("1"); } else { TX_SendLine("0"); }
	}
}

//

void COMIO_ParseCommand (char c) {
	if (VERBOSE==0||c=='$') {
		if (COMIO_IsPrintable(c)==1) { TX_SendByte(c); } else { TX_SendByte('*'); }
		if (c=='$') { TX_SendString("$"); }
		TX_SendLine("");
	} else {
		if (COMIO_IsPrintable(c)==1) {
			TX_SendString("'");
			TX_SendByte(c);
			TX_SendString("' ");
		}
		TX_SendString("[");
		TX_SendString(LTOA(c));
		TX_SendLine("]");
		TX_SendString("> ");
	}

	if ( ((c>=0x41)&&(c<=0x4c)) || ((c>=0x61)&&(c<=0x6c)) ) { COMIO_ParseIO_Set(c); return; }
	if ( ((c>=0x4f)&&(c<=0x5a)) || ((c>=0x6f)&&(c<=0x7a)) ) { COMIO_ParseIO_Read(c); return; }

    switch (c) {

    	case '`': //case 0x1b:
        	TX_SendLine("RESET");
        	WDTCTL = 0;
        	break;


    	case '#':
    		TX_SendLine("VERBOSE MODE");
    		VERBOSE=1;
    		break;


    	case '$':
			//TX_SendLine("CONCISE MODE");
    		VERBOSE=0;
    		break;


    	case ',':
    		TX_SendLine("INFO");
    		TX_SendString("COMIO \t"); TX_SendLine(COMIO_VERSION);
    		TX_SendString("MCU   \t"); TX_SendLine(MCU);
    		TX_SendString("CLOCK \t"); TX_SendLine(LTOA(CLOCKRATE));
    		TX_SendString("BAUD  \t"); TX_SendLine(LTOA(BAUDRATE));
    		break;


    	case '/':
    		TX_SendLine("LABELS");
    		TX_SendLine("A O = 2.0");
    		TX_SendLine("B P = 2.1");
    		TX_SendLine("C Q = 2.2");
    		TX_SendLine("D R = 2.3");
    		TX_SendLine("E S = 2.4");
    		TX_SendLine("F T = 2.5");
    		TX_SendLine("G U = 1.0");
    		TX_SendLine("H V = 1.6");
    		TX_SendLine("I W = 1.3");
    		TX_SendLine("J X = 1.4");
    		TX_SendLine("K Y = 1.5");
    		TX_SendLine("L Z = 1.7");
    		break;


    	case '?':
    		TX_SendLine("HELP");
    		TX_SendLine("");
    		TX_SendLine(" *  NOP");
    		TX_SendLine(" ?  HELP MENU");
    		TX_SendLine(" ,  SHOW INFO");
    		TX_SendLine(" /  PIN LABELS");
    		TX_SendLine(" ~  RESET MCU");
    		TX_SendLine("");
    		TX_SendLine(" #  VERBOSE MODE");
    		TX_SendLine(" $  CONCISE MODE");
    		TX_SendLine("");
    		TX_SendLine("A-L OUTPUT SET HI");
    		TX_SendLine("a-l OUTPUT SET LO");
    		TX_SendLine("");
    		TX_SendLine("U-Z INPUT READ ANALOG");
    		TX_SendLine("o-z INPUT READ DIGITAL");
    		TX_SendLine(" |  INPUT READ TEMPERATURE");
    		break;

			// TODO
    		//TX_SendLine("");
    		//TX_SendLine(" .  DISPLAY PIN IO STATUS");
    		//TX_SendLine(" %  DISPLAY LOOP ANALOG INPUTS");
    		//TX_SendLine(" ^  DISPLAY LOOP DIGITAL INPUTS");
    		//TX_SendLine(" \\  DISPLAY LOOP TEMPERATURE");
    		//TX_SendLine(" &  DISPLAY LOOP EVERYTHING");


    	case '%':
    		TX_SendLine("DISPLAY LOOP ANALOG INPUTS");
    		break;


    	case '^':
    		TX_SendLine("DISPLAY LOOP DIGITAL INPUTS");
    		break;


    	case '.':
        	TX_SendLine("PIN IO STATUS");
    		break;


    	case '|':
        	TX_SendString("TEMP = ");
        	TX_SendLine(LTOA(ADC_ReadTemp()));
    		break;


    	case '*':
    	default:
    		TX_SendLine("NOP");
    		break;

    }
}

//

void COMIO_WaitForInput (void) {
	__bis_SR_register(LPM0_bits); // Wait for incoming character
}

//

void COMIO_SendPrompt (void) {
	TX_SendLine("");
	if (VERBOSE==1) { TX_SendString(": "); }
}

//

void COMIO_LoopDo (void) {
	COMIO_SendPrompt();
	COMIO_WaitForInput();
    COMIO_ParseCommand(RX_DATA);
}

//

void COMIO_Init (void) {
	TX_SendByte(0x1b); TX_SendString("[2J");	// ESC_ClearScreen
	TX_SendByte(0x1b); TX_SendString("[H");  	// ESC_GotoHome

	TX_SendLine("");
	TX_SendLine("## COMIO ##");

	ADC_ReadTemp();

	// INIT HI
	P2OUT |= PIN0;  // A O
	P2OUT |= PIN1;  // B P
	P2OUT |= PIN2;  // C Q
	P2OUT |= PIN3;  // D R
	P2OUT |= PIN4;  // E S
	P2OUT |= PIN5;  // F T
	P1OUT |= PIN0;  // G U
	P1OUT |= PIN6;  // H V

	// INIT LO
	P1OUT &= ~PIN3; // I W
	P1OUT &= ~PIN4; // J X
	P1OUT &= ~PIN5; // K Y
	P1OUT &= ~PIN7; // L Z
}

//

void Init_Timers (void) {
    WDTCTL = WDTPW + WDTHOLD;	// Stop watchdog timer
}

//

void Init_Clocks (void) {
    DCOCTL  = 0x00;            	// Set DCOCLK to 1MHz
    DCOCTL  = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;
}

//

void Init_IO (void) {
    P1OUT = 0x00;           	// Initialize all GPIO
    P1SEL = TX_PIN + RX_PIN; 	// Timer function for TXD/RXD pins
    P1DIR = 0xff & ~RX_PIN;    	// Set all pins but RXD to output

    P2OUT = 0x00;
    P2SEL = 0x00;
    P2DIR = 0xff;
}

//

void Init_UART_Timer (void) {
	TACCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
	TACCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
	TACTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}

//

void Init_ISR (void) {
    __enable_interrupt();
}

//

void Init (void) {
    Init_Timers();
    Init_Clocks();
    Init_IO();
    Init_ISR();

    //Init_UART_Hardware();
	Init_UART_Timer();

	COMIO_Init();
}

//

void LoopDo (void) {
	COMIO_LoopDo();
}

//

void Loop (void) { while (1) { LoopDo(); } }
void main (void) { Init(); Loop(); }

//

#pragma vector = TIMER0_A0_VECTOR
__interrupt void ISR_Timer_A0 (void) {
    static unsigned char tx_count = 10;
    TACCR0 += UARTSPEED;                    			// Add Offset to CCRx$
    if (tx_count == 0) {                    			// All bits TXed?
        TACCTL0 &= ~CCIE;                   			// All bits TXed, disable interrupt
        tx_count = 10;                      			// Re-load bit counter
    } else {
        if (TX_DATA & 0x01) { TACCTL0 &= ~OUTMOD2; } 	// TX Mark '1'
        else { TACCTL0 |= OUTMOD2; } 					// TX Space '0'
        TX_DATA >>= 1;
        tx_count--;
    }
}

//

#pragma vector = TIMER0_A1_VECTOR
__interrupt void ISR_Timer_A1 (void) {
	static unsigned char rx_count = 8;
	static unsigned char rx_data = 0;
	switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { 			// Use calculated branching
		case TA0IV_TACCR1:                        			// TACCR1 CCIFG - UART RX
			TACCR1 += UARTSPEED;                  			// Add Offset to CCRx
			if (TACCTL1 & CAP) {                 			// Capture mode = start bit edge
				TACCTL1 &= ~CAP;                 			// Switch capture to compare mode
				TACCR1 += UARTSPEED/2;						// Point CCRx to middle of D0
			} else {
				rx_data >>= 1;
				if (TACCTL1 & SCCI) { rx_data |= 0x80; }  	// Get bit waiting in receive latch
				rx_count--;
				if (rx_count == 0) {            		 	// All bits RXed?
					RX_DATA = rx_data;           			// Store in global variable
					rx_count = 8;                			// Re-load bit counter
					TACCTL1 |= CAP;              			// Switch compare to capture mode

					int clearbits=1;
					if (clearbits==1) { __bic_SR_register_on_exit(LPM0_bits); } // Clear LPM0 bits from 0(SR)
				}
			}
		break;
	}
}

//


#pragma vector = ADC10_VECTOR
__interrupt void ISR_ADC (void) {
	__low_power_mode_off_on_exit();
}


//

/*
#pragma vector = WDT_VECTOR
__interrupt void ISR_WDT (void) {
	__low_power_mode_off_on_exit();
}
*/
