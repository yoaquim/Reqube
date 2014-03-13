#include "msp430f6433.h"

void rfidSetup();
void ledPortSetup();
void timerSetup();
void getIDfromData();

#define LED			BIT2
#define LEDDIR		P1DIR
#define	LEDOUT		P1OUT
#define RFIDSEL		P8SEL
#define RFIDCTL1	UCA1CTL1
#define RFIDBR0		UCA1BR0
#define RFIDBR1		UCA1BR1
#define RFIDMCTL	UCA1MCTL
#define RFIDMCTL1	UCA1MCTL1
#define RFIDIE		UCA1IE
#define RFIDRXBUF	UCA1RXBUF

char rfidData[16];										// Buffer to store data received from RFID reader
char rfidTagID[12];											// Buffer to store RFID tag's ID number
int xbeeRcvIndex = 0;										// Index to keep track of current received data (from 0 to 15: 16 bytes of data)
int timerCount1 = 0;										// Counter to keep track of times the timer ISR is reached
char receiveFlag = '0';

void main(void) {
	WDTCTL = WDTPW + WDTHOLD; 							// Stop WDT
	rfidSetup();
	ledPortSetup();
	timerSetup();
	__enable_interrupt();
	__bis_SR_register(LPM0);
}

/*
 * RFID and UART Setup
 */
void rfidSetup() {
	RFIDSEL |= BIT3; 									// Set P8.2 as UART RX
	RFIDCTL1 |= UCSWRST; 								// Reset the state machine
	RFIDCTL1 |= UCSSEL_2; 								// SMCLK
	RFIDBR0 = 0x68; 									// 1MHz/9600 = 0x68
	RFIDBR1 = 0x00;
	RFIDMCTL |= UCBRS_1 + UCBRF_0; 						// Modulation UCBRSx=1, UCBRFx=0
	RFIDCTL1 &= ~UCSWRST; 								// Initialize the state machine
	RFIDIE |= UCRXIE;									// Enable RFID Interrupts
}

/*
 * LED pin setup
 */
void ledPortSetup() {
	LEDDIR |= LED;
	LEDOUT &= ~LED;
}

/*
 * Timer setup for LED flash when scanning an RFID tag
 */
void timerSetup() {
	TA0CCTL0 = CCIE;                         			// CCR0 interrupt enabled
	TA0CCR0 = 10000;
	TA0CTL = TASSEL_2;         							// SMCLK, upmode, clear TAR
}

/*
 * Function to start the timer
 */
void startRFIDTimer() {
	TA0CTL |= TACLR + MC_1;
}

/*
 * Separate tag ID from received data
 */
void getIDfromData() {
	int i;
	for(i = 0; i<12; i++) {
		rfidTagID[i] = rfidData[i+1];
	}
}

/*
 * Verify if data was received correctly from RFID reader
 */
int verifyData() {
	if(rfidData[0] == 0x02 && rfidData[13] == 0xD && rfidData[14] == 0xA && rfidData[15] == 0x03) {
		return 1;
	}
	else {
		return 0;
	}
}

/*
 * UART (RFID) Interrupt service routine
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
	switch(__even_in_range(UCA1IV,4)) {
	case 2:                                				// Vector 2 - RXIFG
		rfidData[xbeeRcvIndex] = RFIDRXBUF;
		xbeeRcvIndex++;
		if(xbeeRcvIndex == 16) {
			xbeeRcvIndex = 0;
			if(verifyData()) {
				getIDfromData();
				receiveFlag = '1';
			}
			else {
				receiveFlag = '0';
			}
		}
		else if(xbeeRcvIndex == 1) {
			LEDOUT |= LED;
			startRFIDTimer();
		}
		break;
	default:
		break;
	}
}

/*
 * Timer Interrupt service routine
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {
	timerCount1 += 1;
	if(timerCount1 == 30) {
		timerCount1 = 0;
		LEDOUT &= ~LED;
		TA0CTL &= ~MC_1;
	}
}
