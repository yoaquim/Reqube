#include "msp430f6433.h"

#define XBEERX 		BIT1
#define XBEETX 		BIT0
#define XBEESEL		P2SEL

void xbeeSendByte(char *string, int size);

char rfidData[3];
int xbeeRcvIndex = 0;
void main(void) {
	WDTCTL = WDTPW + WDTHOLD; 					// Stop WDT

	__disable_interrupt();
	PMAPPWD = 0x02D52;
	PMAPCTL = PMAPRECFG;
	P2MAP0 = PM_UCA0TXD;
	P2MAP1 = PM_UCA0RXD;
	PMAPPWD = 0;
	__enable_interrupt();

	XBEESEL |= XBEETX + XBEERX; 				// P2.0,1 = USCI_A0 TXD/RXD

	UCA0CTL1 |= UCSWRST; 						// Reset the state machine
	UCA0CTL1 |= UCSSEL_2; 						// SMCLK
	UCA0BR0 = 0x68; 							// 1MHz/9600=0x68
	UCA0BR1 = 0x00;
	UCA0MCTL |= UCBRS_1 + UCBRF_0; 				// Modulation UCBRSx=1, UCBRFx=0
	UCA0CTL1 &= ~UCSWRST; 						// Initialize the state machine

	__delay_cycles(1000);
	
	int i;
	int j;
	
	xbeeSendByte("#123456789ABC1015.8;", 20);
	for(i = 0; i < 125; i++) {
	 	xbeeSendByte("123456789ABC1015.5;", 19);
		xbeeSendByte("8400DASKSDJA1011.2;", 19);
		xbeeSendByte("123456789ABC4020.9;", 19);
		xbeeSendByte("123456789ABC5017.3;", 19);
		__delay_cycles(150000);
	}
	xbeeSendByte("8400DASKSDJA5030.4;*", 20);
	__no_operation();
	/*for(j = 0; j < 1; j++) {
	  	xbeeSendByte("#123456789ABC1015.5;", 20);
		xbeeSendByte("123456789ABC2011.2;", 19);
		xbeeSendByte("123456789ABC4020.9;", 19);
		xbeeSendByte("123456789ABC5017.2;", 19);
		xbeeSendByte("123456789ABC3030.4;", 19);
		__no_operation();
	}*/
	
	UCA0IE |= UCRXIE;
	__bis_SR_register(LPM0);
}

void xbeeSendByte(char *string, int size) {
	int index = 0;
	for(index; index < size; index++) {
		while (!(UCA0IFG&UCTXIFG)) {} 			// Wait for TX buffer ready
		UCA0TXBUF = string[index];
	}
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
	switch(__even_in_range(UCA0IV,4)) {
	case 2:										// Vector 2 - RXIFG
		while (!(UCA0IFG&UCTXIFG));				// USCI_A0 TX buffer ready?
		rfidData[xbeeRcvIndex] = UCA0RXBUF;		// TX -> RXed character
		xbeeRcvIndex++;
		if(xbeeRcvIndex == 3) {
			xbeeRcvIndex = 0;
		}
		break;
	default:
		break;
	}
}