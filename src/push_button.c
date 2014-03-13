#include "msp430f6433.h"

void buttonSetup();

// Button and Indicator LED pins
#define BUTTONDIR	P1DIR
#define BUTTONOUT	P1OUT
#define BUTTONIE	P1IE
#define BUTTONIES	P1IES
#define BUTTONIFG	P1IFG
#define BUTTON		BIT1
#define BTNLED		BIT2

int buttonCount = 0;														// Keeps track of button presses to prove debouncing works

void main(void) {
	WDTCTL = WDTPW + WDTHOLD;											//Stop watchdog timer
	buttonSetup();
	__enable_interrupt();
	__bis_SR_register(LPM0);
}

/*
 * Setup Button and LED pins and interrupts
 */
void buttonSetup() {
	BUTTONDIR &= ~BUTTON;												// P1.1 as Input for push button
	BUTTONDIR |= BTNLED;												// P1.2 as Output for LED
	BUTTONOUT &= ~BTNLED;												// Turn off LED
	BUTTONIE |= BUTTON;													// Enable interrupt for button
	BUTTONIES &= ~BUTTON;												// Interrupt edge rising
}

/*
 * Port 1 Interrupt Service Routine
 */
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
	switch(__even_in_range(P1IV,2)) {
	case 4:																// P1.1 Interrupt
		__delay_cycles(200000);											// Debouncing
		BUTTONOUT ^= BTNLED;											// Toggle LED on button press
		BUTTONIFG &= ~BUTTON;											// Clear interrupt flag
		buttonCount += 1;													// Increase button press count
		break;
	default:
		break;
	}
}
