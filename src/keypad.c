#include "msp430f6433.h"

#define KEYPADPORT P3DIR
#define KEYPADIN P3IN
#define KEYPADSEL P3SEL
#define KEYPADOUT P3OUT

unsigned int keyMask, loopCount, keyHex, temp, errorFlag, colInput;
signed int keyVal;
unsigned long compare;
unsigned int lookup[] ={
		0x28,	//0
		0x11,	//1
		0x21, 	//2
		0x41, 	//3
		0x12,	//4
		0x22,	//5
		0x42,	//6
		0x14,	//7
		0x24,	//8
		0x44,	//9
		0x18,	//*
		0x48  	//#
};

void setKeypadPort();
void keypadDebouncing();
void keyScan();
void identifyKey();

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;							// Stop WDT
	__bis_SR_register(GIE);

	while(1) {
		setKeypadPort();
		__bis_SR_register(LPM4_bits);
		__no_operation();

		keypadDebouncing();
		keyScan();
		if(errorFlag != 0) {
			__no_operation();
			continue;
		}
		else {
			identifyKey();
		}
	}
}

void setKeypadPort(){
	KEYPADPORT |= 0xF;									//P3.0-P3.3 as Output
	KEYPADOUT = 0xF;									//Set P3.0-P3.3 Output to 1
	KEYPADPORT &= ~0x70;								//P3.4-P3.6 as Input

	P3IES &= ~0x70;										//Low To High Interrupt Edge
	P3IFG = 0;											//Clear interrupt flag
	P3IE = 0x70;										//Enable interrupt for P1.4-P1.6
	errorFlag = 0;										//Clear Error Flag
}

void keypadDebouncing(void) {
	TA0CTL |= TASSEL_2 + TACLR;							//SMCLK, Clear TimerA
	TA0CCTL0 |= CCIE;									//Enable CCR0 Interrupt
	TA0CCR0 = 2000;										//TC = 5125
	TA0CTL |= MC_1;										//Start timer in up mode
	__bis_SR_register(LPM0_bits);						//Enter LPM0
}

void keyScan(void) {
	keyMask = 1;
	loopCount = 4;
	keyHex = 0;

	KEYPADOUT &= ~0x7F; 								//Clear KEYPADOUT
	KEYPADPORT |= 0x70;									//Set P1.4-P1.6 as Output
	KEYPADOUT &= ~0x70;									//Clear P1.4-P1.6 Output to bleed off charge
	KEYPADPORT &= ~0x70;								//Set P1.4-P1.6 as Input

	KEYPADOUT = keyMask;								//Send keyMask value to KEYPADOUT (to rows)

	while(loopCount > 0) {
		KEYPADOUT = keyMask;							//Send keyMask value to KEYPADOUT (to rows)

		compare = 0;
		compare |= (KEYPADIN & 0x70);
		if(compare > 0) {
			colInput = KEYPADIN;
			colInput = colInput & 0x70;
			keyHex = keyMask + colInput;
			__no_operation();
			break;
		}
		else {
			keyMask = keyMask * 2;
		}
		loopCount--;
	}
	if(keyHex == 0) {
		errorFlag = 1;
	}
	KEYPADOUT |= 0xF;
}

void identifyKey(void) {
	keyVal = 11;
	while(keyHex != lookup[keyVal]) {
		keyVal--;
		if(keyVal < 0) {
			errorFlag = 2;
			return;
		}
	}
	__no_operation();
}

#pragma vector=PORT3_VECTOR
__interrupt void Port_1(void) {
	__bic_SR_register_on_exit(LPM4_bits);
	P3IFG = 0;													// P1IFG cleared
	P3IE = 0;													// Disable P3 Interrupts
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void) {
	__bic_SR_register_on_exit(LPM0_bits);
	TA0CTL = TACLR;
	TA0CCTL0 = 0;
}
