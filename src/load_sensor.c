#include <msp430f6433.h>

void adcSetup();
void takeLoadMeasures();
void dtoa();

#define LOADBIT1        BIT0
#define LOADBIT2        BIT1
#define LOADBIT3        BIT2
#define LOADSEL         P6SEL

long value1 = 0;											// Stores value read from ADCMEM
long value2 = 0;
long value3 = 0;											// Stores value of read voltage
double r1 = 0;												// Calculate Load sensor resistance
double r2 = 0;
double r3 = 0;
double weight = 0;
char weightStr[5];

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;								// Stop watchdog timer
	adcSetup();
	__bis_SR_register(GIE);
	while(1) {
		takeLoadMeasures();
		__no_operation();
	}
}

/*
 * Setup for ADC reference, sampling time, and pins
 */
void adcSetup() {
	LOADSEL |= LOADBIT1 + LOADBIT2 + LOADBIT3;				// Set P6.0 to ADC
	ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC;                   	// Sample and Holt time, turn on ADC
	ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;					// Sampling timer, single sequence
	ADC12MCTL0 = ADC12INCH_0;
	ADC12MCTL1 = ADC12INCH_1;
	ADC12MCTL2 = ADC12INCH_2 + ADC12EOS;
	ADC12IE = LOADBIT3;										// Enable ADC BIT0 interrupt
	ADC12CTL0 |= ADC12ENC;
}

/*
 * Function to take 100 measures on ADC pin and obtain an average value
 */
void takeLoadMeasures() {

	int i;
	for(i=0; i<100; i++) {									// Take 100 samples
		ADC12CTL0 |= ADC12SC;								// Start sampling/conversion
		__bis_SR_register(LPM0_bits + GIE);					// Low power mode while ADC conversion is done
		ADC12CTL0 &= ~ADC12SC;
	}
	value1 = value1 / 100;									// Obtain average value
	value2 = value2 / 100;
	value3 = value3 / 100;

	double voltage1 = 3.0 / 4096 * value1;
	double voltage2 = 3.0 / 4096 * value2;
	double voltage3 = 3.0 / 4096 * value3;

	r1 = 3.0 / voltage1 * 1000000 - 1000000;				// Calculate Load sensor resistance
	r2 = 3.0 / voltage2 * 1000000 - 1000000;
	r3 = 3.0 / voltage3 * 1000000 - 1000000;

	double conductance = (1 / r1) + (1 / r2) + (1 / r3);	// Sum of conductances
	weight = (conductance - 0.0000101953) / 0.0000005592;
	if(weight < 0) {
		weight = 0;
	}

	value1 = 0;
	value2 = 0;
	value3 = 0;
	dtoa(weight);

	__no_operation();
}

void dtoa(double val) {  

	if(weight < 10) {
		weightStr[0] = '0';
		weightStr[1] = '0';
		int w2 = weight;
		weightStr[2] = weight + 0x30;
	}
	else if(weight < 100) {
		weightStr[0] = '0';
		int w1 = weight / 10;
		weightStr[1] = w1 + 0x30;

		int w2 = weight - (w1 * 10);
		weightStr[2] = w2 + 0x30;

	}
	else {
		int w0 = weight / 100;
		weightStr[0] = w0 + 0x30;

		int w1 = (weight - (w0 * 100)) / 10;
		weightStr[1] = w1 + 0x30;

		int w2 = weight - (w0 * 100) - (w1 * 10);
		weightStr[2] = w2 + 0x30;
	}

	weightStr[3] = '.';
	int intWeight = weight;
	double decimal = (weight - intWeight) * 10;
	int intDecimal = decimal;
	weightStr[4] = intDecimal + 0x30;
}

/*
 * ADC Interrupt service routine
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
	switch(__even_in_range(ADC12IV,34)) {
	case 10:												// ADC12IFG2 (Three measures sampled and converted)
		value1 += ADC12MEM0;
		value2 += ADC12MEM1;
		value3 += ADC12MEM2;
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	default:
		break;
	}
}
