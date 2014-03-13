#include <msp430f6433.h>
#include "intrinsics.h"
#include "string.h"

/****************************
 *	Function Prototypes
 ****************************/
void setKeypadPort();
void keypadDebouncing();
void keyScan();
void identifyKey();
void startWeighing();
void selectMaterial();
void rfidSetup();
void getIDfromData();
void ledPortSetup();
void setupLEDs();
void timerSetup();
void buttonSetup();
char startDataTransmission();
void clearDataPacket();
void startKeypadTimer();
void getAmountOfPackets();
void storeDataOnSD();
void addDataToBuffer();
void xbeeSetup();
void xbeeSendByte(char charac);
void startXBeeTimer();
void adcSetup();
void dtoa();
void clearSD();
void startRFIDTimer();

void spiSetup();
void csLow();
void csHigh();
char initializeSD();
unsigned char spiSendByte(const unsigned char data);
char setToSPI();
void sendCommand(const char cmd, unsigned long arg, const char crc);
char getResponse(void);
unsigned char spiSendFrame(unsigned char* pBuffer, unsigned int size);
unsigned char spiReadFrame(unsigned char* pBuffer, unsigned int size);
char writeBlock (const unsigned long address, const unsigned long count, unsigned char *pBuffer);
char readBlock(const unsigned long address, const unsigned long count, unsigned char *pBuffer);
char getThisResponse(const char resp);
char setBlockLength (const unsigned long blocklength);
char checIfkBusy(void);
#define readSector(sector, pBuffer) readBlock(sector*512ul, 512, pBuffer)
#define writeSector(sector, pBuffer) writeBlock(sector*512ul, 512, pBuffer)
void clearBuffer(void);
void setBufferToChar(char character);
void testSD();
void setBufferToString(char *str);

/********************************
  	Keypad Ports and Pins
 ********************************/
#define KEYPADPORT 		P3DIR
#define KEYPADIN 		P3IN
#define KEYPADSEL 		P3SEL
#define KEYPADOUT 		P3OUT

/********************************
  	LED Ports and Pins
 ********************************/
#define LEDDIR			P9DIR
#define LEDOUT			P9OUT
#define LEDDIR2			P1DIR
#define	LEDOUT2			P1OUT
#define RFIDLED			BIT3
#define LED1			BIT7
#define LED2			BIT6
#define LED3			BIT5
#define LED4			BIT4
#define LED5			BIT3

/********************************
  	RFID Ports and Pins
 ********************************/
#define RFIDSEL			P8SEL
#define RFIDRX			BIT3
#define RFIDCTL1		UCA1CTL1
#define RFIDBR0			UCA1BR0
#define RFIDBR1			UCA1BR1
#define RFIDMCTL		UCA1MCTL
#define RFIDMCTL1		UCA1MCTL1
#define RFIDIE			UCA1IE
#define RFIDRXBUF		UCA1RXBUF

/********************************
  	Push Button Ports and Pins
 ********************************/
#define BUTTONDIR		P1DIR
#define BUTTONOUT		P1OUT
#define BUTTONIE		P1IE
#define BUTTONIES		P1IES
#define BUTTONIFG		P1IFG
#define BUTTON			BIT1
#define BTNLED			BIT3
#define XMITLED			BIT2

/********************************
  	XBee Ports and Pins
 ********************************/
#define XBEERX 		BIT1
#define XBEETX 		BIT0
#define XBEESEL		P2SEL

/********************************
  	Load Sensor Ports and Pins
 ********************************/
#define LOADBIT1        BIT0
#define LOADBIT2        BIT1
#define LOADBIT3        BIT2
#define LOADSEL         P6SEL

/********************************
  	SD Card Ports and Pins
 ********************************/
#define SPISEL         		P8SEL
#define SPIDIR         		P8DIR
#define SIMO          		BIT5
#define SOMI          		BIT6
#define UCLK          		BIT4
#define CSOUT      			P8OUT
#define CSDIR      			P8DIR
#define CS            		BIT7

/********************************
  SPI Status, Flags and Buffers
 ********************************/
#define SPITXBUF  			UCB1TXBUF
#define SPIRXBUF  			UCB1RXBUF
#define SPITXREADY  		(UCB1IFG&UCTXIFG)
#define SPIRXREADY 			(UCB1IFG&UCRXIFG)
/********************************
		SD COMMANDS
 ********************************/
#define CMD_GO_IDLE					0x40     //CMD0
#define CMD_SEND_OP_COND           	0x41     //CMD1 (ACMD41)
#define CMD_SET_BLOCKLENGTH        	0x50     //CMD16 Set block length for next read/write
#define CMD_READ_SINGLE_BLOCK      	0x51     //CMD17 Read block from memory
#define CMD_WRITE_BLOCK            	0x58     //CMD24
#define CMD_DUMMY 					0xFF

/********************************
		Statuses
 ********************************/
#define SUCCESS           			0x00
#define CARD_READY					0x01
#define BLOCK_SET_ERROR   			0x01
#define RESPONSE_ERROR    			0x02
#define DATA_TOKEN_ERROR  			0x03
#define INIT_ERROR        			0x04
#define CRC_ERROR         			0x10
#define WRITE_ERROR       			0x11
#define OTHER_ERROR       			0x12
#define START_DATA_BLOCK_TOKEN		0xFE

/****************************
 * Variables
 ****************************/
char material = 0;														// Type of material to be weighed next (1=Mixed, 2=Plastic, 3=Aluminum, 4=Glass, 5=Paper)
char buttonCount = 0;													// Keeps track of button presses to start XBee data transmission
unsigned int keyMask, loopCount, keyHex, errorFlag, colInput;			//Variables to identify which key was pressed on keypad
signed int keyVal;														// Numerical value of pressed key (0-11)
const unsigned int lookup[] ={											// Hex value of pressed keys (0x028 = Col2, Row4 = 0, etc.)
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
char rfidData[16];													// Buffer to store data received from RFID reader
char rfidTagID[12];													// Buffer to store RFID tag's ID number
int rfidRcvIndex = 0;													// Index to keep track of current received data (from 0 to 15: 16 bytes of data)
int timerCount1 = 0;													// Counter to keep track of times the Timer0 ISR is reached
int timerCount2 = 0;													// Counter to keep track of times the Timer1 ISR is reached
int timerCount3 = 0;													// Counter to keep track of times the Timer2 ISR is reached
int timerCount4 = 0;													// Counter to keep track of times the Timer3 ISR is reached
char buttonInterrupt = 0;												// Keeps track of button presses for main program functionality
char rfidInterrupt = 0;													// Keeps track of RFID scans for main program functionality
char keypadInterrupt = 0;												// Keeps track of keypad presses for main program functionality
char failedTransmit = 1;
char nextToStore[18];													// Next packet to store on the SD Card
char tagValid = 0;													// Flag to indicate if last RFID tag was scanned correctly
unsigned char status = 1;												// Status of the SD card (0 means ready)
unsigned char buffer[512];												// Buffer for SD card reading/writing (512 bytes in size)
char nextSpaceInBuffer = 0;												// Last space used in current SD card sector
char lastSectorUsed = 0;												// Last sector of SD card used
char xbeeRcvd[3];
int xbeeRcvIndex = 0;
char ackReceived = 0;													// Flag indicating if XBee received an acknowledge from the java app
long value1 = 0;													// Stores value read from ADC channel 0
long value2 = 0;													// Stores value read from ADC channel 1
long value3 = 0;                                                       							// Stores value read from ADC channel 2
double r1 = 0;														// Value of calculated load sensor resistance
double r2 = 0;														// Value of calculated load sensor resistance
double r3 = 0;														// Value of calculated load sensor resistance
double weight = 0;													// Calculated weight

/****************************
 * Main
 ****************************/
int main(void) {
	WDTCTL = WDTPW | WDTHOLD;											// Stop watchdog timer
	setKeypadPort();
	setupLEDs();
	rfidSetup();
	timerSetup();
	buttonSetup();
	spiSetup();
	xbeeSetup();
	adcSetup();

	// SD Card initialization
	while(status != 0) {												// Wait for SD card
		status = initializeSD();
	}
	
	readSector(0, buffer);
	__no_operation();
	readSector(1, buffer);
	__no_operation();
	readSector(2, buffer);
	__no_operation();
	readSector(3, buffer);
	__no_operation();
	
	getAmountOfPackets();							// Get last used slot and next space in buffer from SD

	while(1) {
		__bis_SR_register(LPM0_bits + GIE);
		if(keypadInterrupt == 1) {										// Keypad Interrupt
			keypadDebouncing();
			keyScan();
			if(errorFlag == 0) {
				identifyKey();											// Identify which key was pressed
				if(tagValid == 1) {
					__disable_interrupt();								// Disable interrupts while writing to SD					
					startWeighing();									// Call function to take weight measures
					selectMaterial();									// Select material type and turn on corresponding LED
					storeDataOnSD();									// Store last collected data on SD Card
					clearDataPacket();
					tagValid = 0;
					__enable_interrupt();								// Re-enable interrupts
					startKeypadTimer();
				}
			}
			keypadInterrupt = 0;
			setKeypadPort();
		}
		if(buttonInterrupt == 1 && buttonCount == 2){					// Button Interrupt
			buttonInterrupt = 0;
			buttonCount = 0;
			BUTTONOUT &= ~BTNLED;
			LEDOUT = LED1 + LED3 + LED5;
			int tryLimit = 5;
			BUTTONIE &= ~BUTTON;
			while(tryLimit > 0 && ackReceived == 0) {
				startDataTransmission();
				xbeeRcvIndex = 0;
				tryLimit-= 1;
			}
			BUTTONIFG &= ~BUTTON;
			BUTTONIE |= BUTTON;
			LEDOUT =0;
			if(ackReceived == 1) {
			  	clearSD();
				failedTransmit = 2;
				BUTTONOUT |= RFIDLED;
			}
			else if(ackReceived == 0) {
			  	failedTransmit = 1;
			  	BUTTONOUT |= XMITLED;
			}
			startRFIDTimer();					// Start timer to toggle Sucess LED
			ackReceived = 0;
			int l;
			for(l=0; l<3; l++) {
				xbeeRcvd[l] = 0;
			}
		}
	}
}

/****************************
 * Main program functions
 *************************** */
void selectMaterial() {
	LEDOUT = 0;
	switch(keyVal) {
	case 1:
		material = '1';
		LEDOUT |= LED1;
		break;
	case 2:
		material = '2';
		LEDOUT |= LED2;
		break;
	case 3:
		material = '3';
		LEDOUT |= LED3;
		break;
	case 4:
		material = '4';
		LEDOUT |= LED4;
		break;
	case 5:
		material = '5';
		LEDOUT |= LED5;
		break;
	default:
		material = '1';
		LEDOUT |= LED1;
		break;
	}
	nextToStore[12] = material;
}

void startWeighing() {
	ADC12IE = LOADBIT3;
	int i;
	for(i=0; i<100; i++) {												// Take 100 samples
		ADC12CTL0 |= ADC12SC;											// Start sampling/conversion
		__bis_SR_register(LPM0_bits + GIE);								// Low power mode while ADC conversion is done
		ADC12CTL0 &= ~ADC12SC;
	}
	ADC12IE &= ~LOADBIT3;
	value1 = value1 / 100;												// Obtain average value
	value2 = value2 / 100;
	value3 = value3 / 100;

	double voltage1 = 3.0 / 4096 * value1;
	double voltage2 = 3.0 / 4096 * value2;
	double voltage3 = 3.0 / 4096 * value3;

	r1 = 3.0 / voltage1 * 1000000 - 1000000;							// Calculate Load sensor resistance
	r2 = 3.0 / voltage2 * 1000000 - 1000000;
	r3 = 3.0 / voltage3 * 1000000 - 1000000;

	double conductance = (1 / r1) + (1 / r2) + (1 / r3);			// Sum of conductances
	weight = (conductance - 0.0000101953) / 0.0000005592;
	if(weight < 0) {
		weight = 0;
	}

	value1 = 0;
	value2 = 0;
	value3 = 0;
	dtoa(weight);
}

void dtoa(double val) {  

	if(weight < 10) {
		nextToStore[13] = '0';
		nextToStore[14] = '0';
		int w2 = weight;
		nextToStore[15] = w2 + 0x30;
	}
	else if(weight < 100) {
		nextToStore[13] = '0';
		int w1 = weight / 10;
		nextToStore[14] = w1 + 0x30;

		int w2 = weight - (w1 * 10);
		nextToStore[15] = w2 + 0x30;

	}
	else {
		int w0 = weight / 100;
		nextToStore[13] = w0 + 0x30;

		int w1 = (weight - (w0 * 100)) / 10;
		nextToStore[14] = w1 + 0x30;

		int w2 = weight - (w0 * 100) - (w1 * 10);
		nextToStore[15] = w2 + 0x30;
	}

	nextToStore[16] = '.';
	int intWeight = weight;
	double decimal = (weight - intWeight) * 10;
	int intDecimal = decimal;
	nextToStore[17] = intDecimal + 0x30;
}

char startDataTransmission() {
	if(!(lastSectorUsed == 0 && nextSpaceInBuffer == 0)) {			// Check if there is data to send
		unsigned long i;
		xbeeSendByte('#');
		for(i=0; i <= lastSectorUsed; i++) {
			readSector(i, buffer);
			int limit = 18 * buffer[510];
			int j;
			for(j = 0; j < limit; j++) {
				xbeeSendByte(buffer[j]);
				if((j+1)%18 == 0) {
					xbeeSendByte(';');
				}
			}
			__delay_cycles(100000);
		}
		xbeeSendByte('*');
		startXBeeTimer();
		__bis_SR_register(LPM0_bits);
	}
	else {
	  	ackReceived = 2;
	}
	return ackReceived;
}

void clearDataPacket() {
	int i;
	for(i=0; i<18; i++) {
		nextToStore[i] = 0;
	}
}

/*
 * Obtains the amount of data packets that have been stored on the SD Card
 */
void getAmountOfPackets() {
	readSector(0, buffer);
	lastSectorUsed = buffer[511];
	nextSpaceInBuffer = buffer[510];
	if(lastSectorUsed > 0) {
		readSector(lastSectorUsed, buffer);
		nextSpaceInBuffer = buffer[510];
	}
}

/*
 * Stores latest collected RFID tag, weight and material type to SD
 */
void storeDataOnSD() {	
	readSector(lastSectorUsed, buffer);					// Read Sector (so as to not overwrite existing data)
	addDataToBuffer();							// Add new data to the buffer
	nextSpaceInBuffer++;							// Increase buffer space count (for next data packet)
	buffer[510] = nextSpaceInBuffer;					// Store this value on the buffer
	char newSector = 0;							// Flag to check if the next packet will be written to a new SD sector
	char tempLastSector = lastSectorUsed;					// Current sector being written to
	
	if(nextSpaceInBuffer == 28) {						// Out of space in current sector? (if current sector contains 28 data packets)
	  	lastSectorUsed++;						// Go to next sector
		newSector = 1;							// Set sector flag to true
		nextSpaceInBuffer = 0;						// Beginning of new sector (space 0 of buffer)
	}
	
	if(tempLastSector == 0) {						// If current data being written to sector 0
		buffer[511] = lastSectorUsed;					// Store sector count on sector 0
		writeSector(tempLastSector, buffer);				// Write changes to sector
	}
	else {									// Not on sector 0
		writeSector(tempLastSector, buffer);				// Write changes to sector
		readSector(0, buffer);						// Read data from sector 0
		buffer[511] = lastSectorUsed;					// Store sector count on sector 0
		writeSector(0, buffer);						// Write changes to sector
	}
	
	if(newSector == 1) {							// If next data will be written to a new sector
	  	clearBuffer();							// Empty the buffer
		buffer[510] = 0;
		writeSector(lastSectorUsed, buffer);				// Write changes
	}
}

/*
 * Add newest collected data to the read/write buffer
 */
void addDataToBuffer() {
	int i;
	int j = nextSpaceInBuffer * 18;
	for(i=0; i<18; i++) {
		buffer[j++] = nextToStore[i];
	}
	__no_operation();
}

void clearSD() {
	int i;	
  	for(i = 0; i <= lastSectorUsed; i++) {
		clearBuffer();
		if(i == 0) {
			buffer[510] = 0;
			buffer[511] = 0;
			lastSectorUsed = 0;
			nextSpaceInBuffer = 0;
		}
		writeSector(i, buffer);
	}
}

/****************************
 * ADC functions
 ****************************/

/*
 * Setup for ADC reference, sampling time, and pins
 */
void adcSetup() {
	LOADSEL |= LOADBIT1 + LOADBIT2 + LOADBIT3;							// Set P6.0 to ADC
	ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC;                   	// Sample and Holt time, turn on ADC
	ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;								// Sampling timer, single sequence
	ADC12MCTL0 = ADC12INCH_0;
	ADC12MCTL1 = ADC12INCH_1;
	ADC12MCTL2 = ADC12INCH_2 + ADC12EOS;
	ADC12IE = LOADBIT3;													// Enable ADC BIT0 interrupt
	ADC12CTL0 |= ADC12ENC;
}

/****************************
 * Timer functions
 ****************************/

/*
 * Initialize timer
 */
void timerSetup() {
	TA0CTL = TASSEL_2 + TACLR;         									// SMCLK, up mode, clear TAR
	TA0CCTL0 = CCIE;                         							// CCR0 interrupt enabled
	TA0CCR0 = 10000;

	TA1CTL = TASSEL_2 + TACLR;
	TA1CCTL0 = CCIE;
	TA1CCR0 = 50000;

	TA2CTL = TASSEL_2 + TACLR;
	TA2CCTL0 = CCIE;
	TA2CCR0 = 50000;

	TB0CTL = TASSEL_2 + TACLR;
	TB0CCTL0 = CCIE;
	TB0CCR0 = 50000;
}

/*
 * Function to start the timer
 */
void startRFIDTimer() {
	TA0CCR0 = 10000;
	TA0CTL |= MC_1;
}

/*
 * Wait function for keypad debouncing
 */
void keypadDebouncing(void) {
	TA0CCR0 = 2500;
	TA0CTL |= MC_1;														//Start timer in up mode
	__bis_SR_register(LPM0_bits);										//Enter LPM0
}

void startPushButtonTimer() {
	TA1CCR0 = 50000;
	TA1CTL |= MC_1;
}

void startKeypadTimer() {
	TA2CCR0 = 50000;
	TA2CTL |= MC_1;
}

void startXBeeTimer() {
	TB0CCR0 = 50000;
	TB0CTL |= MC_1;
}
/****************************
 * Keypad Functions
 ****************************/

/*
 * Setup keypad port and pins
 */
void setKeypadPort(){
	KEYPADPORT |= 0xF;													//P3.0-P3.3 as Output
	KEYPADOUT = 0xF;													//Set P3.0-P3.3 Output to 1
	KEYPADPORT &= ~0x70;												//P3.4-P3.6 as Input

	P3IES &= ~0x70;														//Low To High Interrupt Edge
	P3IFG = 0;															//Clear interrupt flag
	P3IE = 0x70;														//Enable interrupt for P1.4-P1.6
	errorFlag = 0;														//Clear Error Flag
}

/*
 * Obtain the hex value of the pressed key in the format (0xXY where X = 2^Col and Y = 2^Row)
 */
void keyScan(void) {
	keyMask = 1;
	loopCount = 4;
	keyHex = 0;

	KEYPADOUT &= ~0x7F; 												//Clear KEYPADOUT
	KEYPADPORT |= 0x70;													//Set P1.4-P1.6 as Output
	KEYPADOUT &= ~0x70;													//Clear P1.4-P1.6 Output to bleed off charge
	KEYPADPORT &= ~0x70;												//Set P1.4-P1.6 as Input

	KEYPADOUT = keyMask;												//Send keyMask value to KEYPADOUT (to rows)

	while(loopCount > 0) {
		KEYPADOUT = keyMask;											//Send keyMask value to KEYPADOUT (to rows)

		unsigned long compare = 0;
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

/*
 * Identify which key of the keypad was pressed and assign it a number (from 0 to 11)
 */
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

/****************************
 * RFID Functions
 ****************************/

/*
 * RFID and UART Setup
 */
void rfidSetup() {
	RFIDSEL |= RFIDRX; 													// Set P8.2 as UART RX
	RFIDCTL1 |= UCSWRST; 												// Reset the state machine
	RFIDCTL1 |= UCSSEL_2; 												// SMCLK
	RFIDBR0 = 0x68; 													// 1MHz/9600 = 0x68
	RFIDBR1 = 0x00;
	RFIDMCTL |= UCBRS_1 + UCBRF_0; 										// Modulation UCBRSx=1, UCBRFx=0
	RFIDCTL1 &= ~UCSWRST; 												// Initialize the state machine
	RFIDIE |= UCRXIE;													// Enable RFID Interrupts
}

/*
 * Separate tag ID from received data
 */
void getIDfromData() {
	int i;
	for(i = 0; i < 12; i++) {
		nextToStore[i] = rfidData[i+1];
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

/****************************
 * Button Functions
 ****************************/

/*
 * Setup Button and LED pins and interrupts
 */
void buttonSetup() {
	BUTTONDIR &= ~BUTTON;												// P1.1 as Input for push button
	BUTTONDIR |= BTNLED + XMITLED;												// P1.2 as Output for LED
	BUTTONOUT &= ~BTNLED + XMITLED;												// Turn off LED
	BUTTONIE |= BUTTON;													// Enable interrupt for button
	BUTTONIES &= ~BUTTON;												// Interrupt edge rising
}

/****************************
 * LED Functions
 ****************************/

/*
 * Setup LED port and make sure all LEDs are off at the start of the program
 */
void setupLEDs() {
	LEDDIR |= LED1 + LED2 + LED3 + LED4 + LED5;
	LEDOUT = 0;

	LEDDIR2 |= RFIDLED;
	LEDOUT2 = 0;
}

/****************************
 * XBee Functions
 ****************************/

/*
 * Initialize XBee pins and ports
 */
void xbeeSetup() {
	__disable_interrupt();
	PMAPPWD = 0x02D52;
	PMAPCTL = PMAPRECFG;
	P2MAP0 = PM_UCA0TXD;
	P2MAP1 = PM_UCA0RXD;
	PMAPPWD = 0;
	//__enable_interrupt();

	XBEESEL |= XBEETX + XBEERX; 											// P2.0,1 = USCI_A0 TXD/RXD

	UCA0CTL1 |= UCSWRST; 													// Reset the state machine
	UCA0CTL1 |= UCSSEL_2; 													// SMCLK
	UCA0BR0 = 0x68; 														// 1MHz/9600=0x68
	UCA0BR1 = 0x00;
	UCA0MCTL |= UCBRS_1 + UCBRF_0; 											// Modulation UCBRSx=1, UCBRFx=0
	UCA0CTL1 &= ~UCSWRST; 													// Initialize the state machine
	UCA0IE |= UCRXIE;
}

void xbeeSendByte(char charac) {
	while (!(UCA0IFG&UCTXIFG)) {} 											// Wait for TX buffer ready
	UCA0TXBUF = charac;
}

/****************************
 * Interrupt Service Routines
 ****************************/

/*
 * Push Button ISR
 */
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
	switch(__even_in_range(P1IV,2)) {
	case 4:																	// P1.1 Interrupt
		__delay_cycles(200000);												// Button Debouncing
		BUTTONIFG &= ~BUTTON;												// Clear interrupt flag
		buttonInterrupt = 1;
		if(buttonCount == 0) {
			buttonCount = 1;
			startPushButtonTimer();
			BUTTONOUT |= BTNLED;
		}
		else if(buttonCount == 1) {
			buttonCount = 2;
			TA1CTL &= ~MC_1;
			TA1CTL |= TACLR;
			timerCount2 = 0;
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;
	default:
		break;
	}
}

/*
 * Keypad ISR
 */
#pragma vector=PORT3_VECTOR
__interrupt void Port_1(void) {
	__bic_SR_register_on_exit(LPM0_bits);
	P3IFG = 0;																// P1IFG cleared
	P3IE = 0;																// Disable P3 Interrupts
	keypadInterrupt = 1;
	__no_operation();
}

/*
 * UART (RFID) Interrupt service routine
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
	switch(__even_in_range(UCA1IV,4)) {
	case 2:                                									// Vector 2 - RXIFG
		rfidData[rfidRcvIndex] = RFIDRXBUF;
		rfidRcvIndex++;
		if(rfidRcvIndex == 16) {
			rfidRcvIndex = 0;
			if(verifyData()) {
				__bic_SR_register_on_exit(LPM0_bits);
				getIDfromData();
				LEDOUT2 |= RFIDLED;
				rfidInterrupt = 1;
				tagValid = 1;
				startRFIDTimer();
			}
			else {
			}
		}
		break;
	default:
		break;
	}
}

/*
 * XBee ISR (for ACK reception)
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
	switch(__even_in_range(UCA0IV,4)) {
	case 2:																	// Vector 2 - RXIFG
		//while (!(UCA0IFG&UCTXIFG));										// USCI_A0 TX buffer ready?
		xbeeRcvd[xbeeRcvIndex] = UCA0RXBUF;									// TX -> RXed character
		xbeeRcvIndex++;
		if(xbeeRcvIndex == 3) {
			xbeeRcvIndex = 0;
			if(xbeeRcvd[0] != 'a' || xbeeRcvd[1] != 'c' || xbeeRcvd[2] != 'k') {
				//TODO
				ackReceived = 0;
			}
			else {
				ackReceived = 1;
				TB0CTL &= ~MC_1;
				TB0CTL |= TACLR;
				timerCount4 = 0;
			}
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;
	default:
		break;
	}
}

/*
 * Timer A0 ISR (for Keypad debouncing and RFID LED)
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {
	if(keypadInterrupt == 1) {												// Keypad Debouncing
		__bic_SR_register_on_exit(LPM0_bits);
		TA0CTL &= ~MC_1;
		TA0CTL |= TACLR;
	}
	else if(rfidInterrupt == 1) {											// RFID LED Delay
		timerCount1 += 1;
		if(timerCount1 == 30) {
			timerCount1 = 0;												// Reset timer count
			LEDOUT2 &= ~RFIDLED;											// Turn off RFID LED
			TA0CTL &= ~MC_1;
			TA0CTL |= TACLR;
			rfidInterrupt = 0;
		}
	}
	else if(failedTransmit == 1) {
	  	timerCount1 += 1;
		if(timerCount1 == 200) {
			timerCount1 = 0;												// Reset timer count
			BUTTONOUT &= ~XMITLED;											// Turn off RFID LED
			TA0CTL &= ~MC_1;
			TA0CTL |= TACLR;
			failedTransmit = 0;
		}
	}
	else if(failedTransmit == 2) {
	  	timerCount1 += 1;
		if(timerCount1 == 200) {
			timerCount1 = 0;												// Reset timer count
			BUTTONOUT &= ~RFIDLED;											// Turn off RFID LED
			TA0CTL &= ~MC_1;
			TA0CTL |= TACLR;
			failedTransmit = 0;
		}
	}
}

/*
 * Timer A1 ISR (for Push Button: worker has two seconds to press button twice to start data transmission)
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) {
	if(buttonInterrupt == 1) {
		timerCount2 += 1;
		if(timerCount2 == 40) {
			timerCount2 = 0;
			buttonInterrupt = 0;
			buttonCount = 0;
			BUTTONOUT &= ~BTNLED;
			TA1CTL &= ~MC_1;
			TA1CTL |= TACLR;
		}
	}
}

/*
 * Timer A2 ISR (for Keypad LEDs)
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void) {
	timerCount3 += 1;
	if(timerCount3 == 40) {
		timerCount3 = 0;
		LEDOUT = 0;
		material = 0;
		TA2CTL &= ~MC_1;
		TA2CTL |= TACLR;
	}
}

/*
 * Timer A3 ISR (for XBee  ACK reception)
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
	timerCount4 += 1;
	if(timerCount4 == 150) {
		timerCount4 = 0;
		xbeeRcvIndex = 0;
		//LEDOUT = 0;
		//material = 0;
		TB0CTL &= ~MC_1;
		TB0CTL |= TACLR;
		__bic_SR_register_on_exit(LPM0_bits);
		ackReceived = 0;
	}
}

/*
 * ADC ISR
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
	switch(__even_in_range(ADC12IV,34)) {
	case 10:																// ADC12IFG2 (Three measures sampled and converted)
		value1 += ADC12MEM0;
		value2 += ADC12MEM1;
		value3 += ADC12MEM2;
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	default:
		break;
	}
}

/****************************
 * SD Card Functions
 ****************************/
/*
 * Setup for SPI port, clock, baud rate, etc.
 */
void spiSetup() {
	CSOUT |= CS;
	CSDIR |= CS;
	SPISEL |= SIMO + SOMI + UCLK;
	UCB1CTL0 = UCMST+UCCKPL+UCMSB+UCSYNC;     									// 3-pin, 8-bit SPI master
	UCB1CTL1 = UCSSEL_2+UCSWRST;              									// SMCLK
	UCB1BR0 = 0x02;                          									// UCLK/2
	UCB1BR1 = 0;
	UCB1CTL1 &= ~UCSWRST;														// **Initialize USCI state machine**
}

/*
 * Set Chip select to low
 */
void csLow() {																	// Chip Select to Low
	CSOUT &= ~CS;
}

/*
 * Set Chip select to high
 */
void csHigh() {																	// Chip Select to High
	CSOUT |= CS;
}

/*
 * Send initial dummy clocks to ready SD card
 */
char initializeSD() {
	csHigh();
	int n;

	//send 80 clock cycles to ready SD card
	for (n = 10; n; n--) {
		spiSendByte(CMD_DUMMY);
	}

	return setToSPI();
}

/*
 * Send a byte to SD via SPI and obtain its response
 */
unsigned char spiSendByte(const unsigned char data) {
	while (!SPITXREADY);										// wait while not ready for TX
	SPITXBUF = data;
	while (!SPIRXREADY);    									// wait for RX buffer
	char ret = UCB1RXBUF;
	__no_operation();
	return ret;
}

/*
 * Set SD to work in SPI mode
 */
char setToSPI() {
	char response = 0x01;
	csLow();

	// Send CMD0 to put SD in SPI mode
	sendCommand(CMD_GO_IDLE,0,0x95);

	// Check if SD Card sends Ready Response
	if(getResponse() != CARD_READY) {
		return INIT_ERROR;
	}

	// Wait until card is done sending CARD_READY response by sending dummy clocks
	while(response == CARD_READY) {
		csHigh();
		spiSendByte(CMD_DUMMY);
		csLow();
		sendCommand(CMD_SEND_OP_COND,0x00,0xff);
		response = getResponse();
	}

	csHigh();
	spiSendByte(CMD_DUMMY);
	return SUCCESS;
}

/*
 * Send a full command (6 bytes) to SD card
 */
void sendCommand(const char cmd, unsigned long arg, const char crc) {
	unsigned char frame[6];
	char temp;
	int i;

	// Commands are in the form 4x where x is the command
	frame[0] = (cmd|0x40);
	for(i=3;i>=0; i--){
		temp = (char)(arg>>(8*i));
		frame[4-i]=(temp);
	}
	frame[5] = (crc);

	// Send full command via SPI
	spiSendFrame(frame,6);
}

/*
 * Send a full commmand frame to SD card via SPI
 */
unsigned char spiSendFrame(unsigned char* pBuffer, unsigned int size) {
	unsigned long i = 0;
	for (i = 0; i < size; i++){
		while (!SPITXREADY);
		SPITXBUF = pBuffer[i];
		while (!SPIRXREADY);
		pBuffer[i] = SPIRXBUF;
	}
	return 0;
}

// Try to get a SUCCESS or CARD_READY response from SD card after sending a command or byte
char getResponse(void) {
	int i=0;
	char response;
	while(i<=64) {
		response = spiSendByte(CMD_DUMMY);
		if(response==0x00) {
			break;
		}
		if(response==0x01) {
			break;
		}
		i++;
	}
	return response;
}

/*
 * Read a specified amount of responses from SD card
 */
unsigned char spiReadFrame(unsigned char* pBuffer, unsigned int size) {
	unsigned long i = 0;
	for (i = 0; i < size; i++){

		// Send dummy command to get response
		while (!SPITXREADY);
		SPITXBUF = CMD_DUMMY;

		// Get response
		while (!SPIRXREADY);
		pBuffer[i] = SPIRXBUF;
	}
	return 0;
}

/*
 * Write to a specific block on the SD card
 */
char writeBlock (const unsigned long address, const unsigned long count, unsigned char *pBuffer) {
	char rvalue = RESPONSE_ERROR;

	// Set block length to write
	if (setBlockLength(count) == SUCCESS) {
		csLow();
		sendCommand(CMD_WRITE_BLOCK, address, 0xFF);
		if (getThisResponse(SUCCESS) == SUCCESS) {
			spiSendByte(CMD_DUMMY);

			// Send data token to signify the start of the data
			spiSendByte(0xFE);

			spiSendFrame(pBuffer, count);

			// Send dummy clocks
			spiSendByte(CMD_DUMMY);
			spiSendByte(CMD_DUMMY);

			checIfkBusy();
			rvalue = SUCCESS;
		}
		else {
			// SD never acknowledged the write command
			rvalue = RESPONSE_ERROR;
		}
	}
	else {
		// Error in setting block length
		rvalue = BLOCK_SET_ERROR;
	}

	csHigh();

	// Send dummy clocks
	spiSendByte(CMD_DUMMY);

	return rvalue;
}

char readBlock(const unsigned long address, const unsigned long count, unsigned char *pBuffer) {
	char rvalue = RESPONSE_ERROR;
	// Set the block length to read
	if (setBlockLength(count) == SUCCESS) {   // block length could be set
		csLow();
		sendCommand(CMD_READ_SINGLE_BLOCK,address, 0xFF);

		// Check if SD acknowledged the read block command
		if (getResponse() == 0x00) {
			if (getThisResponse(START_DATA_BLOCK_TOKEN) == START_DATA_BLOCK_TOKEN) {
				spiReadFrame(pBuffer, count);
				spiSendByte(CMD_DUMMY);
				spiSendByte(CMD_DUMMY);
				rvalue = SUCCESS;
			}
			else {
				// the data token was never received
				rvalue = DATA_TOKEN_ERROR;
			}
		}
		else {
			// SD didn't acknowledge read block command
			rvalue = RESPONSE_ERROR;
		}
	}
	else {
		// Failed to set block length
		rvalue = BLOCK_SET_ERROR;
	}
	csHigh();
	spiSendByte(CMD_DUMMY);
	return rvalue;
}

/*
 * Set SD card read and write block length
 */
char setBlockLength(const unsigned long blocklength) {
	csLow();

	// Set the block length to read
	sendCommand(CMD_SET_BLOCKLENGTH, blocklength, 0xFF);

	// Make sure SD card responds with 0x00 (Success response)
	if(getResponse()!=SUCCESS) {
		initializeSD();
		sendCommand(CMD_SET_BLOCKLENGTH, blocklength, 0xFF);
		getResponse();
	}
	csHigh();

	// Send 8 Clock pulses (delay).
	spiSendByte(CMD_DUMMY);

	return SUCCESS;
}

/*
 * Tries to get a specific response (resp) from SD card.
 * Should be used after sending the correct command.
 */
char getThisResponse(const char resp) {
	int i=0;
	char response;
	while(i<=1000) {
		response=spiSendByte(CMD_DUMMY);
		if(response==resp)break;
		i++;
	}
	return response;
}

/*
 * Check if SD card is busy
 */
char checIfkBusy(void) {
	int i=0;
	char response;
	char rvalue;
	while(i<=64) {
		response = spiSendByte(CMD_DUMMY);
		response &= 0x1f;
		switch(response) {
		case 0x05:
			rvalue=SUCCESS;
			break;
		case 0x0b:
			return(CRC_ERROR);
		case 0x0d:
			return(WRITE_ERROR);
		default:
			rvalue = OTHER_ERROR;
			break;
		}
		if(rvalue == SUCCESS)
			break;
		i++;
	}
	i=0;
	do {
		response = spiSendByte(CMD_DUMMY);
		i++;
	} while(response == 0);
	return response;
}

/*
 * Clear buffer used to send and receive data from SD
 */
void clearBuffer(void) {
	int i = 0;
	for(i = 0; i<512; i++) {
		buffer[i] = 0;
	}
}

/*
 * Set the send/receive buffer to contain a certain character repeated 512 times
 */
void setBufferToChar(char character) {
	int i = 0;
	for(i = 0; i<512; i++) {
		buffer[i] = character;
	}
}

/*
 * Set the buffer to hold a string
 */
void setBufferToString(char *str) {
	int i;
	for (i = 0; i < strlen(str); i++) {
		buffer[i] = str[i];
	}
}

/*
 * Add a string to a specific index of the send/receive buffer
 */
void addStringAt(char *str, int index) {
	int i;
	for (i = 0; i < strlen(str); i++) {
		buffer[i] = str[i];
	}
}

void testSD() {
	int i = 0;
	const unsigned long whereToWrite = 64511;

	//Clear Sectors
	clearBuffer();
	writeSector(0, buffer);
	writeSector(1, buffer);
	writeSector(whereToWrite, buffer);

	// Write Data to Sector 0
	clearBuffer();
	char *temp = "Darryl";
	setBufferToString(temp);
	writeSector(0, buffer);

	// Write Data to Sector 1
	setBufferToChar('a');
	writeSector(1, buffer);

	// Write Data to Sector 'whereToWrite'
	setBufferToChar('b');
	writeSector(whereToWrite, buffer);

	// Read data from Sector 0
	clearBuffer();
	readSector(0, buffer);
	for (i = 0; i < 512; i++) {
		if(i < strlen(temp)) {
			if(buffer[i] != temp[i]) {
				P1OUT |= BIT2;
			}
		}
		else {
			if(buffer[i] != 0) {
				P1OUT |= BIT2;
			}
		}
	}

	// Read data from Sector 1
	clearBuffer();
	readSector(1, buffer);
	for (i = 0; i < 512; i++) {
		if(buffer[i] != 'a') {
			P1OUT |= BIT2;
		}
	}

	// Read data from Sector 20
	clearBuffer();
	readSector(whereToWrite, buffer);
	for (i = 0; i < 512; i++) {
		if(buffer[i] != 'b') {
			P1OUT |= BIT2;
		}
	}
}
