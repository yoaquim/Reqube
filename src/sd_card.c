#include <msp430f6433.h>
#include "intrinsics.h"
#include "string.h"

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

/********************************
		Function Prototypes
 ********************************/
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
		Variables
 ********************************/
unsigned char status = 1;
unsigned int timeout = 0;
unsigned char buffer[512];
/********************************
		Main
 ********************************/
int main(void) {
	WDTCTL = WDTPW + WDTHOLD;
	spiSetup();
	P1DIR |= BIT2;
	P1OUT &= ~BIT2;
	while(status != 0) {
		status = initializeSD();
		timeout++;
		if(timeout == 150) {
			P1OUT |= BIT2;
			__bis_SR_register(LPM0);																//NO SD Card Found, stop execution
		}
	}

	testSD();
	__bis_SR_register(LPM0);
}

/********************************
		Functions
 ********************************/
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
	char *temp = "This is a test for the SD Card";
	setBufferToString(temp);
	writeSector(0, buffer);

	// Write Data to Sector 1
	setBufferToChar('h');
	writeSector(1, buffer);

	// Write Data to Sector 20
	setBufferToChar('i');
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
		if(buffer[i] != 'h') {
			P1OUT |= BIT2;
		}
	}

	// Read data from Sector 20
	clearBuffer();
	readSector(whereToWrite, buffer);
	for (i = 0; i < 512; i++) {
		if(buffer[i] != 'i') {
			P1OUT |= BIT2;
		}
	}
}

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
