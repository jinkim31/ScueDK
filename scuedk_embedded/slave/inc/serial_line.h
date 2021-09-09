/*
 * serial_line.h
 *
 *  Created on: Feb 15, 2021
 *      Author: Jin Kim
 *
 *  SerialLine dataduct C implementation for low-performance MCUs.
 *  Tested on STM32F3(60MHz)
 */

#ifndef SERIALLINE_INC_SERIAL_LINE_H_
#define SERIALLINE_INC_SERIAL_LINE_H_
#include "main.h"

#define RX_BUFFER_SIZE 300

#define HEADER_1 0
#define HEADER_2 1
#define HEADER_3 2
#define RESERVED 3
#define RX_STATE_ID 4
#define LENGTH_1 5
#define LENGTH_2 6
#define INSTRUCTION 7
#define PARAMETER 8
#define CRC_1 9
#define CRC_2 10

#define ADDRESS_1 0
#define ADDRESS_2 1
#define SIZE 2
#define VALUE 3

typedef struct
{
	uint8_t id;
	size_t dataStructSize;
	uint8_t *dataStructurePtr;
	USART_TypeDef* usart;
	int nominalPacketCnt;
	int state;
	uint8_t packetBuffer[RX_BUFFER_SIZE];
	int length;
	int trialCnt;
	unsigned short expectedCRC, actualCRC;
	uint8_t failByte;
	int packetBufferSize;
	int nominalTransmitCnt;
	int test;
}SerialLine;

void initSerialLine(SerialLine* serialLine, uint8_t id, uint8_t* dataStructurePtr, size_t dataStructureSize, USART_TypeDef* usart);

int pushByte(SerialLine *serialLine, uint8_t byte);

int pushPacket(SerialLine *serialLine, uint8_t packet[], int length);

void freeSerialLine(SerialLine *serialLine);

#endif /* SERIALLINE_INC_SERIAL_LINE_H_ */
