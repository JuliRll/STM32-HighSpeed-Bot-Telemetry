/*
 * protocol_handler.c
 *
 *  Created on: Jan 22, 2025
 *      Author: jrlle
 */
#include "protocol_handler.h"
#include <string.h>

/* Estado del protocolo */
typedef enum {
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
	ID,
    PAYLOAD,
	CHECKSUM
} ProtocoloEstado;

static ProtocoloEstado estado = START;

void decodeProtocol(_sRingBuffer *estructura) {
	uint8_t byteActual, checksum = 0;
	int8_t nBytes = 0;

	while (estructura->Rxir != estructura->Rxiw) {
		byteActual = estructura->bufferRx[estructura->Rxir++];

		switch (estado) {
			case START:
				estado = (byteActual == START_BYTE) ? HEADER_1 : START;
				break;

			case HEADER_1:
				estado = (byteActual == HEADER_1_BYTE) ? HEADER_2 : START;
				break;

			case HEADER_2:
				estado = (byteActual == HEADER_2_BYTE) ? HEADER_3 : START;
				break;

			case HEADER_3:
				estado = (byteActual == HEADER_3_BYTE) ? NBYTES : START;
				break;

			case NBYTES:
				nBytes = byteActual;
				estado = TOKEN;
				break;

			case TOKEN:
				if (byteActual == TOKEN_BYTE) {
					estado = ID;
					checksum = START_BYTE ^ HEADER_1_BYTE ^ HEADER_2_BYTE ^ HEADER_3_BYTE ^ nBytes ^ TOKEN_BYTE;
					estructura->payloadRxLength = 0;
				} else {
					estado = START; // Reinicia en caso de error
				}
				break;
			case ID:
				estructura->id = byteActual;
				checksum ^= byteActual;
				nBytes--;
				if(nBytes == 0){
					estado = CHECKSUM;
				}else{
					estado = PAYLOAD;
				}
				break;
			case PAYLOAD:
				estructura->payloadRx[estructura->payloadRxLength++] = byteActual;
				checksum ^= byteActual;
				nBytes--;
				if (nBytes == 0) {
					estado = CHECKSUM;
				}
				break;
			case CHECKSUM:
				if(checksum == byteActual)
					estructura->Rxir = estructura->Rxiw;
				estado = START;
				break;
			default:
				estado = START;
				break;
		}
	}
}



void encodeProtocol(_sRingBuffer *buffer, uint8_t length, short eID) {
    uint8_t checksum = 0;
    uint8_t index = buffer->Txiw;

    buffer->bufferTx[index++] = START_BYTE;
    buffer->bufferTx[index++] = HEADER_1_BYTE;
    buffer->bufferTx[index++] = HEADER_2_BYTE;
    buffer->bufferTx[index++] = HEADER_3_BYTE;
    buffer->bufferTx[index++] = length + 1;
    buffer->bufferTx[index++] = TOKEN_BYTE;
    buffer->bufferTx[index++] = eID;

    if(length > 0){
		for (uint8_t i = 0; i < length; ++i) {
			buffer->bufferTx[index++] = buffer->payloadTx[i];
		}
    }
    for (uint8_t i = buffer->Txiw; i < index; ++i) {
        checksum ^= buffer->bufferTx[i];
    }

    buffer->bufferTx[index++] = checksum;
    buffer->Txiw = index;
}
