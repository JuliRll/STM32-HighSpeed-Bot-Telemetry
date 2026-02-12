/*
 * protocol_handler.h
 *
 *  Created on: Jan 22, 2025
 *      Author: jrlle
 */

/*
 * Librería:
 * decodifica y codifica con un protocolo estándar que se puede redefinir.
 *
 * Se considera un buffer circular con sobreescritura solamente en el buffer de lectura.
 * El buffer de transmisión no contiene desplazamiento de buffer.
 */

#ifndef INC_PROTOCOL_HANDLER_H_
#define INC_PROTOCOL_HANDLER_H_
#include <stdint.h>
#include <stdbool.h>
/* Tamaño máximo del payload */
#define MAX_BUFFER_SIZE 256
#define MAX_PAYLOAD_SIZE 256
/* Definición del protocolo */
#define START_BYTE 'U'
#define HEADER_1_BYTE 'N'
#define HEADER_2_BYTE 'E'
#define HEADER_3_BYTE 'R'
#define TOKEN_BYTE ':'

/* Estructura para manejar datos */
typedef struct {
	uint8_t bufferRx[MAX_BUFFER_SIZE];
	uint8_t bufferTx[MAX_BUFFER_SIZE];
	uint8_t payloadRx[MAX_PAYLOAD_SIZE];
	uint8_t payloadTx[MAX_PAYLOAD_SIZE];
	uint8_t Rxir;
	uint8_t Rxiw;
	uint8_t Txir;
	uint8_t Txiw;
	uint8_t payloadRxLength;
	uint8_t payloadTxLength;
	uint8_t id;
} _sRingBuffer;


/* Prototipos de protocolo */
void decodeProtocol(_sRingBuffer *estructura);
void encodeProtocol(_sRingBuffer *estructura, uint8_t length, short eID);


#endif /* INC_PROTOCOL_HANDLER_H_ */
