/*
 *  serial_frame.h
 *
 *  Created on: May 3, 2025
 *      Author: pboon
 */

#ifndef INC_SERIAL_FRAME_H_
#define INC_SERIAL_FRAME_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>

#define SERIAL_FRAME_MAX_FIELDS 32
#define SERIAL_FRAME_MAX_SIZE 255

typedef enum {
    SERIAL_TYPE_UINT8 = 0,
    SERIAL_TYPE_INT8,
    SERIAL_TYPE_UINT16,
    SERIAL_TYPE_INT16,
    SERIAL_TYPE_UINT32,
    SERIAL_TYPE_INT32,
    SERIAL_TYPE_FLOAT,
    SERIAL_TYPE_DOUBLE
} SerialFrameDataType;

typedef struct {
    SerialFrameDataType type;
    void *data_ptr;
    uint8_t position;
    uint8_t size;
    const char *name;
} SerialFrameField;

typedef struct {
    SerialFrameField tx_fields[SERIAL_FRAME_MAX_FIELDS];
    SerialFrameField rx_fields[SERIAL_FRAME_MAX_FIELDS];
    uint8_t tx_field_count;
    uint8_t rx_field_count;
    uint8_t tx_frame_size;
    uint8_t rx_frame_size;
    uint8_t tx_buffer[SERIAL_FRAME_MAX_SIZE];
    uint8_t rx_buffer[SERIAL_FRAME_MAX_SIZE];
    UART_HandleTypeDef *huart;
    uint8_t header;
    uint8_t terminator;
} SerialFrame;

void SerialFrame_Init(SerialFrame *frame, UART_HandleTypeDef *huart, uint8_t header, uint8_t terminator);

int SerialFrame_AddTxField(SerialFrame *frame, SerialFrameDataType type, void *data_ptr, const char *name);
int SerialFrame_AddRxField(SerialFrame *frame, SerialFrameDataType type, void *data_ptr, const char *name);
void SerialFrame_RemoveAllFields(SerialFrame *frame);

void SerialFrame_BuildTxFrame(SerialFrame *frame);
void SerialFrame_ParseRxFrame(SerialFrame *frame);
void SerialFrame_Transmit(SerialFrame *frame);
void SerialFrame_StartReceive(SerialFrame *frame);

uint8_t SerialFrame_GetDataSize(SerialFrameDataType type);

#endif /* INC_SERIAL_FRAME_H_ */
