/*
 *  serial_frame.c
 *
 *  Created on: May 3, 2025
 *      Author: pboon
 */

#include "serial_frame.h"
#include <stdlib.h>

/* Initialize serial frame */
void SerialFrame_Init(SerialFrame *frame, UART_HandleTypeDef *huart, uint8_t header, uint8_t terminator) {
    frame->huart = huart;
    frame->tx_field_count = 0;
    frame->rx_field_count = 0;
    frame->tx_frame_size = 2; // Header + Terminator
    frame->rx_frame_size = 2; // Header + Terminator
    frame->header = header;
    frame->terminator = terminator;

    memset(frame->tx_buffer, 0, SERIAL_FRAME_MAX_SIZE);
    memset(frame->rx_buffer, 0, SERIAL_FRAME_MAX_SIZE);
}

/* Get size of data type */
uint8_t SerialFrame_GetDataSize(SerialFrameDataType type) {
    switch (type) {
        case SERIAL_TYPE_UINT8:
        case SERIAL_TYPE_INT8:
            return 1;
        case SERIAL_TYPE_UINT16:
        case SERIAL_TYPE_INT16:
            return 2;
        case SERIAL_TYPE_UINT32:
        case SERIAL_TYPE_INT32:
        case SERIAL_TYPE_FLOAT:
            return 4;
        case SERIAL_TYPE_DOUBLE:
            return 8;
        default:
            return 0;
    }
}

/* Add transmit field */
int SerialFrame_AddTxField(SerialFrame *frame, SerialFrameDataType type, void *data_ptr, const char *name) {
    if (frame->tx_field_count >= SERIAL_FRAME_MAX_FIELDS) {
        return -1;
    }

    SerialFrameField *field = &frame->tx_fields[frame->tx_field_count];
    field->type = type;
    field->data_ptr = data_ptr;
    field->name = name;
    field->size = SerialFrame_GetDataSize(type);
    field->position = frame->tx_frame_size - 1; // Before terminator

    frame->tx_frame_size += field->size;
    frame->tx_field_count++;

    // Update other field positions
    for (int i = 0; i < frame->tx_field_count; i++) {
        if (i == frame->tx_field_count - 1) continue;
        frame->tx_fields[i].position = i == 0 ? 1 :
            (frame->tx_fields[i-1].position + frame->tx_fields[i-1].size);
    }

    return 0;
}

/* Add receive field */
int SerialFrame_AddRxField(SerialFrame *frame, SerialFrameDataType type, void *data_ptr, const char *name) {
    if (frame->rx_field_count >= SERIAL_FRAME_MAX_FIELDS) {
        return -1;
    }

    SerialFrameField *field = &frame->rx_fields[frame->rx_field_count];
    field->type = type;
    field->data_ptr = data_ptr;
    field->name = name;
    field->size = SerialFrame_GetDataSize(type);
    field->position = frame->rx_frame_size - 1; // Before terminator

    frame->rx_frame_size += field->size;
    frame->rx_field_count++;

    // Update other field positions
    for (int i = 0; i < frame->rx_field_count; i++) {
        if (i == frame->rx_field_count - 1) continue;
        frame->rx_fields[i].position = i == 0 ? 1 :
            (frame->rx_fields[i-1].position + frame->rx_fields[i-1].size);
    }

    return 0;
}

/* Remove all fields */
void SerialFrame_RemoveAllFields(SerialFrame *frame) {
    frame->tx_field_count = 0;
    frame->rx_field_count = 0;
    frame->tx_frame_size = 2;
    frame->rx_frame_size = 2;
    memset(frame->tx_fields, 0, sizeof(frame->tx_fields));
    memset(frame->rx_fields, 0, sizeof(frame->rx_fields));
}

/* Build transmit frame */
void SerialFrame_BuildTxFrame(SerialFrame *frame) {
    frame->tx_buffer[0] = frame->header;

    for (int i = 0; i < frame->tx_field_count; i++) {
        SerialFrameField *field = &frame->tx_fields[i];
        memcpy(&frame->tx_buffer[field->position], field->data_ptr, field->size);
    }

    frame->tx_buffer[frame->tx_frame_size - 1] = frame->terminator;
}

/* Parse receive frame */
void SerialFrame_ParseRxFrame(SerialFrame *frame) {
    if (frame->rx_buffer[0] != frame->header) {
        return; // Invalid frame
    }

    for (int i = 0; i < frame->rx_field_count; i++) {
        SerialFrameField *field = &frame->rx_fields[i];
        memcpy(field->data_ptr, &frame->rx_buffer[field->position], field->size);
    }
}

/* Transmit frame */
void SerialFrame_Transmit(SerialFrame *frame) {
    HAL_UART_Transmit_DMA(frame->huart, frame->tx_buffer, frame->tx_frame_size);
}

/* Start receive */
void SerialFrame_StartReceive(SerialFrame *frame) {
    HAL_UART_Receive_DMA(frame->huart, frame->rx_buffer, frame->rx_frame_size);
}
