/*
 * ModBusRTU.h
 *
 *  Created on: Feb 28, 2023
 *      Author: AlphaP-Tuf
 */

#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_
#include "stm32g4xx_hal.h"
#include "string.h"
#define MODBUS_MESSAGEBUFFER_SIZE 300
typedef union
{
	uint16_t U16;
	uint8_t U8[2];
}u16u8_t;
typedef enum _ModbusState
{
	Modbus_state_Init,
	Modbus_state_Idle,
	Modbus_state_Emission,
	Modbus_state_Reception,
	Modbus_state_ControlAndWaiting
}ModbusStateTypedef;

typedef enum _modbusFunctioncode
{

	Modbus_function_Read_Coil =1,
	Modbus_function_Read_DiscreteInput,
	Modbus_function_Read_Holding_Register,
	Modbus_function_Read_Input_Registor,
	Modbus_function_Write_SingleCoil,
	Modbus_function_Write_SingleRegister,
	Modbus_function_Diagnostics =8,
	Modbus_function_GetCommEventCounter = 11,
	Modbus_function_Write_MultipleCoil =15,
	Modbus_function_Write_MultipleRegistor

}ModbusFunctionCode;

typedef enum _modbusRecvFrameStatus
{
	Modbus_RecvFrame_Null = -2,
	Modbus_RecvFrame_FrameError = -1,
	Modbus_RecvFrame_Normal = 0,
	Modbus_RecvFrame_IllegalFunction,
	Modbus_RecvFrame_IllegalDataAddress,
	Modbus_RecvFrame_IllegalDataValue,
	Modbus_RecvFrame_SlaveDeviceFailure,
	Modbus_RecvFrame_Acknowlage,
	Modbus_RecvFrame_SlaveDeviceBusy,
	Modbus_RecvFrame_NegativeAcknowage,
	Modbus_RecvFrame_MemoryParityError,
	Modbus_RecvFrame_GatewayTargetDeviceFailedToRespon
}modbusRecvFrameStatus;

//Modbus Handle structure
typedef struct _ModbusHandleTypedef
{
	uint8_t slaveAddress;

	//Register
	u16u8_t *RegisterAddress;
	uint32_t RegisterSize;


	UART_HandleTypeDef* huart; // 19200 8E1 , Enable Interrupt ,Register callback Enable

	TIM_HandleTypeDef* htim; // timer period = 2t ,Enable ONE pulse mode , Enable Interrupt ,Register callback Enable

	//flag
	uint8_t Flag_T15TimeOut;
	uint8_t Flag_T35TimeOut;


	modbusRecvFrameStatus RecvStatus;


	ModbusStateTypedef Mstatus; //Modbus state (for state machine)

	//PDU frame
	uint8_t Rxframe[MODBUS_MESSAGEBUFFER_SIZE];
	uint8_t Txframe[MODBUS_MESSAGEBUFFER_SIZE];
	uint8_t TxCount;
	//Serial frame
	struct _modbusUartStructure
	{
	uint8_t MessageBufferRx[MODBUS_MESSAGEBUFFER_SIZE+3];
	uint16_t RxTail;
	uint8_t MessageBufferTx[MODBUS_MESSAGEBUFFER_SIZE+3];
	uint16_t TxTail;
	} modbusUartStructure;


}ModbusHandleTypedef;

void Modbus_init(ModbusHandleTypedef* hmodbus,UART_HandleTypeDef* huart,TIM_HandleTypeDef* htim ,u16u8_t* RegisterStartAddress,uint8_t slaveAddress,uint32_t RegisterSize);

void Modbus_Protocal_Worker();

#endif /* INC_MODBUSRTU_H_ */
