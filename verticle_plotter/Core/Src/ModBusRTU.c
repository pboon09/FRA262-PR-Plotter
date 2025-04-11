/*
 * ModBusRTU.c
 *
 *  Created on: Feb 28, 2023
 *      Author: AlphaP-Tuf
 */
#include "ModBusRTU.h"


#define MODBUS_UART_BUFFER_SIZE 300

ModbusHandleTypedef* hModbus;

void modbus_1t5_Timeout();
void modbus_3t5_Timeout(TIM_HandleTypeDef *htim);
void Modbus_Emission();
void modbusWrite1Register(); //function 06
void modbusRead1Register(); // function 03
void ModbusErrorReply(uint8_t);
void Modbus_frame_response();
void modbus_ErrorTimeout(UART_HandleTypeDef* huart);

// function for interrupt
void modbus_1t5_Timeout()
{
	//end of package flag set
	hModbus->Flag_T15TimeOut = 1;
	//set 3t5 Timer
	__HAL_TIM_SET_COUNTER(hModbus->htim,0);
	__HAL_TIM_ENABLE(hModbus->htim);
}

void modbus_3t5_Timeout(TIM_HandleTypeDef *htim)
{
	//return package flag set
	hModbus->Flag_T35TimeOut = 1;

}

void modbus_ErrorTimeout(UART_HandleTypeDef* huart)
{
	if(HAL_UART_GetError(huart)==HAL_UART_ERROR_RTO)
	{
		modbus_1t5_Timeout();

	}
}



void Modbus_init(ModbusHandleTypedef* hmodbus,UART_HandleTypeDef* huart,TIM_HandleTypeDef* htim ,u16u8_t* RegisterStartAddress,uint8_t slaveAddress,uint32_t RegisterSize)
{
	hModbus = hmodbus;

	hModbus->huart = huart;
	hModbus->htim = htim;
	hModbus->RegisterAddress = RegisterStartAddress;
	hModbus->slaveAddress = slaveAddress;
	hModbus->RegisterSize = RegisterSize;

	//config timer interrupt

	HAL_TIM_RegisterCallback(hmodbus->htim,HAL_TIM_PERIOD_ELAPSED_CB_ID ,(void*)modbus_3t5_Timeout);

	//config UART interrupt
	HAL_UART_ReceiverTimeout_Config(hmodbus->huart, 16);
	HAL_UART_EnableReceiverTimeout(hmodbus->huart);

	HAL_UART_RegisterCallback(hmodbus->huart, HAL_UART_ERROR_CB_ID, (void*)modbus_ErrorTimeout);
	//HAL_UART_RegisterCallback(hmodbus->huart,HAL_UART_RX_COMPLETE_CB_ID,(void*)modbus_UART_Recived);
	//start Receive
    HAL_UART_Receive_DMA(hModbus->huart,
    		&(hModbus->modbusUartStructure.MessageBufferRx[hModbus->modbusUartStructure.RxTail]),
			MODBUS_UART_BUFFER_SIZE );


    if(hModbus->htim->State == HAL_TIM_STATE_READY)
    	{
    		HAL_TIM_Base_Start_IT(hModbus->htim);
    		HAL_TIM_OnePulse_Start_IT(hModbus->htim, TIM_CHANNEL_1);
    	}

}
/* Table of CRC values for low–order byte */
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

unsigned short CRC16 ( puchMsg, usDataLen ) /* The function returns the CRC as a unsigned short type */
unsigned char *puchMsg ; /* message to calculate CRC upon */
unsigned short usDataLen ; /* quantity of bytes in message */
{
	unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	unsigned uIndex ; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCLo ^ *puchMsg++ ; /* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
		uchCRCHi = auchCRCLo[uIndex] ;
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}



void Modbus_Protocal_Worker()
{
	switch(hModbus->Mstatus)
	{
	default:
	case Modbus_state_Init:
		/*init Modbus protocal*/

		hModbus->Mstatus = Modbus_state_Idle;
		break;
	case Modbus_state_Idle:
		/*Idle state*/

		//check that we have response message
		if(hModbus->TxCount)
		{
			/*set state*/
			hModbus->Mstatus= Modbus_state_Emission;
			Modbus_Emission();
		}

		// Received character
		else if(hModbus->huart->RxXferSize!=hModbus->huart->RxXferCount)
		{
			/*reset Timer flag*/
			hModbus->Flag_T15TimeOut = 0;
			hModbus->Flag_T35TimeOut = 0;
			/*set state*/
			hModbus->Mstatus= Modbus_state_Reception;
		}

		//check that if UART RX not start, start receiving
		if(hModbus->huart->RxState == HAL_UART_STATE_READY)
		{
			hModbus->modbusUartStructure.RxTail =0;
		    HAL_UART_Receive_DMA(hModbus->huart,
		    		&(hModbus->modbusUartStructure.MessageBufferRx[hModbus->modbusUartStructure.RxTail]),
					MODBUS_UART_BUFFER_SIZE );


		}
		break;
	case Modbus_state_Reception:

		if(hModbus->Flag_T15TimeOut)
		{
			/*reset recived flag*/
			hModbus->RecvStatus = Modbus_RecvFrame_Null;
			hModbus->modbusUartStructure.RxTail =
					hModbus->huart->RxXferSize - __HAL_DMA_GET_COUNTER(hModbus->huart->hdmarx); 			/*compute CRC and Slave address*/



			hModbus->Mstatus = Modbus_state_ControlAndWaiting;
		}
		break;
	case Modbus_state_ControlAndWaiting:

		/*Frame Calculation , calculate once*/
		if(hModbus->RecvStatus == Modbus_RecvFrame_Null)
		{
			hModbus->RecvStatus = Modbus_RecvFrame_Normal;
			// check CRC
			u16u8_t CalculateCRC;
			CalculateCRC.U16 = CRC16(hModbus->modbusUartStructure.MessageBufferRx,hModbus->modbusUartStructure.RxTail - 2);

			if(!(CalculateCRC.U8[0] == hModbus->modbusUartStructure.MessageBufferRx[hModbus->modbusUartStructure.RxTail - 2]
			&& CalculateCRC.U8[1] == hModbus->modbusUartStructure.MessageBufferRx[hModbus->modbusUartStructure.RxTail -1]))
			{
				// communication unsuccessful
				hModbus->RecvStatus = Modbus_RecvFrame_FrameError;
				break;
			}

			//check Slave Address
			if(hModbus->modbusUartStructure.MessageBufferRx[0] != hModbus->slaveAddress)
				break;

			//copy recivced frame
			memcpy(hModbus->Rxframe,
					hModbus->modbusUartStructure.MessageBufferRx+1,
					hModbus->modbusUartStructure.RxTail-3);

			//execute command
			Modbus_frame_response();

		}

		if(hModbus->Flag_T35TimeOut)
		{
			hModbus->Mstatus = Modbus_state_Idle;

		}
		break;

	case Modbus_state_Emission:
		if(hModbus->huart->gState==HAL_UART_STATE_READY)
					{
			hModbus->TxCount=0;
			hModbus->Mstatus = Modbus_state_Idle;
					}
		break;


	}
}
void modbusWrite1Register() //function 06
{

	//write data to register
	uint16_t startAddress = (hModbus->Rxframe[1]<<8)+(hModbus->Rxframe[2]);

	if(startAddress > hModbus->RegisterSize)
		{
			 ModbusErrorReply(Modbus_RecvFrame_IllegalDataAddress);
			 return;
		}


	hModbus->RegisterAddress[startAddress].U8[1] = hModbus->Rxframe[3];
	hModbus->RegisterAddress[startAddress].U8[0] = hModbus->Rxframe[4];



	//generate response
	memcpy(hModbus->Txframe,
			hModbus->Rxframe,
			8);
	//set number of byte to sent
	hModbus->TxCount=5;



}

void modbusRead1Register() // function 03
{



	uint16_t numberOfDataToRead =((hModbus->Rxframe[3]<<8)+(hModbus->Rxframe[4]));
	uint16_t startAddress =((hModbus->Rxframe[1]<<8)+(hModbus->Rxframe[2]));

	//check quantity and address range

	if(numberOfDataToRead <1 ||numberOfDataToRead > 0x7D)
	{
		 ModbusErrorReply(Modbus_RecvFrame_IllegalDataValue);
		 return;
	}

	if(startAddress > hModbus->RegisterSize || (startAddress +  numberOfDataToRead) > hModbus->RegisterSize)
	{
		 ModbusErrorReply(Modbus_RecvFrame_IllegalDataAddress);
		 return;
	}


	//generate response
	hModbus->Txframe[0] = Modbus_function_Read_Holding_Register;
	hModbus->Txframe[1] = (2*numberOfDataToRead) & 0xFF;
	register int i;
	for(i=0; i<numberOfDataToRead;i++)
	{
		hModbus->Txframe[2*i+2]=hModbus->RegisterAddress[startAddress+i].U8[1];
		hModbus->Txframe[2*i+3]=hModbus->RegisterAddress[startAddress+i].U8[0];
	}
	hModbus->TxCount = 2+2*numberOfDataToRead;

}

void ModbusErrorReply(uint8_t Errorcode)
{
	hModbus->Txframe[0] = hModbus->Rxframe[0] | 0x80;
	hModbus->Txframe[1] = Errorcode;
	hModbus->TxCount = 2;
}

void Modbus_frame_response()
{
	switch(hModbus->Rxframe[0]) //check funcion
	{
	case Modbus_function_Write_SingleRegister:
		modbusWrite1Register();
		break;
	case Modbus_function_Read_Holding_Register:
		modbusRead1Register();
		break;
	default:
		 ModbusErrorReply(Modbus_RecvFrame_IllegalFunction);
		break;

	}
}

void Modbus_Emission()
{
	if(hModbus->huart->gState==HAL_UART_STATE_READY)
	{
		//generate response package
		hModbus->modbusUartStructure.MessageBufferTx[0] = hModbus->slaveAddress;

		memcpy
		(
				hModbus->modbusUartStructure.MessageBufferTx+1,
				hModbus->Txframe,
				hModbus->TxCount
		);

		hModbus->modbusUartStructure.TxTail = hModbus->TxCount+3;

		u16u8_t CalculateCRC;
		CalculateCRC.U16 = CRC16(hModbus->modbusUartStructure.MessageBufferTx,
				hModbus->modbusUartStructure.TxTail - 2);

		hModbus->modbusUartStructure.MessageBufferTx[hModbus->modbusUartStructure.TxTail-2]
													 =CalculateCRC.U8[0];

		hModbus->modbusUartStructure.MessageBufferTx[hModbus->modbusUartStructure.TxTail-1]
													 =CalculateCRC.U8[1];


		//sent modbus

		if(hModbus->huart->gState==HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_DMA(hModbus->huart
					,hModbus->modbusUartStructure.MessageBufferTx
					,hModbus->modbusUartStructure.TxTail);
		}




	}
	/*reset Timer flag*/
	hModbus->Flag_T15TimeOut = 0;
	hModbus->Flag_T35TimeOut = 0;

}

