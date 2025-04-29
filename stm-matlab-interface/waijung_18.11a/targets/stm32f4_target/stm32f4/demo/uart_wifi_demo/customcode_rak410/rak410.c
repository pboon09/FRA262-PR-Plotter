
#include <string.h>
#include <stdlib.h>

/* 
** We want to use Waijung resource
*/
#include "waijung_hwdrvlib.h"

/* ===============Configuration=========================================
*/
#define RAK410_INITIAL_STRING 	"Welcome to RAK410\r\n"
#define RAK410_NETWORK_SCAN 	"at+scan=0,ZTE Wireless Network\r\n"
//#define RAK410_NETWORK_PSK		"at+psk=???\r\n"
#define RAK410_NETWORK_CONNECT	"at+connect=ZTE Wireless Network\r\n"
#define RAK410_PWRMODE			"at+pwrmode=0\r\n"

/* UART Configuration requirement:
**  Frame: 115200 baud, 8 bit, 1 STOP
**  Flow control: CTS/RTS
*/

/* WiFi Reset pin control: PB0 */
#define WIFI_RESET_PORT GPIOB
#define WIFI_RESET_PIN  GPIO_Pin_0
#define WIFI_RESET_CLK  RCC_AHB1Periph_GPIOB

/* Define UARTx for communicate with Rak410 */
#define RAK410_UART_MODULE 		2

#if RAK410_UART_MODULE == 1
	#if !defined(UTX1_BUFFER_SIZE) || !defined(URX1_BUFFER_SIZE)
		#error "Invalud UART Setup."
	#endif
	#define RAK410_TX_BUFFER_SIZE UTX1_BUFFER_SIZE
	#define RAK410_RX_BUFFER_SIZE URX1_BUFFER_SIZE
	#define RAK410_UART_Write UART1_Write
	#define RAK410_UART_TxFlush UART1_FlushTxBuffer
	#define RAK410_UART_Read UART1_ReadEx
#elif RAK410_UART_MODULE == 2
	#if !defined(UTX2_BUFFER_SIZE) || !defined(URX2_BUFFER_SIZE)
		#error "Invalud UART Setup."
	#endif
	#define RAK410_TX_BUFFER_SIZE UTX2_BUFFER_SIZE
	#define RAK410_RX_BUFFER_SIZE URX2_BUFFER_SIZE
	#define RAK410_UART_Write UART2_Write
	#define RAK410_UART_TxFlush UART2_FlushTxBuffer
	#define RAK410_UART_Read UART2_ReadEx
#elif RAK410_UART_MODULE == 3
	#if !defined(UTX3_BUFFER_SIZE) || !defined(URX3_BUFFER_SIZE)
		#error "Invalud UART Setup."
	#endif
	#define RAK410_TX_BUFFER_SIZE UTX3_BUFFER_SIZE
	#define RAK410_RX_BUFFER_SIZE URX3_BUFFER_SIZE
	#define RAK410_UART_Write UART3_Write
	#define RAK410_UART_TxFlush UART3_FlushTxBuffer
	#define RAK410_UART_Read UART3_ReadEx
#elif RAK410_UART_MODULE == 6
	#if !defined(UTX6_BUFFER_SIZE) || !defined(URX6_BUFFER_SIZE)
		#error "Invalud UART Setup."
	#endif	
	#define RAK410_TX_BUFFER_SIZE UTX6_BUFFER_SIZE
	#define RAK410_RX_BUFFER_SIZE URX6_BUFFER_SIZE
	#define RAK410_UART_Write UART6_Write
	#define RAK410_UART_TxFlush UART6_FlushTxBuffer
	#define RAK410_UART_Read UART6_ReadEx
#else
  #error "Invalid UART Setup module."
#endif

/* ===================================================================
*/
/* UART Tx interface */
void RAK410_Write(const char *buffer, uint16_t buffer_size) {
	RAK410_UART_Write((uint8_t*)buffer, buffer_size);
	RAK410_UART_TxFlush();
}

/* UART Rx interface */
static UARTRX_BUFFER_READ_STRUCT rak410_read_struct;
void RAK410_Read(char *buffer, uint16_t buffer_size, uint16_t *reading_count)
{
	RAK410_UART_Read(&rak410_read_struct, (uint8_t *)buffer, buffer_size, reading_count);
}
/* Return 0: Success, otherwise: Fail */
uint16_t RAK410_ReadLine(char *buffer, uint16_t buffer_size, uint32_t timeout, uint16_t *ret_count)
{
	SYS_TIMER_uS_STRUCT timer;
	uint16_t reading_index, reading_count;	
	
	/* Start timer */
	SysTimer_uS_Start(&timer, timeout);
	
	/* Read bytes */
	reading_index = 0;
	do {
		RAK410_Read(&buffer[reading_index], buffer_size-reading_index-1, &reading_count);
		
		reading_index += reading_count;
		buffer[reading_index] = '\0'; /* NULL terminator */	
		
		/* Reset time-out */
		if(reading_count > 0)
			SysTimer_uS_Start(&timer, timeout);

		/* Check line end: "\r\n" */
		if(reading_index >= 2) {
			if(!strcmp(&buffer[reading_index-2], "\r\n")) {
				/* Done */
				*ret_count= reading_index;
				return 0; /* Success, with data */
			}
		}
		/* Return number of reading */
		*ret_count= reading_index;
		/* Check time-out */
		if(SysTimer_uS_IsTimeout(&timer))
			return 0xFFFF; /* Error: Timeout, with some data */
	} while(reading_index > 0);
	
	/* No byte received, ret_count will return 0 */
	return 0; /* Success, without any data */
}

typedef enum WiFi_Setup_State {
	WIFI_READY = 0,
	WIFI_RESET,
	WIFI_SCAN,
#ifdef RAK410_NETWORK_PSK
	WIFI_PSK,
#endif	
	WIFI_CONNECT,
	WIFI_PWRMODE,
	WIFI_IP_SETUP,
	WIFI_TIMEOUT,
	WIFI_INVALID_RESP,
} WIFI_SETUP_STATE;
static uint8_t WiFi_Reset_index = 0;
static WIFI_SETUP_STATE WiFi_Setup_State = WIFI_RESET;
static SYS_TIMER_uS_STRUCT WiFi_Timer_uS; /* For non-blocking delay */
char Rak410_read_buffer[RAK410_RX_BUFFER_SIZE];
char Rak410_write_buffer[RAK410_TX_BUFFER_SIZE];

/* ================================================================
** RAK410 Setup
*/
void enable_rak410_setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Init WiFi Reset pin */
	RCC_AHB1PeriphClockCmd(WIFI_RESET_CLK, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = WIFI_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(WIFI_RESET_PORT, &GPIO_InitStructure);
  
	/* Init read Struct */
	memset(&rak410_read_struct, 0, sizeof(rak410_read_struct));
	rak410_read_struct.index = 0;//RAK410_RX_BUFFER_SIZE;

	/* State */
	WiFi_Setup_State = WIFI_RESET;
}

static uint8_t rak410_reset_activate = 0;
void output_rak410_setup(uint8_t dhcp,uint32_t owner_ip,uint32_t gateway,uint8_t *status)
{
	static WIFI_SETUP_STATE last_state = WIFI_READY;
	uint16_t readline_sta;
	uint16_t reading_count;
	uint8_t on_enterstate = 0;
	
	
	/* Check reset activation */
	if((rak410_reset_activate != 0) && (WiFi_Setup_State == WIFI_READY)) {
		/* Activate reset state */
		WiFi_Reset_index = 0;
		WiFi_Setup_State = WIFI_RESET;
		
		/* Clear reset activate flag */
		rak410_reset_activate = 0;
	}
	
	/* Check if state changed, to activate OnEnter state event */
	if(last_state != WiFi_Setup_State) {
		last_state = WiFi_Setup_State;
		on_enterstate = 1; /* First time enter to the State */
	}
	else { on_enterstate = 0; }
	
	/* WiFi state */
	switch(WiFi_Setup_State) {
		/* --- Reset --- */
		case WIFI_RESET:
			if(WiFi_Reset_index == 0) { /* Reset pin Low */
				/* Set WiFi RESET pin to Low */
				GPIO_ResetBits(WIFI_RESET_PORT, WIFI_RESET_PIN);
				/* Start non-blocking timer, for pin low duration */
				SysTimer_uS_Start(&WiFi_Timer_uS, 2000000); /* Low 2Sec: must be long enough for disconnect from WiFi AP */
				/* Next Reset state */
				WiFi_Reset_index++;
			}
			else if(WiFi_Reset_index == 1) { /* Reset pin High */
				/* Wait for time-out from previous reset state */
				if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS)) {
					/* Flush UART input */
					do { RAK410_Read(Rak410_read_buffer, sizeof(Rak410_read_buffer)-1, &reading_count); }
					while(reading_count != 0);					
					/* Set WiFi RESET pin to High */
					GPIO_SetBits(WIFI_RESET_PORT, WIFI_RESET_PIN);
					/* Start non-blocking timer, for pin Hi duration */
					SysTimer_uS_Start(&WiFi_Timer_uS, 1000000); /* High 1Second (Wait for Wifi ready) */
					/* Next Reset state */
					WiFi_Reset_index++;
				}
			}
			else if(WiFi_Reset_index == 2) { /* Get message: "Welcome to RAK410\r\n" */
				/* Wait for time-out from previous reset state */
				if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS)) {
					/* Welcome message should be ready:  */
					RAK410_Read(Rak410_read_buffer, sizeof(Rak410_read_buffer), &reading_count);
					Rak410_read_buffer[reading_count] = '\0'; /* NULL terminator */
					if(strstr(Rak410_read_buffer, RAK410_INITIAL_STRING)) {
						/* Success, ==> Change state */
						WiFi_Setup_State = WIFI_SCAN;
					}
					else { /* Fail */
						/* Wait 1 second before re-try */
						SysTimer_uS_Start(&WiFi_Timer_uS, 1000000); /* 1 Second */
						/* Next Reset state */
						WiFi_Reset_index++;
					}
				}
			}
			else if(WiFi_Reset_index == 3) {
				if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS)) {
					WiFi_Reset_index = 0; /* Retry Reset process */
				}
			}
			break;
			
		/* --- Scan network --- */
		case WIFI_SCAN:
			if(on_enterstate) {
				/* Write command string to Wifi */
				RAK410_Write(RAK410_NETWORK_SCAN, (uint16_t)strlen(RAK410_NETWORK_SCAN));
				/* Scan process allow 10 Second (Maximum) */
				SysTimer_uS_Start(&WiFi_Timer_uS, 10000000); /* 10 Second */				
			}
			else {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
				if((readline_sta == 0) && (reading_count > 0)) {
					/* Success */
					if(strstr(Rak410_read_buffer, "OK")) {
						#ifdef RAK410_NETWORK_PSK
						WiFi_Setup_State = WIFI_PSK;
						#else
						WiFi_Setup_State = WIFI_CONNECT;
						#endif
					}
					else {
						/* Invalid response */
						WiFi_Setup_State = WIFI_INVALID_RESP;
					}
				}
				else { /* readline_sta != 0 */
					if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS) || (reading_count > 0))
						WiFi_Setup_State = WIFI_TIMEOUT; /* Time-out, ==> Change state */
				}
			}
			break;
			
#ifdef RAK410_NETWORK_PSK			
		/* --- Set password --- */
		case WIFI_PSK:
			if(on_enterstate) {
				/* Write command string to Wifi */
				RAK410_Write(RAK410_NETWORK_PSK, (uint16_t)strlen(RAK410_NETWORK_PSK));
				/* Scan process allow 10 Second (Maximum) */
				SysTimer_uS_Start(&WiFi_Timer_uS, 10000000); /* 10 Second */				
			}
			else {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
				if((readline_sta == 0) && (reading_count > 0)) {
					/* Success */
					if(!strncmp(Rak410_read_buffer, "OK", 2))
						WiFi_Setup_State = WIFI_CONNECT;
					else /* Invalid response */
						WiFi_Setup_State = WIFI_INVALID_RESP;
				}
				else { /* readline_sta != 0 */
					if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS) || (reading_count > 0))
						WiFi_Setup_State = WIFI_TIMEOUT; /* Time-out, ==> Change state */
				}
			}
			break;		
#endif				
			
		/* --- Connect to network --- */
		case WIFI_CONNECT:
			if(on_enterstate) {
				/* Write command string to Wifi */
				RAK410_Write(RAK410_NETWORK_CONNECT, (uint16_t)strlen(RAK410_NETWORK_CONNECT));
				/* Scan process allow 10 Second (Maximum) */
				SysTimer_uS_Start(&WiFi_Timer_uS, 10000000); /* 10 Second */				
			}
			else {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
				if((readline_sta == 0) && (reading_count > 0)) {					
					if(!strncmp(Rak410_read_buffer, "OK", 2)) /* Success */
						WiFi_Setup_State = WIFI_PWRMODE;
					else /* Invalid response */
						WiFi_Setup_State = WIFI_INVALID_RESP;
				}
				else { /* readline_sta != 0 */
					if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS) || (reading_count > 0))
						WiFi_Setup_State = WIFI_TIMEOUT; /* Time-out, ==> Change state */
				}
			}
			break;
			
		/* --- Set Power Mode --- */
		case WIFI_PWRMODE:
			if(on_enterstate) {
				/* Write command string to Wifi */
				RAK410_Write(RAK410_PWRMODE, (uint16_t)strlen(RAK410_PWRMODE));
				/* Scan process allow 10 Second (Maximum) */
				SysTimer_uS_Start(&WiFi_Timer_uS, 10000000); /* 10 Second */				
			}
			else {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
				if((readline_sta == 0) && (reading_count > 0)) {					
					if(!strncmp(Rak410_read_buffer, "OK", 2)) /* Success */
						WiFi_Setup_State = WIFI_IP_SETUP;
					else /* Invalid response */
						WiFi_Setup_State = WIFI_INVALID_RESP;
				}
				else { /* readline_sta != 0 */
					if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS) || (reading_count > 0))
						WiFi_Setup_State = WIFI_TIMEOUT; /* Time-out, ==> Change state */
				}
			}		
			break;
			
		/* --- Connect UDP --- */
		case WIFI_IP_SETUP:
			if(on_enterstate) {
				/* Configure module IP as DHCP */
				if(dhcp) {
					strcpy(Rak410_write_buffer, "at+ipdhcp=0\r\n");
					RAK410_Write(Rak410_write_buffer, (uint16_t)strlen(Rak410_write_buffer));
				}
				/* Configure module IP as static IP */
				else {
					sprintf(Rak410_write_buffer, "at+ipstatic=%u.%u.%u.%u,255.255.255.0,%u.%u.%u.%u,0,0\r\n",
						(owner_ip>>24)&0xFF, (owner_ip>>16)&0xFF, (owner_ip>>8)&0xFF, (owner_ip>>0)&0xFF,
						(gateway>>24)&0xFF, (gateway>>16)&0xFF, (gateway>>8)&0xFF, (gateway>>0)&0xFF);
					RAK410_Write(Rak410_write_buffer, (uint16_t)strlen(Rak410_write_buffer));
				}
				/* Timeout check setup */
				SysTimer_uS_Start(&WiFi_Timer_uS, 10000000); /* 10 Second */			
			}
			else {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
				if((readline_sta == 0) && (reading_count > 0)) {					
					if(!strncmp(Rak410_read_buffer, "OK", 2)) /* Success */
						WiFi_Setup_State = WIFI_READY;
					else /* Invalid response */
						WiFi_Setup_State = WIFI_INVALID_RESP;
				}
				else { /* readline_sta != 0 */
					if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS) || (reading_count > 0))
						WiFi_Setup_State = WIFI_TIMEOUT; /* Time-out, ==> Change state */
				}			
			}				
			break;
			
		/* --- Time-out --- */
		case WIFI_TIMEOUT:
			if(!on_enterstate) {
				/* Re-try reset state */
				WiFi_Reset_index = 0; /* Retry Reset process */
				WiFi_Setup_State = WIFI_RESET;
			}
			break;
			
		/* --- Invalid response --- */
		case WIFI_INVALID_RESP:
			if(on_enterstate) {
				SysTimer_uS_Start(&WiFi_Timer_uS, 1000000); /* 1 Second */
			}
			else {
				if(SysTimer_uS_IsTimeout(&WiFi_Timer_uS)) {
					/* Re-try reset state */
					WiFi_Reset_index = 0; /* Retry Reset process */
					WiFi_Setup_State = WIFI_RESET;
				}
			}
			break;
			
		/* --- Ready --- */
		case WIFI_READY:
			break;
	}	
	
	/* Reset the current status of Setup */
	*status = (uint32_t)WiFi_Setup_State;
}

void disable_rak410_setup(void)
{
	/* Do nothing */
}

/* ================================================================
** RAK410 UDP Send
*/
void enable_rak410_udpsend(void)
{
	/* Do nothing */
}

typedef enum WiFi_UDPSend_State {
	WIFI_UDPSEND_READY = 0,
	WIFI_UDPSEND_WAIT,
	WIFI_UDPSEND_PORTOPEN,
	WIFI_UDPSEND_PORTOPEN_CHECK,
	WIFI_UDPSEND_ERROR,
} WIFI_UDPSEND_STATE;

static SYS_TIMER_uS_STRUCT WiFi_UDPSendTimer_uS; /* For non-blocking delay */
static WIFI_UDPSEND_STATE WiFi_UDPSend_State = WIFI_UDPSEND_WAIT;
void output_rak410_udpsend(uint32_t ip, uint32_t port, uint32_t data1,uint32_t data2, uint8_t *status)
{
	uint16_t reading_count;
	uint16_t readline_sta;
	int data_index;
	
	/* Wifi Setup is Ready? */
	switch(WiFi_UDPSend_State) 
	{
		/* --- Wait for WiFi setup --- */
		case WIFI_UDPSEND_WAIT:
			if(WiFi_Setup_State == WIFI_READY) {
				WiFi_UDPSend_State = WIFI_UDPSEND_PORTOPEN;
			}
			else {
				break;
			}
			
		/* --- Setup and connect destinatin address and port --- */
		case WIFI_UDPSEND_PORTOPEN:
			/* Write command string to Wifi */
			sprintf(Rak410_write_buffer, "at+udp=%u.%u.%u.%u,%u,%u\r\n", \
				(ip>>24)&0xFF, (ip>>16)&0xFF, (ip>>8)&0xFF,	(ip>>0)&0xFF, port,	port);
			RAK410_Write(Rak410_write_buffer, (uint16_t)strlen(Rak410_write_buffer));
			/* Timeout check setup */
			SysTimer_uS_Start(&WiFi_UDPSendTimer_uS, 5000000); /* 5 Second */
			/* Change State */
			WiFi_UDPSend_State = WIFI_UDPSEND_PORTOPEN_CHECK;
			break;
		case WIFI_UDPSEND_PORTOPEN_CHECK:
			readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count); /* 100ms Data timeout */
			if((readline_sta == 0) && (reading_count > 0)) {					
				if(!strncmp(Rak410_read_buffer, "OK", 2)) {/* Success */
					WiFi_UDPSend_State = WIFI_UDPSEND_READY; /* Ready */
				}
				else {
					/* Invalid response */
					WiFi_UDPSend_State = WIFI_UDPSEND_ERROR;
					break;
				}
			}
			else { /* readline_sta != 0 */
				if(SysTimer_uS_IsTimeout(&WiFi_UDPSendTimer_uS) || (reading_count > 0))
					WiFi_UDPSend_State = WIFI_UDPSEND_ERROR; /* Time-out, ==> Change state */
				break;
			}
				
		/* --- UDP Send is ready --- */
		case WIFI_UDPSEND_READY:
			/* Copy command string */
			strcpy(Rak410_write_buffer, "at+send_data=0,8,"); /* Data length is 8 */
			data_index = strlen(Rak410_write_buffer);		
			/* Copy Data1 */
			memcpy(&Rak410_write_buffer[data_index], &data1, 4);
			data_index += 4;
			/* Copy Data2 */
			memcpy(&Rak410_write_buffer[data_index], &data2, 4);
			data_index += 4;
			/* Copy: "\r\n" */
			strcpy(&Rak410_write_buffer[data_index], "\r\n");
			data_index += 2;		
				
			/* Write command */
			RAK410_Write(Rak410_write_buffer, data_index);

			/* Activate 1Second timer for time-out check. */
			SysTimer_uS_Start(&WiFi_Timer_uS, 1000000); /* 1 Second */	
			/* Wait for response */
			do {
				readline_sta = RAK410_ReadLine(Rak410_read_buffer, sizeof(Rak410_read_buffer), 100000, &reading_count);
			} while((readline_sta == 0) && (reading_count == 0) && !SysTimer_uS_IsTimeout(&WiFi_Timer_uS));
			if((readline_sta == 0) && (reading_count > 0)) {					
				if(!strncmp(Rak410_read_buffer, "OK", 2))
					{ *status = 0; /* Success */ }
				else
					{ WiFi_UDPSend_State = WIFI_UDPSEND_ERROR; /* Error: Did not return "OK\0\r\n"*/ }
			}
			else { 
				WiFi_UDPSend_State = WIFI_UDPSEND_ERROR; /* Fail: missing end of line "\r\n" in received packet */ 
			}
			
			break;
		/* Error */
		case WIFI_UDPSEND_ERROR:
		default:
			/* Invalid */
			break;
	}
		
	/* Status */
	*status = (uint8_t)WiFi_UDPSend_State;
	if(WiFi_UDPSend_State == WIFI_UDPSEND_ERROR) {
		/* Activate reset */
		rak410_reset_activate = (*status != 0);
		/* UDP port need re-configure */
		WiFi_UDPSend_State = WIFI_UDPSEND_WAIT;
	}	
}

void disable_rak410_udpsend(void)
{
	/* Do nothing */
}
