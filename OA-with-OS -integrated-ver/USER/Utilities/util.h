#ifndef UTILITIES
#define UTILITIES

#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define RXBUFFERSIZE   1 //缓存大小
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	
extern uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer

#endif
