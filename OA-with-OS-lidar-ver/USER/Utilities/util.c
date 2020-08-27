#include "util.h"

//==============文件内包含常用的函数定义===============
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}

//串口中断服务程序
uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
			osMessagePut(lidar_data_transmission_queueHandle,aRxBuffer[0],osWaitForever);
	}
	if(huart->Instance==USART3)
	{
		;
	}
}
