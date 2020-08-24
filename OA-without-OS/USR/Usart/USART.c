#include "USART.h"

int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}

//串口中断服务程序
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
	if(huart->Instance==USART2)
	{
		WriteRingBuff(&RINGBUFF_UART2,LENGTH_OF_BUFF,aRxBuffer[0]);
	}
	if(huart->Instance==USART3)
	{
		WriteRingBuff(&RINGBUFF_UART3,LENGTH_OF_BUFF,aRxBuffer[0]);
	}
}
