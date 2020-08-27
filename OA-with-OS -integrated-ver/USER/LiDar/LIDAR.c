#include "LIDAR.h"
#define TRUE 1
#define FALSE 0
//激光雷达申请读取数据
uint8_t read_command[4] = {0x5A,0x04,0x04,0x62};
//激光雷达一帧长度为9bytes
uint8_t frame_data[9]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//激光雷达读取距离数据
uint16_t lidar_distance=0;

uint16_t read_one_frame(uint16_t *distance,UART_HandleTypeDef *huart,uint8_t *frame_data)
{
	uint8_t cnt=0;
	uint8_t checksum=0;
	HAL_UART_Transmit(huart,read_command,4,50);
	HAL_Delay(1);
	if(osMessageWaiting(lidar_data_transmission_queueHandle) >= 9)
	{
		do
		{
			frame_data[0] = osMessageGet(lidar_data_transmission_queueHandle,100).value.v;
		}while(osMessageWaiting(lidar_data_transmission_queueHandle) > 0 && frame_data[0] != 0x59);
		frame_data[1] = osMessageGet(lidar_data_transmission_queueHandle,100).value.v;
		if(frame_data[1] == 0x59)
		{
			for(cnt=0;cnt<7;cnt++)
			{
				frame_data[cnt+2] = osMessageGet(lidar_data_transmission_queueHandle,100).value.v;
			}
		}
		for(cnt=0;cnt<8;cnt++)
		{
			checksum += frame_data[cnt];
		}
		checksum &= 0xff;
		if(checksum == frame_data[8])
		{
			*distance = frame_data[2]+(frame_data[3]<<8);
			return TRUE;
			
		}
	}
	return (uint16_t)FALSE;
}


