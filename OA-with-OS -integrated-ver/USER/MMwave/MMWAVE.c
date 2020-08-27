#include "MMWAVE.h"
#define MAXIMUM_ANGLE 100
#define ANGLE_SLICE MAXIMUM_ANGLE/5



uint16_t obstacle_array[5];
static uint8_t obstacle_array_counter[5];//引入计数器进行滤波
//uint16_t obstacle_array_counter_mean_distance[5];//计数器个数个数据后，每个数值对应的平均距离

uint8_t mmwave_process_data(uint16_t *obstacle_array_counter_mean_distance,uint8_t frame_number_to_filter,uint8_t obstacle_confirm_threshold)
{
	/*读取mmwave消息队列里存储的毫米波雷达信息*/
	osEvent message;
	static uint8_t rx_data_header[2];
	static uint8_t rx_data_package[8];
	static uint8_t frames_rx_counter=0;
//	uint8_t object_number = 0;
	uint8_t temp,temp1;
	uint16_t distance;
	int8_t angle;
	rx_data_header[0] = 0;
	rx_data_header[1] = 1;

	if(frames_rx_counter == 0)
	{
		for(temp1=0;temp1<5;temp1++)
		{
			obstacle_array_counter[temp1] = 0;
			obstacle_array_counter_mean_distance[temp1] = 0xFFFF;
		}		
	}

	message = osMessageGet(mmwave_data_transmission_queueHandle,osWaitForever);
	if(message.status == osEventMessage)
	{
		rx_data_header[0] = message.value.v;
	}	
	
	if(rx_data_header[0] == 0x0C || rx_data_header[0] == 0x0B)
	{
		message = osMessageGet(mmwave_data_transmission_queueHandle,osWaitForever);
		if(message.status == osEventMessage)
		{
			rx_data_header[1] = message.value.v;
		}				
	}
	
	if(rx_data_header[1] == 0x07)//包头已收到
	{
		switch(rx_data_header[0])
		{
			case 0x0B:
			{  /*在0x0B包内读取检测到的目标个数*/
//				message = osMessageGet(mmwave_data_transmission_queueHandle,osWaitForever);
//				if(message.status == osEventMessage)
//				{
//					object_number = message.value.v;//获得检测目标个数
//				}			
//				printf("目标个数：%d \n",object_number);
				break;
			}
			case 0x0C:
			{
				for(temp = 0;temp<8;temp++)
				{//接收完整的一帧
					message = osMessageGet(mmwave_data_transmission_queueHandle,osWaitForever);
					if(message.status == osEventMessage)
					{
						rx_data_package[temp] = message.value.v;
					}
				}
				frames_rx_counter += 1;
				distance = (((uint16_t)rx_data_package[2])<<8) +	rx_data_package[3];	
				angle = ((int8_t)rx_data_package[4])*2 - 90;
//				printf("当前目标方位角:%d,距离:%d\n",angle,distance);
				
				for(temp1=0;temp1<5;temp1++)
				{
					obstacle_array[temp1] = 0;
				}
				
				for(temp1=0;temp1<5;temp1++)
				{
					if(-(MAXIMUM_ANGLE/2)+temp1*ANGLE_SLICE <=angle && angle<=-(MAXIMUM_ANGLE/2)+(temp1+1)*ANGLE_SLICE)
					{
						obstacle_array_counter[temp1] += 1;
						obstacle_array_counter_mean_distance[temp1] += distance;//初始值为0xFFFF，再加会导致溢出，这里利用了溢出
						obstacle_array[temp1] = distance;
						break;
					}
				}
				
				if(frames_rx_counter >= frame_number_to_filter)
				{
					frames_rx_counter = 0;
					for(temp1=0;temp1<5;temp1++)
					{
						if(obstacle_array_counter[temp1] >= obstacle_confirm_threshold)
							obstacle_array_counter_mean_distance[temp1] /= frame_number_to_filter;
						else
							obstacle_array_counter_mean_distance[temp1] = 0xFFFF;
					}	
					printf("\n");
					for(temp1=0;temp1<5;temp1++)
					{
						printf(" %d ",obstacle_array_counter_mean_distance[temp1]);
					}printf("\n");					
					return 1;
				}

			}
		}
	}
	return 0;
}


uint16_t process_obstacle_array(uint16_t *minimum_dis,avoidance_obj avoid_params,uint16_t *obstacle_array)
{
	uint8_t temp1,temp2;
	uint16_t byte_form_array;
	uint8_t obstacle_bitmask_array[5];
	*minimum_dis = obstacle_array[0];//找出障碍物的最小距离
	for(temp1=0;temp1<5;temp1++)
	{
		/*找出最小的障碍物距离*/
		if(obstacle_array[temp1]<=*minimum_dis)
			*minimum_dis = obstacle_array[temp1];
		
		/*将障碍物信息转换为掩码格式*/
		if(obstacle_array[temp1] <= avoid_params.secure_distance)
			obstacle_bitmask_array[temp1] = 1;
		else
			obstacle_bitmask_array[temp1] = 0; 
	}
	/*测试*/
	printf("\n");
	for(temp1=0;temp1<5;temp1++)
	{
		printf(" %d ",obstacle_bitmask_array[temp1]);
	}printf("\n");	
		
		/*将掩码格式转换为字节形式：{1,0,1,0,1} -> (uint16_t)10101*/
		byte_form_array = 0;
		for(temp1=0;temp1<5;temp1++)
		{
			temp2 = obstacle_bitmask_array[temp1];
			byte_form_array |= temp2 << (4-temp1);
		}
			
		return byte_form_array;
}

