#include "LIDAR.h"
#if OA_SENSOR == Lidar

//激光雷达申请读取数据
u8 read_command[4] = {0x5A,0x04,0x04,0x62};
//激光雷达一帧长度为9bytes
u8 frame_data[9]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//激光雷达读取距离数据
u16 lidar_distance=0;

u16 read_one_frame(u16 *distance,UART_HandleTypeDef *huart,RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 *frame_data)
{
	u8 cnt=0;
	u8 checksum=0;
	HAL_UART_Transmit(huart,read_command,4,50);
	HAL_Delay(1);
	if(ringbuff->Length >= 9)
	{
		do
		{
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&frame_data[0]);
		}while(ringbuff->Length > 0 && frame_data[0] != 0x59);
		
		Read_ringBuff(ringbuff,RINGBUFF_LEN,&frame_data[1]);
		if(frame_data[1] == 0x59)
		{
			for(cnt=0;cnt<7;cnt++)
			{
				Read_ringBuff(ringbuff,RINGBUFF_LEN,&frame_data[cnt+2]);
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
	return (u16)FALSE;
}


/*一通道数字低通滤波器
输入量与输出量是同一个量。
*/
float ch1_Float_LowPass_Filter(float input,float last_output,u8 sample_rate,float f_c)
{
	float tau = 1/(2*3.14*f_c);
	float sample_cycle = 1/(float)sample_rate;
	float a = sample_cycle/(tau+sample_cycle);
	//printf("\ninput= %f,inc=%f,a=%f,LPF:%f\n",input,last_output,a,(input*a+(1-a)*last_output));
	return (input*a+(1-a)*last_output);
}

//利用数字LPF对激光雷达数据滤波，过滤掉异常数据
u16 lidar_data_process(u16 raw_distance,u16 processed_distance,u8 sample_rate,float f_c)
{
	u16 temp_distance_data;
	temp_distance_data = (u16)ch1_Float_LowPass_Filter((float)raw_distance,(float)processed_distance,sample_rate,f_c);
	return temp_distance_data;
}


#endif
