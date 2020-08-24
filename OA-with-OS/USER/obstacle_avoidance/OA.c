#include "OA.h"

//存放雷达探测到前方障碍物距离数据
uint16_t obstacle_array[20];
uint8_t obstacle_array_bitmask[20];
uint8_t deducted_obstacle_array[5];

//设置避障区域类型
uint8_t ZONE_FLAG =HOT_ZONE;

uint16_t raw_distance,processed_distance;
uint8_t initial_array_fill_flag = 0;
/*设定避障数据*/

avoidance_obj lidar_avoid_params={35,150,25};

uint8_t obstacle_detection(avoidance_obj lidar_avoid_params,uint8_t sample_rate,float f_c,
											UART_HandleTypeDef *huart,RingBuff_t * ringbuff,uint16_t RINGBUFF_LEN,uint8_t *frame_data)
{
	int32_t angle  = 0;
	int32_t temp1=0,temp2=0;//临时变量
	uint8_t minimum_dis = 0;
	const uint8_t divide_span = (end_angle_pos - start_angle_pos)/5;
	const uint8_t skip_digits = (20-(end_angle_pos - start_angle_pos))/2;

	
	if(last_servo_pos != servo_pos)
	{
		last_servo_pos = servo_pos;	
		/*Step 1:检测障碍物距离，读取当前舵机角度*/
		read_one_frame(&raw_distance,huart,ringbuff,LENGTH_OF_BUFF,frame_data);
		//read_one_frame(&raw_distance,&huart2,&RINGBUFF_UART2,LENGTH_OF_BUFF,frame_data);
		if(raw_distance < 2)
		{
			HAL_Delay(20);
			read_one_frame(&raw_distance,huart,ringbuff,LENGTH_OF_BUFF,frame_data);
		}
		
		processed_distance = raw_distance;//lidar_data_process(raw_distance,processed_distance,sample_rate,f_c);//
		angle = convert_servo_angle(servo_pos);
		/*Step 2:读取障碍物数据，等待障碍数组填满*/
		obstacle_array[servo_pos-5] = processed_distance;
		//printf("Angle:%d ,front distance:%d \n",angle,raw_distance);
		
		/*仅当已经收集完一次障碍物数据后再开始数据处理*/
		if(initial_array_fill_flag < (20-2*skip_digits))
		{
			initial_array_fill_flag += 1;
		}else 
		{
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//开启 PWM 通道
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);//开启 PWM 通道
			initial_array_fill_flag = 0xFF;
		}
		/*Step 3:处理障碍数组*/
		if(initial_array_fill_flag == 0xFF)
		{
			//找出障碍物的最小距离
			minimum_dis = obstacle_array[skip_digits];
			for(temp1=(skip_digits);temp1<20-(skip_digits);temp1++)
			{
				if(obstacle_array[temp1]<=minimum_dis)
					minimum_dis = obstacle_array[temp1];
			}
			
			/*Sub step 1:计算障碍物的平均距离，如果小于热区hotzone设定的分界值，则设定为热区模式*/
			temp2 = 0;
			for(temp1=(skip_digits);temp1<20-(skip_digits);temp1++)
			{
				temp2 += obstacle_array[temp1];
			}
			temp2 /= 20;
			if(temp2 <= lidar_avoid_params.hotzone_limit) ZONE_FLAG = HOT_ZONE;
			else ZONE_FLAG = COLD_ZONE;
			//printf("Current mode:%d\n",ZONE_FLAG);
			
			/*Sub step 2:处理长度20的障碍物数组*/
			if(processed_distance >= lidar_avoid_params.secure_distance)
			{
				obstacle_array_bitmask[servo_pos-5] = 0;
			}else
			{
				obstacle_array_bitmask[servo_pos-5] = 1;//1表示检测到障碍物在避障范围内
			}
			
			/*Sub step 2:把长度为20的障碍物数组缩减为5*/
			deducted_obstacle_array[0] = 0;
			deducted_obstacle_array[4] = 0;
			for(temp2=0;temp2<divide_span;temp2++)
			{
				if(obstacle_array_bitmask[(skip_digits)+temp2]==1)deducted_obstacle_array[0] = 1;
				if(obstacle_array_bitmask[(skip_digits+4*divide_span)+temp2]==1)deducted_obstacle_array[4] = 1;
			}
			
//调试用			
//			printf("\n");			
//			for(temp1=0;temp1<20;temp1++)
//			{
//				printf("%d ",obstacle_array[temp1]);
//			}
//			printf("\n");
			
			for(temp1=1;temp1<4;temp1++)
			{
				deducted_obstacle_array[temp1] = 0;
				for(temp2=0;temp2<divide_span;temp2++)
				{
					deducted_obstacle_array[temp1] += obstacle_array_bitmask[(skip_digits+temp1*divide_span)+temp2];
				}
				deducted_obstacle_array[temp1] = (deducted_obstacle_array[temp1]/(divide_span/2))>1?1:0;
			}

		}
		return minimum_dis;		
	}
	else return 0;
}

void obstacle_avoidance(avoidance_obj avoid_params,uint8_t * deducted_obstacle_array,uint16_t minimum_distance)
{
	uint16_t byte_form_array=0;
	uint8_t temp1;
	uint16_t temp2;
	printf("minimum distance:%d, set param:%d",minimum_distance,avoid_params.minimum_distance);
	printf("\n");
	for(temp1=0;temp1<5;temp1++)
	{
		printf("%d ",deducted_obstacle_array[temp1]);
	}printf("\n");
	
	for(temp1=0;temp1<5;temp1++)
	{
		temp2 = deducted_obstacle_array[temp1];
		byte_form_array |= temp2 << (4-temp1);
	}
	
	if(byte_form_array == 0x17||byte_form_array == 0x15||byte_form_array == 0x1d||
		byte_form_array == 0x1f||minimum_distance <= avoid_params.minimum_distance)
			{
				stop();
			  HAL_Delay(100);	
				backward(16);
				HAL_Delay(100);	
			}//当1的个数大于4或障碍数组为10101时，倒车
	else if((byte_form_array & 0x0e) == 0x00) forward(16);
	else if((byte_form_array&0x1c)>>3 >= (byte_form_array&0x07)) left_steel();
	else if((byte_form_array&0x1c)>>3 < (byte_form_array&0x07)) right_steel();
	else printf("undefined");
	
	return;
}
