#include "SERVO.h"
#if OA_SENSOR == Lidar

u8 start_angle_pos = 10;
u8 end_angle_pos = 20;

u8 servo_pos = 5;
u8 last_servo_pos=0; //用来存储上一次舵机位置信息
u8 direction = 1;



void TIM_SetTIM3Compare1(u32 compare)
{
	TIM3->CCR1=compare;
}

//ARG
//position:转动开始位置；direction:转动方向；delay:向舵机发送数据延时
u8 spin_servo(u8 *position,u8 * direction,u8 max_pos,u8 min_pos)
{
	u8 direction_shift_flag = 0;
	//direction = 1:增大方向；0:减小方向
	if(*position>=max_pos-1) 
	{
		*position = max_pos-1;
		*direction=0;
		direction_shift_flag = 1;
	}
	else if(*position<=min_pos)
	{
		*position = min_pos;
		*direction = 1;
		direction_shift_flag = 1;
	}
	switch(*direction)
	{
		case 1:*position+=1;break; 
		case 0:*position-=1;break;
		default: *position = 15;break;
	}
	TIM_SetTIM3Compare1(*position);
	//printf("position = %d\n direction=%d\n",*position,*direction);
	return direction_shift_flag;
}


void initialize_servo(u8 target_pos,u8 round,u16 delay)
{
	u8 temp_pos = 5;
	u8 direction = 1;
	u8 half_round=0;
	spin_servo(&temp_pos,&direction,25,5);
	while(half_round < round || temp_pos != target_pos)
	{
		HAL_Delay(delay);
		if(spin_servo(&temp_pos,&direction,25,5))
		{
			half_round += 1;
		}
	}
}

int16_t convert_servo_angle(u8 current_pos)
{
	int16_t angle_degree = (current_pos - 15)*angle_per_pulse;
	return angle_degree;
}
#endif
