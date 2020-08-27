#include "SERVO.h"

uint8_t start_angle_pos = 10;
uint8_t end_angle_pos = 20;

uint8_t servo_pos = 5;
uint8_t last_servo_pos=0; //用来存储上一次舵机位置信息
uint8_t direction = 1;


void TIM_SetTIM3Compare1(uint32_t compare)
{
	TIM3->CCR1=compare;
}

//ARG
//position:转动开始位置；direction:转动方向；delay:向舵机发送数据延时
uint8_t spin_servo(uint8_t *position,uint8_t * direction,uint8_t max_pos,uint8_t min_pos)
{
	uint8_t direction_shift_flag = 0;
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


void initialize_servo(uint8_t target_pos,uint8_t round,uint16_t delay)
{
	uint8_t temp_pos = 5;
	uint8_t direction = 1;
	uint8_t half_round=0;
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

int16_t convert_servo_angle(uint8_t current_pos)
{
	int16_t angle_degree = (current_pos - 15)*angle_per_pulse;
	return angle_degree;
}
