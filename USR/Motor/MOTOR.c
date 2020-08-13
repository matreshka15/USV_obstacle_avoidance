#include "MOTOR.h"

//void forward(void)
//{
//	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
//	//HAL_GPIO_WritePin()
//}
void TIM_SetTIM3Compare2(u32 compare)
{
	TIM3->CCR2=compare;
}
void TIM_SetTIM3Compare3(u32 compare)
{
	TIM3->CCR3=compare;
}

void left_wheel_forward(u8 speed_pwm)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	TIM_SetTIM3Compare2(speed_pwm);
}
void left_wheel_backward(u8 speed_pwm)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	TIM_SetTIM3Compare2(speed_pwm);
}
void left_wheel_stop(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	
}

void right_wheel_forward(u8 speed_pwm)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	TIM_SetTIM3Compare3(speed_pwm);
}
void right_wheel_backward(u8 speed_pwm)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	TIM_SetTIM3Compare3(speed_pwm);
}
void right_wheel_stop(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
}


void forward(u8 speed_pwm)
{
	left_wheel_forward(speed_pwm);
	right_wheel_forward((u8)((float)speed_pwm*1.3));
}

void backward(u8 speed_pwm)
{
	left_wheel_backward(speed_pwm);
	right_wheel_backward((u8)((float)speed_pwm*1.5));	
}
void stop(void)
{
	right_wheel_stop();
	left_wheel_stop();
}

void turn(u8 left_speed_pwm,u8 right_speed_pwm)
{
	left_wheel_forward(left_speed_pwm);
	right_wheel_forward((u8)((float)right_speed_pwm*1.4));
}
void left_steel(void)
{
	turn(18,25);
}
void right_steel(void)
{
	turn(25,18);
}
