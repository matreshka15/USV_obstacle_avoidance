#ifndef MOTOR
#define MOTOR
#include "sys.h"
#include "main.h"
#include "stdio.h"
void TIM_SetTIM3Compare2(u32 compare);
void TIM_SetTIM3Compare3(u32 compare);

void left_wheel_forward(u8 speed_pwm);

void forward(u8 speed_pwm);
void backward(u8 speed_pwm);
void stop(void);
void turn(u8 left_speed_pwm,u8 right_speed_pwm);
void left_steel(void);
void right_steel(void);

#endif
