#ifndef MOTOR
#define MOTOR
#include "main.h"
#include "stdio.h"
void TIM_SetTIM3Compare2(uint32_t compare);
void TIM_SetTIM3Compare3(uint32_t compare);

void left_wheel_forward(uint8_t speed_pwm);

void forward(uint8_t speed_pwm);
void backward(uint8_t speed_pwm);
void stop(void);
void turn(uint8_t left_speed_pwm,uint8_t right_speed_pwm);
void left_steel(void);
void right_steel(void);

#endif
