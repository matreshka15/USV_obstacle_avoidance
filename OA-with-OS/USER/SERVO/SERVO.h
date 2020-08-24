#ifndef SERVO
#define SERVO
#include "main.h"
#include "stdio.h"
//#include "LIDAR.h"
////////////////////////////////////////////////////////////////////////////////// 	
//舵机参数设置
#define servo_min_angle 0
#define servo_max_angle 180
#define angle_per_pulse (servo_max_angle-servo_min_angle)/20



#define TIMER_OFF() __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE)
#define TIMER_ON() __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE)

extern uint8_t direction;
extern uint8_t servo_pos;
extern uint8_t last_servo_pos;

extern uint8_t end_angle_pos;
extern uint8_t start_angle_pos;
void TIM_SetTIM3Compare1(uint32_t compare);
uint8_t spin_servo(uint8_t *position,uint8_t * direction,uint8_t max_pos,uint8_t min_pos);
void initialize_servo(uint8_t target_pos,uint8_t round,uint16_t delay);
int16_t convert_servo_angle(uint8_t current_pos);
#endif
