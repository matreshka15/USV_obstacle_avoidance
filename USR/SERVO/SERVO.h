#ifndef SERVO
#define SERVO
#include "sys.h"
#include "main.h"
#include "stdio.h"
#include "LIDAR.h"
////////////////////////////////////////////////////////////////////////////////// 	
//舵机参数设置
#define servo_min_angle 0
#define servo_max_angle 180
#define angle_per_pulse (servo_max_angle-servo_min_angle)/20



#define TIMER_OFF() __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE)
#define TIMER_ON() __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE)

extern u8 direction;
extern u8 servo_pos;
extern u8 last_servo_pos;

extern u8 end_angle_pos;
extern u8 start_angle_pos;
void TIM_SetTIM3Compare1(u32 compare);
u8 spin_servo(u8 *position,u8 * direction,u8 max_pos,u8 min_pos);
void initialize_servo(u8 target_pos,u8 round,u16 delay);
int16_t convert_servo_angle(u8 current_pos);
#endif
