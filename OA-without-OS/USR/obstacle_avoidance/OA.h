#ifndef OA
#define OA
#include "sys.h"
#include "USART.h"
#include "RINGBUFF.h"
#include "main.h"
#include "SERVO.h"
#include "MOTOR.h"
#include "MMWAVE.h"
//函数宏定义
#define HOT_ZONE 0
#define COLD_ZONE 1
#define TIMER_OVERFLOW_TIME 100
#define HOT_ZONE_SPIN_DELAY 100
#define COLD_ZONE_SPIN_DELAY 200
extern u8 ZONE_FLAG;
extern u8 deducted_obstacle_array[5];


typedef struct
{
	volatile u16 secure_distance;	//安全距离：在安全距离以内的物体会被认为是障碍物
	volatile u16 hotzone_limit;		//热区判定距离：平均障碍物距离小于热区距离时会进入热区模式（扫描速度更快）
	volatile  u16 minimum_distance;//最低距离：当有物体小于最低距离时，采取停止、倒车策略躲避
}avoidance_obj;


extern u16 obstacle_array[20];
#if OA_SENSOR == Lidar
extern avoidance_obj lidar_avoid_params;
u16 read_one_frame(u16 *distance,UART_HandleTypeDef *huart,RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 *frame_data);
u8 obstacle_detection(avoidance_obj lidar_avoid_params,u8 sample_rate,float f_c,
											UART_HandleTypeDef *huart,RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 *frame_data);
#endif
#if OA_SENSOR == mmWAVE
extern avoidance_obj mmWAVE_avoid_params;
u16 obstacle_detection(avoidance_obj mmWAVE_avoid_params,RingBuff_t * ringbuff,u16 RINGBUFF_LEN);
#endif
void obstacle_avoidance(avoidance_obj avoid_params,u8 * deducted_obstacle_array,u16 minimum_distance);
#endif
