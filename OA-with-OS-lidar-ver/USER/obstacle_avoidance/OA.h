#ifndef OA
#define OA
#include "main.h"
#include "MOTOR.h"
#include "SERVO.h"

//函数宏定义
#define HOT_ZONE 0
#define COLD_ZONE 1
#define TIMER_OVERFLOW_TIME 100
#define HOT_ZONE_SPIN_DELAY 100
#define COLD_ZONE_SPIN_DELAY 200


typedef struct
{
	volatile uint16_t secure_distance;	//安全距离：在安全距离以内的物体会被认为是障碍物
	volatile uint16_t hotzone_limit;		//热区判定距离：平均障碍物距离小于热区距离时会进入热区模式（扫描速度更快）
	volatile  uint16_t minimum_distance;//最低距离：当有物体小于最低距离时，采取停止、倒车策略躲避
}avoidance_obj;


extern uint16_t obstacle_array[20];

#endif
