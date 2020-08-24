#ifndef LiDar
#define LiDar
#include "sys.h"
#include "USART.h"
#include "RINGBUFF.h"
#include "main.h"
#include "SERVO.h"
#include "MOTOR.h"

extern u8 frame_data[9];
extern u16 lidar_distance;

u16 lidar_data_process(u16 raw_distance,u16 processed_distance,u8 sample_rate,float f_c);


#endif
