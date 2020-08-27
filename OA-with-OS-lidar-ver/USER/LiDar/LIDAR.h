#ifndef LiDar
#define LiDar
#include "main.h"
#include "SERVO.h"
#include "MOTOR.h"
#include "cmsis_os.h"
extern uint8_t frame_data[9];
extern uint16_t lidar_distance;

uint16_t read_one_frame(uint16_t *distance,UART_HandleTypeDef *huart,uint8_t *frame_data);


#endif
