#ifndef MM_WAVE
#define MM_WAVE
#include "main.h"
#include "cmsis_os.h"
#include "util.h"
#include "OA.h"
uint8_t mmwave_process_data(uint16_t *obstacle_array,uint8_t frame_number_to_filter,uint8_t obstacle_confirm_threshold);
uint16_t process_obstacle_array(uint16_t *minimum_dis,avoidance_obj avoid_params,uint16_t *obstacle_array);

#endif
