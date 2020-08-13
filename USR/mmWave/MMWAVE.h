#ifndef mmWaveSensor
#define mmWaveSensor
#include "sys.h"
#include "RINGBUFF.h"

u32 read_one_frame_mmwave(RingBuff_t * ringbuff,u16 RINGBUFF_LEN);

#endif
