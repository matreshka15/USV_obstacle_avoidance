#ifndef RINGBUFF
#define RINGBUFF

#include "sys.h"
#include "USART.h"
#define LENGTH_OF_BUFF 40
/*环形缓冲区结构体定义*/
typedef struct
{
	volatile u16 Head;
	volatile u16 Tail;
	volatile u16 Length;
	volatile u8 Ring_Buff[LENGTH_OF_BUFF];
}RingBuff_t;

void RingBuff_Init(RingBuff_t * ringbuff);
u8 WriteRingBuff(RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 data);
u8 Read_ringBuff(RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 *rData);

extern RingBuff_t RINGBUFF_UART2;
extern RingBuff_t RINGBUFF_UART3;
#endif
