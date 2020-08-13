#include "RINGBUFF.h"

//环形缓冲区定义
RingBuff_t RINGBUFF_UART2;
RingBuff_t RINGBUFF_UART3;
//环形缓冲区操作函数
void RingBuff_Init(RingBuff_t* ringbuff)
{
	ringbuff->Head = 0;
	ringbuff->Tail = 0;
	ringbuff->Length = 0;
}

u8 WriteRingBuff(RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 data)
{
	if(ringbuff->Length >= RINGBUFF_LEN)
	{
		return FALSE;
	}
	ringbuff->Ring_Buff[ringbuff->Tail] = data;
	ringbuff->Tail = (ringbuff->Tail+1)%RINGBUFF_LEN;
	ringbuff->Length++;
	return TRUE;
}

u8 Read_ringBuff(RingBuff_t * ringbuff,u16 RINGBUFF_LEN,u8 *rData)
{
	if(ringbuff->Length == 0)
	{
		return FALSE;
	}
	*rData = ringbuff->Ring_Buff[ringbuff->Head];
	ringbuff->Head = (ringbuff->Head+1)%RINGBUFF_LEN;
	ringbuff->Length--;
	return TRUE;
}
