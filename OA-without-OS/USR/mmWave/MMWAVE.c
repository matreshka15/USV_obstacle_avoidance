#include "MMWAVE.h"
#if OA_SENSOR == mmWAVE

u32 read_one_frame_mmwave(RingBuff_t * ringbuff,u16 RINGBUFF_LEN)
{
	u8 cnt=0;
	u8 temp,temp1;
	u8 object_number = 0;
	u16 minimum_distance = 0;
	u16 temp_distance = 0;
	if(ringbuff->Length >= 15)
	{
		do
		{
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
		}while(ringbuff->Length >= 9 && temp != 0x7E);
		
		Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
		if(temp == 0x7E)
		{
			for(cnt=0;cnt<3;cnt++)
			{
				Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
			}
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&object_number);
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp1);
			minimum_distance = ((((u16)(temp)) << 8)|((u16)(temp1)));
			
			for(cnt=0;cnt<object_number-1;cnt++)
			{
				Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
				Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp1);
				temp_distance = ((((u16)(temp)) << 8)|(u16)(temp1));
				if(temp_distance < minimum_distance)
				{
					minimum_distance = temp_distance;
				}
			}
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp);
			Read_ringBuff(ringbuff,RINGBUFF_LEN,&temp1);
			if(temp == 0xEF&&temp1 == 0xEF)
			{
				return ((((u32)(object_number)) << 16)|(u32)(minimum_distance));
			}
		}
	}
	return (u32)FALSE;
}
#endif
