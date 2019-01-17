#include "Motor_Init.h"
#include "can.h"
s8 test[8];
void MOtor_Init(s32 start)
{
	u8  y;
	s32 start1;
	start1=(s16)start;
	test[0]=(start1>>8);
	test[1]=start1;
	for(y=2;y<8;y++)
	{
			test[y]=0;
	}
	CAN1_Send_Msg(test,8);
}
	
