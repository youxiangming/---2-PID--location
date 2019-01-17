#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
void CAN1_Send_Msg(s8* msg,s8 len);						//发送数据

void Motor_Speed_Send_analysis(void);

float Location_PID_Change(void);
	
typedef struct{
	u16 motor_angle;
	s16 motor_speed;
	s16 motor_dianliu;
	s16 motor_temperature;
	u16 motor_real_angle;
} Motor_Nature;
void Date_recieve_analysis(void);
float	Speed_PID_Change(float target,s16 real_speed);
#endif

















