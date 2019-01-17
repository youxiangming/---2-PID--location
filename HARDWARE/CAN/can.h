#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
void CAN1_Send_Msg(s8* msg,s8 len);						//��������

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

















