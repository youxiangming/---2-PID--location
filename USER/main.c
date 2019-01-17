#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "can.h"
#include "timer.h"
#include "Motor_Init.h"

int main(void)
{ 
	TIM3_Int_Init(200-1,420-1);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);  
	uart_init(115200);	
	LED_Init();					
	KEY_Init(); 			  
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//1M
 	TIM3_Int_Init(200-1,420-1);
	while(1); 
}

