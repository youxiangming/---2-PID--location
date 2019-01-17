#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
	CanTxMsg TxMessage;
	CanRxMsg RxMessage;
	Motor_Nature Motor_base;
	s8 Send[8];	
	float Output;
	u16 Full_angle=360;
	u16 Full_range=8191;
	s16 angle_error;
	s16 angle_old_error=0;
	float Out_Speed;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef  NVIC_InitStructure;

    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		

	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	return 0;
}   
void CAN1_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);	
		Date_recieve_analysis();
		CAN_ClearITPendingBit(CAN1,CAN_FIFO0);
	}
}
void CAN1_Send_Msg(s8*msg,s8 len)
{	
  u8 mbox;
  u16 i;
  TxMessage.StdId=0x200;	 // ��׼��ʶ��Ϊ0
	TxMessage.IDE=0;		
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}
void Date_recieve_analysis(void)
{
	Motor_base.motor_angle=(u16)((RxMessage.Data[0]<<8)|RxMessage.Data[1]);
	Motor_base.motor_speed=(s16)((RxMessage.Data[2]<<8)|RxMessage.Data[3]);
	Motor_base.motor_dianliu=(s16)((RxMessage.Data[4]<<8)|RxMessage.Data[5]);
	Motor_base.motor_temperature=(s16)RxMessage.Data[6];
}
s16 Old_error,error,n=0;
s16 real_speed;
float	Speed_PID_Change(float target,s16 real_speed){
	float Kp=5.1;
	float Ki=4.3,Kd=0.045;
	if(real_speed>1500)	return 1500;
	error=target-real_speed;
	if(n==1)Old_error=0;
	if(n==2)Old_error=error;
	Output=Kp*((float)error)+Ki*(((float)Old_error)+((float)error))+Kd*(((float)error)-((float)Old_error));
	n=2;
	if(Output>3000)return 2500;
	if(Output<-3000)return -2500;
	else return Output;
}
float Location_PID_Change(void)
{
	float angle_Kp=1,angle_Ki=0.001,angle_Kd=10;
	s16 target_location=3000,target_real_value=3000;
	angle_error=target_real_value-Motor_base.motor_angle;
	Out_Speed=angle_Kp*((float)angle_error)+angle_Ki*(((float)angle_old_error)+((float)angle_error))+angle_Kd*(((float)angle_error)-((float)angle_old_error));
	angle_old_error=angle_error;
	return Out_Speed;
}
void Motor_Speed_Send_analysis(void)
{
	u8 p;
	s16 final_Speed_Output;
	float final_angle;
	final_angle=Location_PID_Change();
	final_Speed_Output=(s16)Speed_PID_Change(final_angle,Motor_base.motor_speed);
	Send[0]=(final_Speed_Output>>8);
	Send[1]=final_Speed_Output;
	for(p=2;p<8;p++)
	Send[p]=0;
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		Motor_Speed_Send_analysis();
		CAN1_Send_Msg(Send,8);
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}












