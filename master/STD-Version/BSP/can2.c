#include "can2.h"

/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/
int CAN2_ID;
CanRxMsg RxMessage;
void CAN2_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;//开漏输出
  	gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉电阻
 	gpio.GPIO_Speed = GPIO_Speed_100MHz;//
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic); 

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0 CAN_Filter_FIFO0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
} 

void CAN2_Send(uint32_t Equipment_ID,int16_t Data0,int16_t Data1,int16_t Data2,int16_t Data3)		
{		
	CanTxMsg TxMessage;
	
	TxMessage.StdId = Equipment_ID;					 //使用的扩展ID
	TxMessage.IDE = CAN_ID_STD;				 //标准模式
	TxMessage.RTR = CAN_RTR_DATA;			 //数据帧RTR=0，远程帧RTR=1
	TxMessage.DLC = 8;							 	 //数据长度为8字节

	TxMessage.Data[0] = Data0>>8; 
	TxMessage.Data[1] = Data0;
	TxMessage.Data[2] = Data1>>8; 
	TxMessage.Data[3] = Data1;
	TxMessage.Data[4] = Data2>>8; 
	TxMessage.Data[5] =	Data2;
	TxMessage.Data[6] = Data3>>8; 
	TxMessage.Data[7] =	Data3;

	CAN_Transmit(CAN2, &TxMessage);	//发送数据
}

uint32_t temp;
int i;
void CAN2_RX0_IRQHandler(void)
{
		
		if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {				 
			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
			CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
			CAN2_ID=RxMessage.StdId-0x200;
			
			switch(CAN2_ID)
			{

			case 1:
			  Chassis.speed_get=RxMessage.Data[2];
			  Chassis.speed_get=Chassis.speed_get<<8;
			  Chassis.speed_get=Chassis.speed_get|RxMessage.Data[3];
			  Chassis.PID_PVM.measure=Chassis.speed_get;//底盘电机3510
			  break;


			case 2:
		      An_Bullet_PPM.position_get_last=An_Bullet_PPM.position_get_now;
	          An_Bullet_PPM.position_get_now=RxMessage.Data[0];
              An_Bullet_PPM.position_get_now= An_Bullet_PPM.position_get_now<<8;
			  An_Bullet_PPM.position_get_now= An_Bullet_PPM.position_get_now|RxMessage.Data[1];
			  if((An_Bullet_PPM.position_get_last-An_Bullet_PPM.position_get_now)>4800)
				{
					An_Bullet_PPM.position_round++;		
				}
				if((An_Bullet_PPM.position_get_now-An_Bullet_PPM.position_get_last)>4800)
				{
					An_Bullet_PPM.position_round--;	
				}
				An_Bullet_PPM.PID_PPM.measure=An_Bullet_PPM.position_round*cnt_per_round+An_Bullet_PPM.position_get_now;
			  
//			  An_Bullet_PPM.PID_PPM.measure= An_Bullet_PPM.position;		
			
			
              Launcher_Dial.speed_get=RxMessage.Data[2];
              Launcher_Dial.speed_get=Launcher_Dial.speed_get<<8;
			  Launcher_Dial.speed_get=Launcher_Dial.speed_get|RxMessage.Data[3];
			  Launcher_Dial.PID_PVM.measure=Launcher_Dial.speed_get;
			  break;
			
			case 10:
			  	
			  Gimbal_yaw_PPM.position_get_last=Gimbal_yaw_PPM.position_get_now;
			  Gimbal_yaw_PPM.position_get_now=RxMessage.Data[0];
			  Gimbal_yaw_PPM.position_get_now=Gimbal_yaw_PPM.position_get_now<<8;
			  Gimbal_yaw_PPM.position_get_now=Gimbal_yaw_PPM.position_get_now|RxMessage.Data[1];
			  if((Gimbal_yaw_PPM.position_get_last-Gimbal_yaw_PPM.position_get_now)>8100)
				{
					Gimbal_yaw_PPM.position_round++;
				}
				if((Gimbal_yaw_PPM.position_get_now-Gimbal_yaw_PPM.position_get_last)>8100)
				{
					Gimbal_yaw_PPM.position_round--;
				}
				Gimbal_yaw_PPM.PID_PPM.measure=(Gimbal_yaw_PPM.position_round*cnt_per_round+Gimbal_yaw_PPM.position_get_now-5790);
				
				
			  Gimbal_yaw_PVM.speed_get=RxMessage.Data[2];
			  Gimbal_yaw_PVM.speed_get=Gimbal_yaw_PVM.speed_get<<8;
			  Gimbal_yaw_PVM.speed_get=Gimbal_yaw_PVM.speed_get|RxMessage.Data[3];				
			  break;
				
			case 9:
			  Gimbal_pitch_PPM.position_get_last=Gimbal_pitch_PPM.position_get_now;
			  Gimbal_pitch_PPM.position_get_now=RxMessage.Data[0];
			  Gimbal_pitch_PPM.position_get_now=Gimbal_pitch_PPM.position_get_now<<8;
			  Gimbal_pitch_PPM.position_get_now=Gimbal_pitch_PPM.position_get_now|RxMessage.Data[1];
			  if((Gimbal_pitch_PPM.position_get_last-Gimbal_pitch_PPM.position_get_now)>8000)
				{
					Gimbal_pitch_PPM.position_round++;
				}
				if((Gimbal_pitch_PPM.position_get_now-Gimbal_pitch_PPM.position_get_last)>8000)
				{
					Gimbal_pitch_PPM.position_round--;
				}			  
			  
			  Gimbal_pitch_PPM.PID_PPM.measure=-(Gimbal_pitch_PPM.position_round*cnt_per_round+Gimbal_pitch_PPM.position_get_now-6680);
			  
			  Gimbal_pitch_PVM.speed_get=RxMessage.Data[2];
			  Gimbal_pitch_PVM.speed_get=Gimbal_pitch_PVM.speed_get<<8;
			  Gimbal_pitch_PVM.speed_get=Gimbal_pitch_PVM.speed_get|RxMessage.Data[3];		
				break;
				default:				
				  break;
					

			}				
			

		}
}

void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}



