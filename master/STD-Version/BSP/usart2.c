#include "usart2.h"

volatile unsigned char sbus_rx_buffer[25];
RC_Ctl_t RC_Ctl;

void usart2_Init()
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart2;
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef nvic;
	
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

	gpio.GPIO_Pin = GPIO_Pin_3 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);
	
	USART_DeInit(USART2);
	usart2.USART_BaudRate = 100000;
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_Even;
	usart2.USART_Mode = USART_Mode_Rx;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
	USART_Cmd(USART2,ENABLE);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
/* -------------- Configure NVIC ---------------------------------------*/
	nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
/* -------------- Configure DMA -----------------------------------------*/
	
	DMA_DeInit(DMA1_Stream5);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = 18;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_Mode_Normal;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5,&dma);
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);
}


void delay_ms1(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=42000;
		while(a--);
	}
}

void Get_RC_Data(void)
{
	RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right	
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
	RC_Ctl.dial.angle=sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);//²¨ÂÖ
	
  
	if (
		(RC_Ctl.rc.ch0>1684||RC_Ctl.rc.ch0<364)
		||(RC_Ctl.rc.ch1>1684||RC_Ctl.rc.ch1<364)
	  ||(RC_Ctl.rc.ch2>1684||RC_Ctl.rc.ch2<364)
	  ||(RC_Ctl.rc.ch3>1684||RC_Ctl.rc.ch3<364)
	  ||((RC_Ctl.rc.s1 != 1)&&(RC_Ctl.rc.s1 != 2)&&(RC_Ctl.rc.s1 != 3))
	  ||((RC_Ctl.rc.s2 != 1)&&(RC_Ctl.rc.s2 != 2)&&(RC_Ctl.rc.s2 != 3))
	)
    {
			
			
		RC_Ctl.rc.ch0 = 1024;
    RC_Ctl.rc.ch1 = 1024;
    RC_Ctl.rc.ch2 = 1024;
    RC_Ctl.rc.ch3 = 1024;
    
    RC_Ctl.rc.s1 = 3;
    RC_Ctl.rc.s2 = 3;
        
    }
    
    
}

//float  Q=1.8,R=2.5;
//int see;
extern uint32_t  Remote_time;
void DMA1_Stream5_IRQHandler()
{

	
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
//		if(RobotMonitor==control_Mode)
//		{
//		
		Get_RC_Data();
   		
//		}
		Remote_time = sysTickUptime + 20;
		
  }
}






