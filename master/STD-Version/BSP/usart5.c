#include "usart5.h"
#include <string.h>




 u8 jbus_rx_buffer[200];


//串口5配置
void UART5_Config(uint32_t bound)
{
       //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//使能UART5时钟

    //串口5对应引脚复用映射
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOC12复用为UART5
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOD2复用为UART5 
    //UART5端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOC11与GPIOC12
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC12

    //UART5端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOD2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PD2
    //UART5 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
    USART_Init(UART5, &USART_InitStructure); //初始化串口5
    USART_Cmd(UART5, ENABLE);  //使能串口5
    USART_ClearFlag(UART5, USART_FLAG_TC);  

    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);//开启相关中断
    //Usart5 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//串口5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;       //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器、

}


//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//外设到存储器模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量  
void MYDMA_ConfigPtoM(DMA_Stream_TypeDef *DMA_Streamx)
{ 

    DMA_InitTypeDef  DMA_InitStructure;

    if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 

    }else 
    {
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
    }
  DMA_DeInit(DMA_Streamx);

    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 

  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(UART5->DR);// par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)jbus_rx_buffer;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = 200;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
			
//	DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream0,ENABLE);		

} 


void USART5_Init()
{
	
	 UART5_Config(115200);
      //串口5接收DMA配置
   MYDMA_ConfigPtoM(DMA1_Stream0); 
   DMA2_Stream2->NDTR = 0;
   USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE); //允许DMA接收
	
	
	
	
}



 


