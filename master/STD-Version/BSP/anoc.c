/**
 * @file        anoc.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        26-February-2020
 * @brief       This file includes the ANOC Protocol external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.9.0)
 * @Version		
 */

/**
 *	@Zigbee\anoc
 *	Zigbee结合匿名上位机无线调试
 */
 
/* Includes ------------------------------------------------------------------*/
#include "anoc.h"


/* Private typedef -----------------------------------------------------------*/
//typedef struct {
//	uint8_t sof[2];
//	uint8_t cmd_id;
//	uint8_t data_length;
//}Anoc_Frame_Header_t;

//typedef struct {
//	uint8_t buf[ANOC_MAX_DATA_LENGTH];
//}Anoc_Data_t;

//typedef struct {
//	uint8_t check_sum;
//}Anoc_Frame_Tailer_t;

//typedef struct {
//	Anoc_Frame_Header_t	FrameHeader;
//	Anoc_Data_t			Data;
//	Anoc_Frame_Tailer_t	FrameTailer;
//}Anoc_TxPacket_t;

//typedef enum {
//	ANOC_CMD_SENSER = 0x02
//}Anoc_Cmd_Names_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//static Anoc_TxPacket_t	Anoc_TxPacket = {
//	{
//		.sof[0] = 0xAA,
//		.sof[1] = 0xAA,
//		.cmd_id = ANOC_CMD_SENSER,
//		.data_length = 18,
//	},
//};

/* ## Global variables ## ----------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* API functions -------------------------------------------------------------*/

void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口5对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOC 12 复用为 usart1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOD 2 复用为 usart1
	
	// usart1 端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10; //GPIO D2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //无上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PD2

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//接收模式
  USART_Init(USART1, &USART_InitStructure); //初始化 usart1

	USART_Cmd(USART1, ENABLE);

}

void Anoc_SendChar(u8 chr)
{
	USART1->DR = chr;
	
	while((USART1->SR&0x40)==0);
}

//void USART1_IRQHandler(void)
//{
//  u8 receive_lenth;
//	
//	if(USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET)
//	{
//		receive_lenth = USART1->SR;
//		receive_lenth = USART1->DR;
//	}
//	
//}


void ANOC_SendToPc1(int16_t rol, int16_t pit, int16_t yaw)
{
	uint16_t static send_cnt = 0;
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[17];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 12;
	
	data_to_pc[4] = (rol & 0xff00) >> 8;
	data_to_pc[5] = (rol & 0xff);
	data_to_pc[6] = (pit & 0xff00) >> 8;
	data_to_pc[7] = (pit & 0xff);
	data_to_pc[8] = (yaw & 0xff00) >> 8;
	data_to_pc[9] = (yaw & 0xff);
	
	data_to_pc[10] = 0;
	data_to_pc[11] = 0;
	data_to_pc[12] = 0;
	data_to_pc[13] = 0;
	data_to_pc[14] = 0;
	data_to_pc[15] = 0;
		
	for(i = 0; i < 16; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[16] = check_sum & 0xff;
	
	if(send_cnt < 5000) {
		for(i = 0; i < 17; i++) {
			Anoc_SendChar(data_to_pc[i]);
		}
		send_cnt++;
	}
}

void ANOC_SendToPc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[28];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 18;
	
	data_to_pc[4] = (ax & 0xff00) >> 8;
	data_to_pc[5] = (ax & 0xff);
	data_to_pc[6] = (ay & 0xff00) >> 8;
	data_to_pc[7] = (ay & 0xff);
	data_to_pc[8] = (az & 0xff00) >> 8;
	data_to_pc[9] = (az & 0xff);
	
	data_to_pc[10] = (gx & 0xff00) >> 8;
	data_to_pc[11] = (gx & 0xff);
	data_to_pc[12] = (gy & 0xff00) >> 8;
	data_to_pc[13] = (gy & 0xff);
	data_to_pc[14] = (gz & 0xff00) >> 8;
	data_to_pc[15] = (gz & 0xff);
	
	data_to_pc[16] = (mx & 0xff00) >> 8;
	data_to_pc[17] = (mx & 0xff);
	data_to_pc[18] = (my & 0xff00) >> 8;
	data_to_pc[19] = (my & 0xff);
	data_to_pc[20] = (mz & 0xff00) >> 8;
	data_to_pc[21] = (mz & 0xff);
	
	for(i = 0; i < 22; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[22] = check_sum & 0xff;
	
	for(i = 0; i < 23; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}


void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[29];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 24;
	
	/* 以默认的小端模式发送数据 */
	memcpy(&data_to_pc[4], (uint8_t*)&yaw, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&pitch, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&roll, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&rateYaw, 4);
	memcpy(&data_to_pc[20], (uint8_t*)&ratePitch, 4);
	memcpy(&data_to_pc[24], (uint8_t*)&rateRoll, 4);
	
//	send_cnt++;
//	以下操作会将数据转化成大端模式发送出去
//	data_to_pc[16] = (rateYaw & 0xff00) >> 8;
//	data_to_pc[17] = (rateYaw & 0xff);
//	data_to_pc[18] = (ratePitch & 0xff00) >> 8;
//	data_to_pc[19] = (ratePitch & 0xff);
//	data_to_pc[20] = (rateRoll & 0xff00) >> 8;
//	data_to_pc[21] = (rateRoll & 0xff);
	
	for(i = 0; i < 28; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[28] = check_sum & 0xff;
	
//	USART1_DMA_SendBuf(data_to_pc, 23);
	for(i = 0; i < 29; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc2(float shoot_freq, float shoot_ping, float shoot_heat, float shoot_pwm, float shoot_speed, float shoot_xxx)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[29];	
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 24;	
	
	memcpy(&data_to_pc[4], (uint8_t*)&shoot_freq, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&shoot_ping, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&shoot_heat, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&shoot_pwm, 4);
	memcpy(&data_to_pc[20], (uint8_t*)&shoot_speed, 4);
	memcpy(&data_to_pc[24], (uint8_t*)&shoot_speed, 4);
	
	for(i = 0; i < 28; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[28] = check_sum & 0xff;	
	
	for(i = 0; i < 29; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc3(void)
{
	
}
