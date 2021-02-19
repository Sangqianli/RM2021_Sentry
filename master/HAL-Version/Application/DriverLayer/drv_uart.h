#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rp_config.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void USART1_Init(void);
void USART2_Init(void);
void USART4_Init(void);
void USART5_Init(void);
void UART1_SendData(uint8_t *Data,uint16_t Size);
void UART5_SendData(uint8_t *Data,uint16_t Size);
void UART_SendData(drv_uart_t *drv,uint8_t *txData,uint16_t size);
#endif
