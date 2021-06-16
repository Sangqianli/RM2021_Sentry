#ifndef __JUDGE_POTOCOL_H
#define __JUDGE_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "crc.h"
#include "cmsis_os.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void USART5_rxDataHandler(uint8_t *rxBuf);

#endif
