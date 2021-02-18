#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define CHASSIS_CAN_ID_LF	0x205U
#define CHASSIS_CAN_ID_RF	0x206U
#define CHASSIS_CAN_ID_LB	0x207U
#define CHASSIS_CAN_ID_RB	0x208U

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void	CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void 	CAN_SendDataBuff(drv_type_t drv_type, uint32_t std_id, int16_t *txBuff);

#endif
