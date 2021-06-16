#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define LEADER_DATA_ID  0x1FFU
#define CHASSIS_CAN_ID	0x201U
#define DIAL_CAN_ID	    0x202U
#define GIMBAL_CAN_ID_PITCH	0x209U
#define GIMBAL_CAN_ID_YAW	0x20AU

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void CAN1_SendDataBuff( uint32_t std_id, int16_t *txBuff);
void CAN2_SendDataBuff( uint32_t std_id, int16_t *txBuff);

#endif
