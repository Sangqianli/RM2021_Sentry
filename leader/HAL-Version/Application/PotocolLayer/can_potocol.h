#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
#define SHOOT_DATA_ID   0x2ffU
#define MASTER_MODE_ID  0x200U
#define CHASSIS_CAN_ID	0x201U
#define DIAL_CAN_ID	    0x205U
#define GIMBAL_CAN_ID_PITCH	0x208U
#define GIMBAL_CAN_ID_YAW	0x206U
#define	FIRE_LEFT_ID		0x206U
#define	FIRE_RIGHT_ID		0x207U
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void CAN1_SendDataBuff( uint32_t std_id, int16_t *txBuff);
void CAN2_SendDataBuff( uint32_t std_id, int16_t *txBuff);

#endif
