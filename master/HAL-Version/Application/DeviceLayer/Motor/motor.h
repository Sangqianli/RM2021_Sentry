#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct motor_info_struct {
	uint16_t	angle;
	int16_t		speed;
	int16_t		current;
	uint16_t	angle_prev;
	int32_t		angle_sum;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;	
} motor_info_t;

typedef struct motor_struct {
	motor_info_t 	*info;
	drv_can_t				*driver;
	void					(*init)(struct motor_struct *self);
	void					(*update)(struct motor_struct *self, uint8_t *rxBuf);
	void					(*check)(struct motor_struct *self);	
	void					(*heart_beat)(struct motor_struct *self);
	dev_work_state_t		work_state;
	dev_errno_t				errno;
	dev_id_t				id;
} motor_t;

extern motor_t	motor[MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/


#endif
