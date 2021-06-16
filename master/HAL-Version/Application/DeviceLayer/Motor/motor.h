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



typedef __packed struct
{
    uint8_t attack_now : 1;
	uint8_t is_hero : 1;
	uint8_t others :6;
}leader_mode_t;
typedef struct leader_info_struct
{
	leader_mode_t data;
	uint8_t temporary_data;
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}leader_info_t;

typedef struct leader_struct{
	leader_info_t *info;
	void					(*init)(struct  leader_struct *self);
	void					(*update)(struct  leader_struct *self, uint8_t *rxBuf);	
	void					(*heart_beat)(struct  leader_struct *self);
	dev_work_state_t		work_state;
}leader_t;

extern motor_t	motor[MOTOR_CNT];

extern leader_t leader_sensor;
/* Exported functions --------------------------------------------------------*/


#endif
