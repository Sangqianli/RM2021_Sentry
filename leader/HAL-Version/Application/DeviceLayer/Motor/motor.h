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
	int16_t     temperature;
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
    uint8_t online : 1;
	uint8_t mode : 2;
	uint8_t attack_colour :2;//红2蓝1没有则默认0
	uint8_t friction_now :1;
	uint8_t dial_now : 1;
	uint8_t others :1;
}master_mode_t;
typedef struct master_info_struct
{
	master_mode_t modes;
	uint16_t cooling_heat;
	uint16_t bullet_speed;
	uint8_t temporary_data;
	uint8_t     mode_pre;	
	bool     mode_siwtch;	
	uint8_t		init_flag;
	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;		
}master_info_t;

typedef struct master_struct{
	master_info_t *info;
	void					(*init)(struct  master_struct *self);
	void					(*update)(struct  master_struct *self, uint8_t *rxBuf);	
	void				    (*check)(struct  master_struct *self);		
	void					(*heart_beat)(struct  master_struct *self);
	dev_work_state_t		work_state;
}master_t;


extern motor_t	motor[MOTOR_CNT];

extern master_t master_sensor;

/* Exported functions --------------------------------------------------------*/


#endif
