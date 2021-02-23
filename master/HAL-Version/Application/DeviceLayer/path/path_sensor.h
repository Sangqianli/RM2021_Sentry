#ifndef __PATH_SENSOR_H
#define __PATH_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
/* Exported macro ------------------------------------------------------------*/
#define LEFT_PES !PDin(12)
#define RIGHT_PES !PDin(13)
#define ENCODER_CIRCLE 4000
/* Exported types ------------------------------------------------------------*/
typedef struct path_info_struct {
	int16_t		mileage_now;
	int16_t		mileage_prev;
	int32_t		mileage_total;
	bool		left_touch;
	bool		right_touch;	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
} path_info_t;

typedef struct path_struct {
	path_info_t 	        *info;
	drv_path_t				*driver;
	void					(*init)(struct path_struct *self);
	void					(*update)(struct path_struct *self);
	void					(*check)(struct path_struct *self);	
	void					(*heart_beat)(struct path_struct *self);
	dev_work_state_t		work_state;
	dev_errno_t				errno;
	dev_id_t				id;
} path_t;

extern path_t  path_sensor;

#endif
