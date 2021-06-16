#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rc_sensor.h"
#include "imu_sensor.h"
#include "path_sensor.h"
#include "motor.h"
#include "vision_sensor.h"
#include "judge_sensor.h"

#include "can_potocol.h"
#include "vision_potocol.h"
#include "judge_potocol.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
	motor_t		*motor[MOTOR_CNT];
	path_t              *path_sen;
	vision_sensor_t     *vision_sen;
	judge_sensor_t      *judge_sen;
	master_t            *master_sen;
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
