#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	CHASSIS_MODE_NORMAL 		= 0, // 正常模式
	CHASSIS_MODE_BUFF   		= 1, // 打符模式
	CHASSIS_MODE_RELOAD_BULLET	= 2, // 底盘低速补弹模式
	CHASSIS_MODE_SZUPUP			= 3, // SZU爬坡模式
} chassis_mode_t;

typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
	float		out;
} chassis_motor_pid_t;

typedef struct {
	pid_ctrl_t	angle;
	float 		out;
} chassis_z_pid_t;

typedef struct {
	chassis_motor_pid_t		(*motor)[CHAS_MOTOR_CNT];
	chassis_z_pid_t			*z_atti;	// z方向姿态
} chassis_ctrl_t;

typedef struct {
	chassis_motor_t	*chas_motor[CHAS_MOTOR_CNT];
	gimbal_motor_t	*yaw_motor;
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		remote_mode;
	co_pid_mode_t		co_pid_mode;
	chassis_mode_t		local_mode;
	co_angle_logic_t	co_angle_logic;
	
	bool				top_gyro;
}chassis_info_t;

typedef struct chassis{
	chassis_ctrl_t	*controller;
	chassis_dev_t	*dev;
	chassis_info_t	*info;
	bool			test_open;
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
	bool			(*if_back_to_mid_angle)(void);
	bool			(*if_top_gyro_open)(void);
}chassis_t;

extern chassis_t chassis;

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
bool CHASSIS_IfBackToMiddleAngle(void);

#endif
