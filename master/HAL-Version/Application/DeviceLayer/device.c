/**
 * @file        device.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
	.rc_sen = &rc_sensor,
	.imu_sen = &imu_sensor,
	.path_sen = &path_sensor,
	.motor[CHASSIS] = &motor[CHASSIS],
	.motor[DIAL] = &motor[DIAL],
	.motor[GIMBAL_PITCH] = &motor[GIMBAL_PITCH],
	.motor[GIMBAL_YAW] = &motor[GIMBAL_YAW],
	.vision_sen = &vision_sensor,
	.judge_sen = &judge_sensor,
	.leader_sen = &leader_sensor,	
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEV_Init(void)
{
	dev_list.rc_sen->init(dev_list.rc_sen);
//	dev_list.imu_sen->init(dev_list.imu_sen);
	dev_list.path_sen->init(dev_list.path_sen);
	dev_list.motor[CHASSIS]->init(dev_list.motor[CHASSIS]);
	dev_list.motor[DIAL]->init(dev_list.motor[DIAL]);
	dev_list.motor[GIMBAL_PITCH]->init(dev_list.motor[GIMBAL_PITCH]);
	dev_list.motor[GIMBAL_YAW]->init(dev_list.motor[GIMBAL_YAW]);
	dev_list.vision_sen->init(dev_list.vision_sen);
	dev_list.judge_sen->init(dev_list.judge_sen);	
	dev_list.leader_sen->init(dev_list.leader_sen);	
}
