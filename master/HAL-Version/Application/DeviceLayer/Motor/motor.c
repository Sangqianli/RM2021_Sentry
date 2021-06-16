/**
 * @file        motor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        11-September-2020
 * @brief       Chassis Motor(RM3508).
 */
 
/* Includes ------------------------------------------------------------------*/
#include "motor.h"

#include "can_potocol.h"
#include "rp_math.h"

extern void motor_update(motor_t *motor, uint8_t *rxBuf);
extern void motor_init(motor_t *motor);

extern void leader_update(leader_t *leader, uint8_t *rxBuf);
extern void leader_init(leader_t *leader);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void motor_check(motor_t *motor);
static void motor_heart_beat(motor_t *motor);

static void leader_heart_beat(leader_t *leader);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 电机驱动
drv_can_t		motor_driver[] = {
	[CHASSIS] = {
		.type = DRV_CAN2,
		.can_id = CHASSIS_CAN_ID,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[DIAL] = {
		.type = DRV_CAN2,
		.can_id =  DIAL_CAN_ID,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[GIMBAL_PITCH] = {
		.type = DRV_CAN2,
		.can_id =  GIMBAL_CAN_ID_PITCH,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
	[GIMBAL_YAW] = {
		.type = DRV_CAN2,
		.can_id = GIMBAL_CAN_ID_YAW,
//		.std_id = RM3508_GetStdId,
//		.drv_id = RM3508_GetDrvId,
		.tx_data = CAN_SendSingleData,
	},
};

// 电机信息
motor_info_t 	motor_info[] = {
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
	{
		.offline_max_cnt = 50,
	},
};

// 底盘电机传感器
motor_t	   motor[] = {
	[CHASSIS] = {
		.info = &motor_info[CHASSIS],
		.driver = &motor_driver[CHASSIS],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS,
	},
	[DIAL] = {
		.info = &motor_info[DIAL],
		.driver = &motor_driver[DIAL],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_DIAL,
	},
	[GIMBAL_PITCH] = {
		.info = &motor_info[GIMBAL_PITCH],
		.driver = &motor_driver[GIMBAL_PITCH],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_PITCH,
	},
	[GIMBAL_YAW] = {
		.info = &motor_info[GIMBAL_YAW],
		.driver = &motor_driver[GIMBAL_YAW],
		.init = motor_init,
		.update = motor_update,
		.check = motor_check,
		.heart_beat = motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_GIMBAL_YAW,
	},
};

leader_info_t 	leader_info = {
	.offline_max_cnt = 50,
};

leader_t leader_sensor = {
	.info = &leader_info,
	.init = leader_init,
	.update = leader_update,
	.heart_beat = leader_heart_beat,
	.work_state = DEV_OFFLINE,
};

/* Private functions ---------------------------------------------------------*/
static void motor_check(motor_t *motor)
{
	int16_t err;
	motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8191 + err;
//		    round--;
		/* 8191↑ -> 0 */
		else
//			round++;
			motor_info->angle_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

static void motor_heart_beat(motor_t *motor)
{
	motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;      //正常连接时会在can中断中清零
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
		motor->info->init_flag = false;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

static void leader_heart_beat(leader_t *leader)
{
	leader_info_t *leader_info = leader->info;
	
	leader_info->offline_cnt++;      //正常连接时会在can中断中清零
	if(leader_info->offline_cnt > leader_info->offline_max_cnt) {
		leader_info->offline_cnt = leader_info->offline_max_cnt;
		leader->work_state = DEV_OFFLINE;
		leader->info->init_flag = false;
	}
	else {
		if(leader->work_state == DEV_OFFLINE)
			leader->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/

