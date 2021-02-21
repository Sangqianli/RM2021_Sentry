/**
 * @file        system_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        27-October-2020
 * @brief       Decision Center.
 */

/* Includes ------------------------------------------------------------------*/
#include "system_task.h"

#include "cmsis_os.h"
#include "rc_sensor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
//flag_t flag = {
//	.gimbal = {
//		.reset_start = false,
//		.reset_ok = false,
//	},
//};

system_t sys = {
	.remote_mode = RC,
	.state = SYS_STATE_RCLOST,
	.auto_mode = AUTO_MODE_SCOUT,
	.switch_state.SYS_RESET = false,
	.switch_state.REMOTE_SWITCH = false,
    .switch_state.AUTO_MODE_SWITCH = false,
	.switch_state.ALL_READY	= false,
	.fire_state.FRICTION_OPEN = false,
	.fire_state.FIRE_OPEN = false,	
	.predict_state.PREDICT_OPEN = false,
	.predict_state.PREDICT_ACTION = false,
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	通过遥控器更新系统信息(非正常状态下重置遥控信息)
 */
static void rc_update_info(void)
{
	if(sys.state != SYS_STATE_NORMAL) {
			
	}
	else {
		if( (rc_sensor.info->s2 == RC_SW_MID)||(rc_sensor.info->s2 == RC_SW_DOWN) )
		{
			sys.remote_mode = RC;
		}
		else if(rc_sensor.info->s2 == RC_SW_UP)
		{
			sys.remote_mode = AUTO;
		}
	}
}

/**
 *	@brief	根据遥控器切换控制方式
 */
static void system_ctrl_mode_switch(void)
{
	if( (rc_sensor.info->s2_switch_uptomid)||(rc_sensor.info->s2_siwtch_up) )
	{
		sys.switch_state.REMOTE_SWITCH = true;
	}
		
	if(rc_sensor.info->s1_siwtch_up)
	{
		if(sys.fire_state.FRICTION_OPEN)
			sys.fire_state.FRICTION_OPEN = false;
		else 
			sys.fire_state.FRICTION_OPEN = true;	
		rc_sensor.info->s1_siwtch_up = false;
	}
	if(rc_sensor.info->s2_siwtch_down)
	{
		if(sys.fire_state.FIRE_OPEN)
			sys.fire_state.FIRE_OPEN = false;
		else
			sys.fire_state.FIRE_OPEN = true;
		rc_sensor.info->s2_siwtch_down = false;
	}	
}


static void system_state_machine(void)
{
	if(sys.switch_state.ALL_READY)//系统正常且复位完成后允许切换
	{
	    system_ctrl_mode_switch();
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统决策任务
 */
void StartSystemTask(void const * argument)
{
	for(;;)
	{
		portENTER_CRITICAL();
		
		// 更新遥控信息
		rc_update_info();
		
		/* 遥控离线 */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 
		/* 遥控在线 */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* 遥控正常 */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* 失联恢复 */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// 可在此处同步云台复位标志位					
					// 系统参数复位
					sys.switch_state.SYS_RESET = true;//失联复位标志位
					sys.switch_state.ALL_READY = false;//未复位好
					sys.remote_mode = RC;
//					sys.state = SYS_STATE_NORMAL;
				}
				sys.state = SYS_STATE_NORMAL;
				// 可在此处等待云台复位后才允许切换状态
				system_state_machine();
			}
			/* 遥控错误 */
			else if(rc_sensor.errno == DEV_DATA_ERR) {
				sys.state = SYS_STATE_RCERR;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			} else {
				sys.state = SYS_STATE_WRONG;
				//reset CPU
				__set_FAULTMASK(1);
				NVIC_SystemReset();
			}
		}
		
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
