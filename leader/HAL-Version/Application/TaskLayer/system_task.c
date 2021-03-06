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
#include "device.h"

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
static void Data_clear()
{
    sys.fire_state.FRICTION_OPEN = false;
    sys.fire_state.FIRE_OPEN = false;
    sys.predict_state.PREDICT_OPEN = false;
    sys.auto_mode = AUTO_MODE_SCOUT;

    pid_clear(&Gimbal_process.YAW_PPM);
    pid_clear(&Gimbal_process.YAW_PVM);
    pid_clear(&Gimbal_process.PITCH_PPM);
    pid_clear(&Gimbal_process.PITCH_PVM);

    motor[GIMBAL_YAW].info->angle_sum = 0;
    motor[GIMBAL_PITCH].info->angle_sum = 0;

    Gimbal_process.Yaw_taget = 0;
    Gimbal_process.Pitch_taget = 0;

    Chassis_process.Trip_times = 0;
}
/**
 *	@brief	通过遥控器更新系统信息(非正常状态下重置遥控信息)
 */
static void rc_update_info(void)
{
    if(sys.state != SYS_STATE_NORMAL) {

    }
    else {
        if(rc_sensor.work_state == DEV_ONLINE)
        {

            if (rc_sensor.info->s2 == RC_SW_MID)
            {
                sys.remote_mode = RC;
            }
            else if(rc_sensor.info->s2 == RC_SW_UP)
            {
                sys.remote_mode = AUTO;
            } else if (rc_sensor.info->s2 == RC_SW_DOWN)
            {
                sys.remote_mode = INSPECTION;
            }
        } else if(master_sensor.info->modes.online == 1)
        {
            if(master_sensor.info->modes.mode == RC_SW_MID)
            {
                sys.remote_mode = RC;
            }
            else if(master_sensor.info->modes.mode == RC_SW_UP)
            {
                sys.remote_mode = AUTO;
            } else if(master_sensor.info->modes.mode == RC_SW_DOWN)
            {
                sys.remote_mode = INSPECTION;
            }
        }
    }
}

/**
 *	@brief	根据遥控器切换控制方式
 */
static void system_ctrl_mode_switch(void)
{
    if( (rc_sensor.info->s2_switch_uptomid)||(rc_sensor.info->s2_siwtch_up)||(rc_sensor.info->s2_switch_downtomid)||(rc_sensor.info->s2_siwtch_down)||(master_sensor.info->mode_siwtch) )
    {
        sys.switch_state.REMOTE_SWITCH = true;
        sys.switch_state.RESET_CAL = true;
        sys.switch_state.ALL_READY = false;
        rc_sensor.info->s2_switch_uptomid = false;
        rc_sensor.info->s2_siwtch_up = false;
        rc_sensor.info->s2_switch_downtomid = false;
        rc_sensor.info->s2_siwtch_down = false;

        master_sensor.info->mode_siwtch = false;

        Data_clear();
    }
}


static void system_state_machine(void)
{
    if( (sys.switch_state.REMOTE_SWITCH == false)&&(sys.switch_state.SYS_RESET == false) )
        sys.switch_state.ALL_READY = true;
    if(sys.switch_state.ALL_READY)//系统正常且复位完成后允许切换
    {
        system_ctrl_mode_switch();
    }
}

/* Exported functions --------------------------------------------------------*/
void Application_Init()  //任务层初始化
{
    Chassis_Init();
    Gimbal_Init();
    Fire_Init();
    Vision_Init();
}
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
        if( (rc_sensor.work_state == DEV_OFFLINE)&&(master_sensor.info->modes.online == 0) )
        {
            sys.state = SYS_STATE_RCLOST;
            RC_ResetData(&rc_sensor);
            Data_clear();//清除任务信息
        }
        /* 遥控在线 */
        else if( (rc_sensor.work_state == DEV_ONLINE)||(master_sensor.info->modes.online == 1) )
        {
            /* 遥控正常 */
            if( (rc_sensor.errno == NONE_ERR)||(master_sensor.info->modes.mode != 0) )
            {
                /* 失联恢复 */
                if(sys.state == SYS_STATE_RCLOST)
                {
                    // 可在此处同步云台复位标志位
                    // 系统参数复位
                    sys.switch_state.SYS_RESET = true;//失联复位标志位
                    sys.switch_state.RESET_CAL = true;
                    sys.switch_state.ALL_READY = false;//未复位好
                    sys.remote_mode = RC;
//					sys.state = SYS_STATE_NORMAL;
                }
                sys.state = SYS_STATE_NORMAL;
                // 可在此处等待云台复位后才允许切换状态
                system_state_machine();
            }
            /* 遥控错误 */
            else if( (rc_sensor.errno == DEV_DATA_ERR)||(master_sensor.info->modes.mode == 0) ) {
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
