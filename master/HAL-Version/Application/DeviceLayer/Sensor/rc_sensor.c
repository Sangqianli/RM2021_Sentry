/**
 * @file        rc_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Device Rc.
 */

/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"

#include "rp_math.h"
#include "device.h"
#include "rc_potocol.h"

extern void rc_sensor_init(rc_sensor_t *rc_sen);
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 遥控器驱动
drv_uart_t	rc_sensor_driver = {
    .type = DRV_UART2,
    .tx_byte = NULL,
};

// 遥控器信息
rc_sensor_info_t 	rc_sensor_info = {
    .offline_max_cnt = 60,
};

// 遥控器传感器
rc_sensor_t	rc_sensor = {
    .info = &rc_sensor_info,
    .init = rc_sensor_init,
    .update = rc_sensor_update,
    .check = rc_sensor_check,
    .heart_beat = rc_sensor_heart_beat,
    .work_state = DEV_OFFLINE,
    .id = DEV_ID_RC,
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	遥控器数据检查
 */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;
    if(!rc_info->init_flag)
    {
        rc_info->init_flag = true;
        rc_info->s1_pre=rc_info->s1;
        rc_info->s2_pre=rc_info->s2;
    }

    if(abs(rc_info->ch0) > 660 ||
            abs(rc_info->ch1) > 660 ||
            abs(rc_info->ch2) > 660 ||
            abs(rc_info->ch3) > 660)
    {
        rc_sen->errno = DEV_DATA_ERR;
        rc_info->ch0 = 0;
        rc_info->ch1 = 0;
        rc_info->ch2 = 0;
        rc_info->ch3 = 0;
        rc_info->s1 = RC_SW_MID;
        rc_info->s2 = RC_SW_MID;
        rc_info->thumbwheel = 0;
    }
    else
    {
        rc_sen->errno = NONE_ERR;
    }
    /*用完记得清0*/
    if(rc_info->s1_pre!=rc_info->s1)//s1切换记录
    {
        switch (rc_info->s1)
        {
        case RC_SW_UP:
            if(rc_info->s1_pre==RC_SW_MID)
                rc_info->s1_siwtch_up=true;
            break;
        case RC_SW_MID:
            if(rc_info->s1_pre==RC_SW_UP)
                rc_info->s1_switch_uptomid=true;
            if(rc_info->s1_pre==RC_SW_DOWN)
                rc_info->s1_switch_downtomid=true;
            break;
        case RC_SW_DOWN:
            if(rc_info->s1_pre==RC_SW_MID)
                rc_info->s1_siwtch_down=true;
            break;
        default:
            rc_sen->errno = DEV_DATA_ERR;
            break;
        }
    }
    if(rc_info->s2_pre!=rc_info->s2)//s2切换记录
    {
        switch (rc_info->s2)
        {
        case RC_SW_UP:
            if(rc_info->s2_pre==RC_SW_MID)
                rc_info->s2_siwtch_up=true;
            break;
        case RC_SW_MID:
            if(rc_info->s2_pre==RC_SW_UP)
                rc_info->s2_switch_uptomid=true;
            if(rc_info->s2_pre==RC_SW_DOWN)
                rc_info->s2_switch_downtomid=true;
            break;
        case RC_SW_DOWN:
            if(rc_info->s2_pre==RC_SW_MID)
                rc_info->s2_siwtch_down=true;
            break;
        default:
            rc_sen->errno = DEV_DATA_ERR;
            break;
        }
    }
    /****************/
    rc_info->s1_pre=rc_info->s1;
    rc_info->s2_pre=rc_info->s2;	//记录上次的值
}

/**
 *	@brief	遥控器心跳包
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
    rc_sensor_info_t *rc_info = rc_sen->info;

    rc_info->offline_cnt++;
    if(rc_info->offline_cnt > rc_info->offline_max_cnt) {
        rc_info->offline_cnt = rc_info->offline_max_cnt;
        rc_sen->work_state = DEV_OFFLINE;
        rc_info->init_flag = false;//需要重新初始化
    }
    else {
        /* 离线->在线 */
        if(rc_sen->work_state == DEV_OFFLINE)
            rc_sen->work_state = DEV_ONLINE;
    }
}

/* Exported functions --------------------------------------------------------*/
bool RC_IsChannelReset(void)
{
    if(  (DeathZoom(rc_sensor_info.ch0, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch1, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch2, 0, 50) == 0) &&
            (DeathZoom(rc_sensor_info.ch3, 0, 50) == 0)   )
    {
        return true;
    }
    return false;
}

void RC_ResetData(rc_sensor_t *rc)
{
    // 通道值强行设置成中间值(不拨动摇杆的状态)
    rc->info->ch0 = 0;
    rc->info->ch1 = 0;
    rc->info->ch2 = 0;
    rc->info->ch3 = 0;
    // 左右开关选择强行设置成中间值状态
    rc->info->s1 = RC_SW_MID;
    rc->info->s2 = RC_SW_MID;
    // 鼠标
    rc->info->mouse_vx = 0;
    rc->info->mouse_vy = 0;
    rc->info->mouse_vz = 0;
    rc->info->mouse_btn_l = 0;
    rc->info->mouse_btn_r = 0;
    // 键盘
    rc->info->key_v = 0;
    // 左拨轮
    rc->info->thumbwheel = 0;
}
