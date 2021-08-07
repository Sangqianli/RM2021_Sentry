/**
 * @file        judge_sensor.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        18-February-2021
 * @brief       About the Pathway.
 */

/* Includes ------------------------------------------------------------------*/
#include "judge_sensor.h"
#include "rp_math.h"
#include "device.h"
#include "judge_potocol.h"

extern void judge_update( uint8_t *rxBuf);
extern void judge_init(judge_sensor_t *judge);
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void judge_check(judge_sensor_t *judge);
static void judge_heart_beat(judge_sensor_t *judge);
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 裁判系统驱动
drv_uart_t	judge_sensor_driver = {
    .type = DRV_UART5,
    .tx_byte =  UART_SendData,
};

// 裁判系统信息
judge_info_t 	judge_sensor_info = {
    .offline_max_cnt = 200,
};

// 裁判系统传感器
judge_sensor_t	judge_sensor = {
    .info = &judge_sensor_info,
    .init = judge_init,
    .update = judge_update,
    .check = judge_check,
    .heart_beat = judge_heart_beat,
    .work_state = DEV_OFFLINE,
    .id = DEV_ID_JUDJE	,
};
/* Private functions ---------------------------------------------------------*/
static void judge_check(judge_sensor_t *judge_sen)
{
    judge_info_t *judge_info = judge_sen->info;
    if(judge_sen->work_state == DEV_OFFLINE)
    {
        judge_info->power_heat_update = false;
    }

}

static void judge_heart_beat(judge_sensor_t *judge_sen)
{
    judge_info_t *judge_info = judge_sen->info;

    judge_info->offline_cnt++;
    if(judge_info->offline_cnt > judge_info->offline_max_cnt) {
        judge_info->offline_cnt = judge_info->offline_max_cnt;
        judge_sen->work_state = DEV_OFFLINE;
    }
    else {
        /* 离线->在线 */
        if(judge_sen->work_state == DEV_OFFLINE)
            judge_sen->work_state = DEV_ONLINE;
    }
}
/* Exported functions --------------------------------------------------------*/
