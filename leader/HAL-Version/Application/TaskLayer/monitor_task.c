/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"

#include "drv_io.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	chassis_motor[CHAS_LF].heart_beat(&chassis_motor[CHAS_LF]);
	chassis_motor[CHAS_RF].heart_beat(&chassis_motor[CHAS_RF]);
	chassis_motor[CHAS_LB].heart_beat(&chassis_motor[CHAS_LB]);
	chassis_motor[CHAS_RB].heart_beat(&chassis_motor[CHAS_RB]);
}

static void system_led_flash(void)
{
	static uint16_t led_blue_flash = 0;
	
	led_blue_flash++;
	if(led_blue_flash > 500) 
	{
		led_blue_flash = 0;
		LED_BLUE_TOGGLE();
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统监控任务
 */
void StartMonitorTask(void const * argument)
{
	//LED_RED_ON();
	for(;;)
	{
		system_led_flash();
		device_heart_beat();
		imu_sensor.update(&imu_sensor);
		osDelay(1);
	}
}
