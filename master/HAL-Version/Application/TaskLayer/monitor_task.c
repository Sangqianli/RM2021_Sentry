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
	motor[CHASSIS].heart_beat(&motor[CHASSIS]);
	motor[DIAL].heart_beat(&motor[DIAL]);
	motor[GIMBAL_PITCH].heart_beat(&motor[GIMBAL_PITCH]);
	motor[GIMBAL_YAW].heart_beat(&motor[GIMBAL_YAW]);
	vision_sensor.heart_beat(&vision_sensor);
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

static void device_get(void)
{
	imu_sensor.update(&imu_sensor);
	path_sensor.update(&path_sensor);
	path_sensor.check(&path_sensor);
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
		device_get();
		osDelay(1);
	}
}
