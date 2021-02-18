/**
 * @file        path_sensor.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        18-February-2021
 * @brief       About the Pathway.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "path_sensor.h"
#include "drv_io.h"
#include "rp_math.h"
#include "device.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void path_update(path_t *path);
static void path_init(path_t *path);
static void path_check(path_t *path);
static void path_heart_beat(path_t *path);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
drv_path_t  path_driver = {
	.type = DRV_ENCODER_AND_IOIN,
};

path_info_t  path_info = {
	.offline_max_cnt = 500,
};

path_t 	path_sensor = {
	.info = &path_info,
	.driver = &path_driver,
	.init = path_init,
	.update = path_update,
	.check = path_check,
	.heart_beat = path_heart_beat,
	.work_state = DEV_ONLINE,
	.id = DEV_ID_ENCODER_AND_TOUCH,	
};
/* Private functions ---------------------------------------------------------*/
static void path_init(path_t *path)
{
	path->info->mileage_now=0;
	path->info->mileage_prev=0;
	path->info->mileage_total=0;
	path->info->left_touch=0;
	path->info->right_touch=0;
	path->info->offline_cnt=0;
}

static void path_update(path_t *path)
{	
	path_info_t *path_info = path->info;
    static uint8_t delay_left=0;
    static uint8_t delay_right=0;	
	if(LEFT_PES)
		delay_left++;
	else
	{
		path_info->left_touch=false;
		delay_left=0;
	}
	if(RIGHT_PES)
		delay_right++;
	else
	{
		path_info->right_touch=false;
		delay_right=0;		
	}

	if(delay_left>80)
	{
		path_info->left_touch=true;
		delay_left=0;
	}
	if(delay_right>80)
	{
		path_info->right_touch=true;
		delay_right=0;
	}

    path_info->mileage_now=(int16_t)TIM1->CNT;
    path_info->mileage_prev=path_info->mileage_now;	
	TIM1->CNT=0;
}
static void path_check(path_t *path)
{
	path_info_t *path_info = path->info;
	if(path_info->mileage_now>2000)
		path_info->mileage_now-=4000;
	path_info->mileage_total+=path_info->mileage_now;
}

static void path_heart_beat(path_t *path)
{
	
}
/* Exported functions --------------------------------------------------------*/


