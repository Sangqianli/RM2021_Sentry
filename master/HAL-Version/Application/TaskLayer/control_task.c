/**
 * @file        control_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"

#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	¿ØÖÆÈÎÎñ
 */
void StartControlTask(void const * argument)
{
	//module.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) {
			//module.ctrl()
		} else {
			//module.self_protect();
		}
		
		osDelay(2);
	}
}
