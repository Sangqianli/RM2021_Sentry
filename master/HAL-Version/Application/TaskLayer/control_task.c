/**
 * @file        control_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "driver.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t StopData_0x200[4] = {0,0,0,0};
int16_t StopData_0x2FF[4] = {0,0,0,0};
int16_t StopPwm[2] = {0,0};
/* Exported variables --------------------------------------------------------*/
int16_t NormalData_0x200[4]= {0,0,0,0};
int16_t NormalData_0x2FF[4]= {0,0,0,0};
int16_t NormalPwm[2] = {0,0};
/* Private functions ---------------------------------------------------------*/
//static void Control_Init()
//{
//
//
//}

static void Control_Stop()
{
    CAN2_SendDataBuff(0x200,StopData_0x200);
    CAN2_SendDataBuff(0x2ff,StopData_0x2FF);
//    CAN2_SendDataBuff(0x1ff,StopData_0x2FF);	
    FRICTION_PwmOut(0, 0);
    LED_RED_ON();
    LED_GREEN_OFF();
    LASER_OFF();
}

static void Control_Normal()
{
	static int16_t cnt = 0;
	if(cnt > 2)
	{
		CAN2_SendDataBuff(0x2ff,NormalData_0x2FF);
         CAN2_SendDataBuff(0x200,NormalData_0x200);
		 cnt = 0;
	}else if(cnt <= 2)
	{
         CAN2_SendDataBuff(0x2ff,NormalData_0x2FF);
	}
	cnt ++;
//	CAN2_SendDataBuff(0x2ff,NormalData_0x2FF);
//	CAN2_SendDataBuff(0x200,NormalData_0x200);

//	CAN2_SendDataBuff(0x1ff,NormalData_0x2FF);
//    CAN2_SendDataBuff(0x200,StopData_0x200);
//    CAN2_SendDataBuff(0x2ff,StopData_0x2FF);	
    FRICTION_PwmOut(NormalPwm[0], NormalPwm[1]);
    LED_RED_OFF();
    LED_GREEN_ON();
    LASER_ON();
}
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	¿ØÖÆÈÎÎñ
 */
void StartControlTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL) {
            Control_Normal();
        } else {
            Control_Stop();
        }
        osDelay(1);
    }
}
