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
int16_t StopData_0x1FF[4] = {0,0,0,0};
int16_t StopPwm[2] = {0,0};
/* Exported variables --------------------------------------------------------*/
int16_t NormalData_0x200[4]= {0,0,0,0};
int16_t NormalData_0x2FF[4]= {0,0,0,0};
int16_t NormalData_0x1FF[4]= {0,0,0,0};
int16_t NormalData_Can1_0x1FF[4]= {0,0,0,0};
int16_t NormalPwm[2] = {0,0};
/* Private functions ---------------------------------------------------------*/
int16_t Leader_Data = 0;
static void Control_Leader()
{
    if( (sys.auto_mode == AUTO_MODE_ATTACK) && (Vision_process.data_kal.DistanceGet_KF <= 4.f) )
    {
        Leader_Data = RP_SET_BIT( Leader_Data,1);
    } else
    {
        Leader_Data = RP_CLEAR_BIT( Leader_Data,1);
    }

    if(vision_sensor.info->RxPacket.RxData.identify_hero == 1)
    {
        Leader_Data = RP_SET_BIT( Leader_Data,2);
    } else
    {
        Leader_Data = RP_CLEAR_BIT( Leader_Data,2);
    }
    NormalData_0x1FF[3] = Leader_Data;

}
static void Control_Stop()
{
//    CAN2_SendDataBuff(0x1ff,StopData_0x1FF);
//    CAN1_SendDataBuff(0x1ff,StopData_0x1FF);	
    CAN2_SendDataBuff(0x1ff,StopData_0x1FF);
    CAN1_SendDataBuff(0x1ff,StopData_0x1FF);		
    FRICTION_PwmOut(0, 0);
    LED_RED_ON();
    LED_GREEN_OFF();
    LASER_OFF();

    Leader_Data = 0;
}

static void Control_Normal()
{
    CAN2_SendDataBuff(0x1ff,NormalData_0x1FF);
    CAN1_SendDataBuff(0x1ff,NormalData_Can1_0x1FF);
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
            Control_Leader();
            Control_Normal();
        } else {
            Control_Stop();
        }
        osDelay(2);
    }
}
