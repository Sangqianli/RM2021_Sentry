/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"

#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define Normal_Speed -2000
#define First_Speed  -1000
#define Swerve_Ratio 0.1
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
Chassis_t Chassis_process;
/* Private functions ---------------------------------------------------------*/
static void Cruise_First()
{
    static uint8_t step=0;	/*初始化步骤*/
    if(Chassis_process.init_flag == false)		/*未对里程完成初始化*/
    {
        switch(step)
        {
        case 0:			/*初始启动先向左边靠*/
        {
            if(path_sensor.info->left_touch)		/*左边的点触开关*/
            {
                Chassis_process.Speed_taget=0;
                step=1;
            }
            else	/*未识别到 -- 向左运动*/
            {
                Chassis_process.Speed_taget = -1000;	/*向左运动*/
            }
            break;
        }
        case 1:
        {
            if(path_sensor.info->left_touch)
            {
                Chassis_process.Speed_taget=100;	/*向右微调，调节到刚好不触发点触开关的位置*/
            }
            else			/*微调完成后先停住，进入下一个调节阶段*/
            {
                Chassis_process.Speed_taget=0;
                path_sensor.info->mileage_total=0;	/*清除里程数 -- 开始记录*/
                step=2;
            }
            break;
        }
        case 2:
        {
            if(path_sensor.info->right_touch)	/*已经识别到右轨道*/
            {
                Chassis_process.Speed_taget=0;
                step=3;
            }
            else 		/*未识别到*/
            {
                Chassis_process.Speed_taget=1000;			/*向右运动*/
            }
            break;
        }
        case 3:
        {
            if(path_sensor.info->right_touch)
            {
                Chassis_process.Speed_taget=-100;		/*已经识别到右点触开关，现在微调到刚好未识别到的状态*/
            }
            else			/*微调完成*/
            {
                Chassis_process.Speed_taget=0;															/*停止运动*/
                Chassis_process.Mileage_atrip=path_sensor.info->mileage_total;	/*记录总里程数*/
                step=0;																							/*复位step*/
                Chassis_process.init_flag=true;											/*更新标志位*/
                Chassis_process.Derection_flag=1;//向左运动标志位
            }
            break;
        }
        }
    }
}
static void Cruise_Normal()
{
    if(Chassis_process.Derection_flag == 1)
    {
        if(path_sensor.info->left_touch)//向左运动时碰到点触开关
        {
            Chassis_process.swerve_judge = true;//开始反弹判断
        }

        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->left_touch == false)//反弹至点触开关释放
            {
                Chassis_process.Derection_flag = -1;//向右跑轨
                path_sensor.info->mileage_total = 0;//清空里程数
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//反弹完成
            }
        }
    }//左

    if(Chassis_process.Derection_flag == -1)
    {
        if(path_sensor.info->right_touch)//向右运动时碰到点触开关
        {
            Chassis_process.swerve_judge = true;//开始反弹判断
        }
        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->right_touch == false)//反弹至点触开关释放
            {
                Chassis_process.Derection_flag = 1;//向左跑轨
                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//反弹完成
            }
        }
    }//右
}

/*进入能量回收范围的处理*/
static void Chassis_Rebound()
{
    if( ( (Chassis_process.Mileage_atrip-path_sensor.info->mileage_now)<=(Chassis_process.Mileage_atrip*Swerve_Ratio) )  ||
            ( (path_sensor.info->mileage_now<(Chassis_process.Mileage_atrip*Swerve_Ratio)) ) )
    {
        Chassis_process.swerve_flag = true;//反弹流程开始
    }
    if(Chassis_process.swerve_flag)
        Chassis_process.PVM.out = 0;
    else//没有反弹，正常巡航
    {
        Chassis_process.Speed_taget=Chassis_process.Derection_flag*Normal_Speed;
    }
}

static void Chassis_RCcontrol()
{
    Chassis_process.Speed_taget = rc_sensor.info->ch2*Chassis_process.rotate_ratio;
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    NormalData_0x200[0] = Chassis_process.PVM.out;
}

static void Chassis_AUTOcontrol()
{
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
        Chassis_process.PVM.out = 0;
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/* Exported functions --------------------------------------------------------*/
void Chassis_Init()
{
    Chassis_process.PVM.kp = 18;
    Chassis_process.PVM.ki = 0.11;
    Chassis_process.PVM.kd = 0;
    Chassis_process.PVM.integral_max = 8000;
    Chassis_process.PVM.out_max = 12000;
    Chassis_process.init_flag = false;
    Chassis_process.Derection_flag = 1;//初始向左
    Chassis_process.rotate_ratio = 8;
}
void StartChassisTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL)
        {
            if(sys.remote_mode == RC)
            {
                Chassis_RCcontrol();
            }
            else if(sys.remote_mode == AUTO)
            {
                Cruise_First();
                if(Chassis_process.init_flag)
                {
                    Cruise_Normal();
                    Chassis_Rebound();
                }
                Chassis_AUTOcontrol();
            }
        }
        osDelay(2);
    }
}


