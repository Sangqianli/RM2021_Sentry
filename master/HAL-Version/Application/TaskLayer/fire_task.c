/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "fire_task.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define FIRING_RATE_LOW   345
#define FIRING_RATE_MID   500
#define FIRING_RATE_HIGH  685    /*射速*/

#define SHOOT_FREQ_LOW    1080   /*4射频*/
#define SHOOT_FREQ_MID    2160   /*8射频*/
#define SHOOT_FREQ_HIGH   4320   /*16射频*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
Fire_t Fire_process = {
	.Stuck_flag = false,
	.Speed_target = SHOOT_FREQ_LOW	
	};
/* Private functions ---------------------------------------------------------*/
static void Friction_Control()
{
	if(sys.fire_state.FRICTION_OPEN)
	{
		Fire_process.Friction_target += 5;
		if(Fire_process.Friction_target >= FIRING_RATE_HIGH)
			Fire_process.Friction_target = FIRING_RATE_HIGH;
	}else
	{
		Fire_process.Friction_target -= 5;		
		if(Fire_process.Friction_target <= 0)
			Fire_process.Friction_target = 0;		
	}
	NormalPwm[0] = Fire_process.Friction_target;
	NormalPwm[1] = Fire_process.Friction_target;
}


void Dial_Remote_An()
{
    static int16_t stuck_cnt=0;
    static int16_t reverse_cnt=0;	
	if(sys.fire_state.FIRE_OPEN)
	{
		Fire_process.PPM.target = motor[DIAL].info->angle_sum + 36959.5f;
		sys.fire_state.FIRE_OPEN = false;
	}
	if( (abs(Fire_process.PPM.err)>20000) && (reverse_cnt == 0) )
	{
		stuck_cnt++;
		if(stuck_cnt>120)//5ms任务
		{
			Fire_process.Stuck_flag = true;
			stuck_cnt = 0;
		}
	}
    if(Fire_process.Stuck_flag)	
	{
		Fire_process.Speed_target = -400;
		reverse_cnt++;
	    Fire_process.PVM.target = Fire_process.Speed_target;
	    Fire_process.PVM.measure = motor[DIAL].info->speed;		
		if(reverse_cnt>100)
		{
			Fire_process.Stuck_flag = false;
			reverse_cnt = 0;
		}
	}
	
	if(Fire_process.Stuck_flag == false)
	{
        Fire_process.PPM.measure = motor[DIAL].info->angle_sum;	
        pid_calculate(&Fire_process.PPM);	
	    Fire_process.PVM.target = Fire_process.PPM.out;
	    Fire_process.PVM.measure = motor[DIAL].info->speed;
	}

	pid_calculate(&Fire_process.PVM);
}

static void  Dial_Remote_Continue()
{
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;
	if( sys.fire_state.FIRE_OPEN == true && (Fire_process.Stuck_flag == false) )	
	{
		Fire_process.Speed_target = SHOOT_FREQ_LOW;
	}
	if(sys.fire_state.FIRE_OPEN == false)	
	{
		Fire_process.Speed_target = 0;
		stuck_cnt=0;
		reverse_cnt=0;
	}	
	if( (abs(Fire_process.PVM.err)>1000) && (reverse_cnt == 0) )
	{
		stuck_cnt++;
		if(stuck_cnt>120)//5ms任务
		{
			Fire_process.Stuck_flag = true;
			stuck_cnt = 0;
		}
	}
    if(Fire_process.Stuck_flag)	
	{
		Fire_process.Speed_target = -400;
		reverse_cnt++;
		if(reverse_cnt>100)
		{
			Fire_process.Stuck_flag = false;
			reverse_cnt = 0;
		}
	}
	Fire_process.PVM.target = Fire_process.Speed_target;
	Fire_process.PVM.measure = motor[DIAL].info->speed;			
	
	pid_calculate(&Fire_process.PVM);	
}
/**
* @brief 开火判断决策
* @param void
* @return void
* 对开火的条件判断
*/
static void Fire_Judge()
{
	
	
	
}

void Dial_Auto()
{
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;
	
	Fire_Judge();
	
	if( sys.fire_state.FIRE_OPEN == true && (Fire_process.Stuck_flag == false) )	
	{
		Fire_process.Speed_target = SHOOT_FREQ_LOW;
	}
	if(sys.fire_state.FIRE_OPEN == false)	
	{
		Fire_process.Speed_target = 0;
		stuck_cnt=0;
		reverse_cnt=0;
	}	
	if( (abs(Fire_process.PVM.err)>1000) && (reverse_cnt == 0) )
	{
		stuck_cnt++;
		if(stuck_cnt>120)//5ms任务
		{
			Fire_process.Stuck_flag = true;
			stuck_cnt = 0;
		}
	}
    if(Fire_process.Stuck_flag)	
	{
		Fire_process.Speed_target = -400;
		reverse_cnt++;
		if(reverse_cnt>100)
		{
			Fire_process.Stuck_flag = false;
			reverse_cnt = 0;
		}
	}
	Fire_process.PVM.target = Fire_process.Speed_target;
	Fire_process.PVM.measure = motor[DIAL].info->speed;			
	
	pid_calculate(&Fire_process.PVM);
}

static void Dail_text()
{
    NormalData_0x200[1] = (int16_t)Fire_process.PVM.out;	
}
/* Exported functions --------------------------------------------------------*/
void Fire_Init()
{
    Fire_process.PPM.target=0;
    Fire_process.PPM.kp=0.1;
    Fire_process.PPM.ki=0;
    Fire_process.PPM.kd=0;
    Fire_process.PPM.integral_max=3000;
    Fire_process.PPM.out_max=8000;
    Fire_process.PPM.out=0;//拨盘位置环

    Fire_process.PVM.target=0;
    Fire_process.PVM.kp=8;//
    Fire_process.PVM.ki=0.02;//
    Fire_process.PVM.kd=0;
    Fire_process.PVM.integral_max=6000;
    Fire_process.PVM.out_max=8000;
    Fire_process.PVM.out=0;//yaw速度环	
}
void StartFireTask(void const * argument)
{
	if( (sys.state == SYS_STATE_NORMAL) && (sys.switch_state.ALL_READY) )
	{
		Friction_Control();
		if(sys.fire_state.FRICTION_OPEN)
		{
			if(sys.remote_mode == RC)
			{
//				Dial_Remote_An();
				Dial_Remote_Continue();
			}
			else if(sys.remote_mode == AUTO)
			{
				 Dial_Auto();
			}
			Dail_text();
		}			
	}	
	osDelay(5);
}
