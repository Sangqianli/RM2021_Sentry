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
#define FIRING_RATE_HIGH  450      /*27m射速630*/

#define SHOOT_FREQ_ONE    270
#define SHOOT_FREQ_LOW    1080   /*4射频*/
#define SHOOT_FREQ_MID    2160   /*8射频*/
#define SHOOT_FREQ_TWELVE 3240   /*12射频*/
#define SHOOT_FREQ_HIGH   4320   /*16射频*/
#define SHOOT_FREQ_HEATLIMIT   6480  /*24射频*/
#define SHOOT_FREQ_VERYHIGH   8640   /*32射频*/
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
    static int32_t friction_cnt = 0;
    if(sys.remote_mode == AUTO)
    {
//        if(judge_sensor.info->GameStatus.game_progress == 4) //比赛开始后再开摩擦轮
//        {
//            sys.fire_state.FRICTION_OPEN = true;
//            friction_cnt ++;
//            if( friction_cnt > 1000)
//            {
//                sys.fire_state.FRICTION_OPEN = true;
//                Fire_process.Friction_ready = true;
//            }//摩擦轮解锁2s后再解锁拨盘
//        }
//        else
//        {
//            sys.fire_state.FRICTION_OPEN = false;
//            sys.fire_state.FIRE_OPEN = false;//不在比赛流程时自动关摩擦轮和拨盘
//            friction_cnt = 0;
//            Fire_process.Friction_ready = false;
//        }
        if(master_sensor.info->modes.friction_now == 1)
        {
            sys.fire_state.FRICTION_OPEN = true;
            friction_cnt ++;
            if( friction_cnt > 2000)
            {
                sys.fire_state.FRICTION_OPEN = true;
                Fire_process.Friction_ready = true;
            }//摩擦轮解锁4           s后再解锁拨盘
        }
        else
        {
            sys.fire_state.FRICTION_OPEN = false;
            sys.fire_state.FIRE_OPEN = false;//不在比赛流程时自动关摩擦轮和拨盘
            friction_cnt = 0;
            Fire_process.Friction_ready = false;
        }
    }
    if(sys.remote_mode == RC)
    {
        friction_cnt = 0;
        Fire_process.Friction_ready = false;
    }

    if(sys.fire_state.FRICTION_OPEN)
    {
        Fire_process.Friction_target += 5;
        if(Fire_process.Friction_target >= FIRING_RATE_HIGH)
            Fire_process.Friction_target = FIRING_RATE_HIGH;
    } else
    {
        Fire_process.Friction_target -= 5;
        if(Fire_process.Friction_target <= 0)
            Fire_process.Friction_target = 0;
    }
//    if(judge_sensor.info->GameRobotStatus.mains_power_shooter_output == 1)
//    {
//        if(judge_sensor.info->ShootData.bullet_speed >29.9f)
//        {
//            NormalPwm[0] = Fire_process.Friction_target - 50;
//            NormalPwm[1] = Fire_process.Friction_target - 50;
//        }
//        else
//        {
    NormalPwm[0] = Fire_process.Friction_target;
    NormalPwm[1] = Fire_process.Friction_target;
//        }
//    } else
//    {
//        sys.fire_state.FRICTION_OPEN = false;//关闭摩擦轮输出，让重新上电时不会给大
//        sys.fire_state.FIRE_OPEN = false; //关闭拨盘输出
////        NormalPwm[0] = 0;
////        NormalPwm[1] = 0;
//    }
}


void Dial_Remote_An()
{
    static int16_t stuck_cnt=0;
    static int16_t reverse_cnt=0;
    if(sys.fire_state.FIRE_OPEN)
    {
        Fire_process.PPM.target = motor[DIAL].info->angle_sum + 36859.5f;
        sys.fire_state.FIRE_OPEN = false;
    }
    if( (abs(Fire_process.PPM.err)>40000) && (reverse_cnt == 0) )
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
//        Fire_process.Speed_target = SHOOT_FREQ_ONE;
        Fire_process.Speed_target = SHOOT_FREQ_MID;
//		Fire_process.Speed_target = SHOOT_FREQ_HIGH;
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
        if(stuck_cnt>350)//2ms任务
        {
            Fire_process.Stuck_flag = true;
            stuck_cnt = 0;
        }
    }
    if(Fire_process.Stuck_flag)
    {
        Fire_process.Speed_target = -400;
        reverse_cnt++;
        if(reverse_cnt>250)
        {
            Fire_process.Stuck_flag = false;
            reverse_cnt = 0;
        }
    }
}
/**
* @brief 开火判断决策
* @param void
* @return void
* 对开火的条件判断
*/
static void Fire_Judge()
{
    static uint16_t Fire_cnt=0;
//    static float Fly_time=0,Real_Speed=0;
    static float yaw_width;

//    Fly_time=Vision_process.data_kal.DistanceGet_KF/25 ;//30
//    Real_Speed=(1000.f/vision_sensor.info->State.rx_time_fps)*Vision_process.speed_get;

    yaw_width = abs(Vision_process.predict_angle * 2.f);//原1.2

    if(sys.fire_state.FRICTION_OPEN == false)
    {
        sys.fire_state.FIRE_OPEN = false;
    }

    if(sys.remote_mode == RC)
    {
        if(rc_sensor.work_state == DEV_ONLINE)
        {
            if(rc_sensor.info->s1_siwtch_up)
            {
                if(sys.fire_state.FRICTION_OPEN)
                    sys.fire_state.FRICTION_OPEN = false;
                else
                    sys.fire_state.FRICTION_OPEN = true;
                rc_sensor.info->s1_siwtch_up = false;
            }
            if(rc_sensor.info->s1_siwtch_down)
            {
                if(sys.fire_state.FRICTION_OPEN)
                {
                    if(sys.fire_state.FIRE_OPEN)
                        sys.fire_state.FIRE_OPEN = false;
                    else
                        sys.fire_state.FIRE_OPEN = true;
                }
                rc_sensor.info->s1_siwtch_down = false;
            }
        } else if(master_sensor.info->modes.friction_now == 1)
        {
            sys.fire_state.FRICTION_OPEN = true;
        }
        else
        {
            sys.fire_state.FRICTION_OPEN = false;
        }
    }


    if(sys.remote_mode == AUTO)
    {
        if(sys.fire_state.FRICTION_OPEN)
        {
            if( (sys.auto_mode == AUTO_MODE_ATTACK)  //还有&&(Vision_process.speed_get * Vision_process.data_kal.YawGet_KF <= 0)
                    &&( (abs(Vision_process.predict_angle)<10) || ( abs(Gimbal_process.YAW_PPM.err)<yaw_width ) )     //旧版还有这个(abs(motor[GIMBAL_YAW].info->angle_sum - Vision_process.data_kal.YawGet_KF) <= abs(Real_Speed*Fly_time))||
                    &&( (abs(Vision_process.data_kal.PitchGet_KF)<=5) || (Vision_process.gyro_anti) )
                    && ((sys.predict_state.PREDICT_OPEN) || (Vision_process.gyro_anti) )
                    && (vision_sensor.work_state == DEV_ONLINE)
                    && (Chassis_process.init_flag)
                    && (Fire_process.Friction_ready)			)
            {
                sys.fire_state.FIRE_OPEN=true;
                Fire_cnt=0;
            }
            else
            {
                Fire_cnt++;
                if(Fire_cnt>20)
                    sys.fire_state.FIRE_OPEN=false;
            }
        }
    }
}

/**
* @brief 开火判断决策
* @param void
* @return void
*2.1版本
*/
bool yawok,deadok,allok;
static bool Is_Yaw_Now()
{
    static float yaw_width, dead_width;
    yaw_width = abs(Vision_process.predict_angle  * 1.5f);
    dead_width = abs(Vision_process.predict_angle * 0.8f);//1.0
    if(abs(Vision_process.data_kal.YawGet_KF) <= yaw_width)
        yawok = true;
    else
        yawok = false;
    if(abs(Gimbal_process.YAW_PPM.err) <= dead_width)
        deadok = true;
    else
        deadok = false;

    if( (abs(Vision_process.data_kal.YawGet_KF) <= yaw_width)&&(abs(Gimbal_process.YAW_PPM.err) <= dead_width) )
//	if( (abs(Vision_process.data_kal.YawGet_KF) <= yaw_width) )
//	if(abs(Gimbal_process.YAW_PPM.err) <= dead_width)
    {
        allok = true;
        return true;
    }
    else
    {
        allok = false;
        return false;
    }
}//判断yaw轴是否进入开火范围
static void Fire_Judge2_1()
{
    static uint16_t Fire_cnt=0;
//    static float Fly_time=0,Real_Speed=0;
//    static float yaw_width;

//    Fly_time=Vision_process.data_kal.DistanceGet_KF/25 ;//30
//    Real_Speed=(1000.f/vision_sensor.info->State.rx_time_fps)*Vision_process.speed_get;

//    yaw_width = abs(Vision_process.predict_angle * 1.5f);//原1.8

    if(sys.fire_state.FRICTION_OPEN == false)
    {
        sys.fire_state.FIRE_OPEN = false;
    }

    if(sys.remote_mode == RC)
    {
        if(rc_sensor.work_state == DEV_ONLINE)
        {
            if(rc_sensor.info->s1_siwtch_up)
            {
                if(sys.fire_state.FRICTION_OPEN)
                    sys.fire_state.FRICTION_OPEN = false;
                else
                    sys.fire_state.FRICTION_OPEN = true;
                rc_sensor.info->s1_siwtch_up = false;
            }
            if(rc_sensor.info->s1_siwtch_down)
            {
                if(sys.fire_state.FRICTION_OPEN)
                {
                    if(sys.fire_state.FIRE_OPEN)
                        sys.fire_state.FIRE_OPEN = false;
                    else
                        sys.fire_state.FIRE_OPEN = true;
                }
                rc_sensor.info->s1_siwtch_down = false;
            }
        } else
        {
            if(master_sensor.info->modes.friction_now == 1)
            {
                sys.fire_state.FRICTION_OPEN = true;
            }
            else
            {
                sys.fire_state.FRICTION_OPEN = false;
            }

            if(master_sensor.info->modes.dial_now == 1)
            {
                if(sys.fire_state.FRICTION_OPEN)
                {
                    sys.fire_state.FIRE_OPEN = true;
                }
                else
                {
                    sys.fire_state.FIRE_OPEN = false;
                }
            } else
            {
                sys.fire_state.FIRE_OPEN = false;
            }
        }
    }


    if(sys.remote_mode == AUTO)
    {
        if(sys.fire_state.FRICTION_OPEN)
        {
            if( (sys.auto_mode == AUTO_MODE_ATTACK)  //还有&&(Vision_process.speed_get * Vision_process.data_kal.YawGet_KF <= 0)
//				    &&(Vision_process.speed_get * Vision_process.data_kal.YawGet_KF <= 0)//超前判断

                    &&( (abs(Vision_process.predict_angle)<10) ||  Is_Yaw_Now() )     //旧版还有这个(abs(motor[GIMBAL_YAW].info->angle_sum - Vision_process.data_kal.YawGet_KF) <= abs(Real_Speed*Fly_time))||
                    &&( (abs(Vision_process.data_kal.PitchGet_KF)<=12) || (Vision_process.gyro_anti) )
                    && ((sys.predict_state.PREDICT_OPEN) || (Vision_process.gyro_anti) )
                    && (vision_sensor.work_state == DEV_ONLINE)
                    && (Fire_process.Friction_ready)			)
            {
                sys.fire_state.FIRE_OPEN=true;
                Fire_cnt=0;
            }
            else
            {
                Fire_cnt++;
                if(Fire_cnt>40)
                {
                    sys.fire_state.FIRE_OPEN=false;
                    Fire_cnt = 0;
                }
            }
        }
    }
}

void Dial_Auto()
{
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;

    if( sys.fire_state.FIRE_OPEN == true && (Fire_process.Stuck_flag == false) )
    {
        if( (abs(Vision_process.predict_angle)<5)||(judge_sensor.info->GameRobotStatus.remain_HP <= HP_Danger) )
        {
//            Fire_process.Speed_target = SHOOT_FREQ_HIGH; //比较静止的时候高射频
            Fire_process.Speed_target = SHOOT_FREQ_HEATLIMIT;
//            Fire_process.Speed_target = SHOOT_FREQ_MID;
        }

        else
        {
            Fire_process.Speed_target = SHOOT_FREQ_HEATLIMIT;
//            Fire_process.Speed_target = SHOOT_FREQ_HIGH;
//            Fire_process.Speed_target = SHOOT_FREQ_MID;
//			Fire_process.Speed_target = SHOOT_FREQ_TWELVE;
        }

        if(master_sensor.info->cooling_heat > 280)
        {
//            Fire_process.Speed_target = SHOOT_FREQ_LOW;
            Fire_process.Speed_target = SHOOT_FREQ_ONE;
        }
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
        if(stuck_cnt>350)//2ms任务
        {
            Fire_process.Stuck_flag = true;
            stuck_cnt = 0;
        }
    } else
    {
        stuck_cnt = 0;
    }
    if(Fire_process.Stuck_flag)
    {
        Fire_process.Speed_target = -400;
        reverse_cnt++;
        if(reverse_cnt>250)
        {
            Fire_process.Stuck_flag = false;
            reverse_cnt = 0;
        }
    }
}

static void Dail_text()
{
    Fire_process.PVM.target = Fire_process.Speed_target;
    Fire_process.PVM.measure = motor[DIAL].info->speed;

    pid_calculate(&Fire_process.PVM);
    if( (sys.fire_state.FIRE_OPEN) && (sys.fire_state.FRICTION_OPEN) )//原来有 && (judge_sensor.info->GameRobotStatus.mains_power_shooter_output == 1)
    {
        NormalData_0x1FF[2] = (int16_t)Fire_process.PVM.out;
//		NormalData_0x200[2] = 0;
    }
    else
    {
        sys.fire_state.FIRE_OPEN = false;
        Fire_process.Speed_target = 0;
        Fire_process.PVM.target = 0;
        NormalData_0x1FF[2] = (int16_t)Fire_process.PVM.out;
//        NormalData_0x1FF[2] = 0;
    }
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
    Fire_process.PVM.kp=5;//
    Fire_process.PVM.ki=0.02;//
    Fire_process.PVM.kd=0;
    Fire_process.PVM.integral_max=6000;
    Fire_process.PVM.out_max=8000;
    Fire_process.PVM.out=0;
}
void StartFireTask(void const * argument)
{
    for(;;)
    {
        if( (sys.state == SYS_STATE_NORMAL) && (sys.switch_state.ALL_READY) )
        {
            Fire_Judge2_1();
//            Fire_Judge();
            if(sys.fire_state.FRICTION_OPEN)
            {
                if(sys.remote_mode == RC)
                {
//				      Dial_Remote_An();
                    Dial_Remote_Continue();
                }
                else if(sys.remote_mode == AUTO)
                {
                    Dial_Auto();
                }
            }
            Dail_text();
        }
        Friction_Control();
        osDelay(2);
    }
}
