/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define SCOUT_PITCH_SPEED  1
#define SCOUT_YAW_SPEED    2	/*侦察速度*/
#define SCOUT_YAW_RIGHT_ANGLE  -1000
#define SCOUT_YAW_LEFT_ANGLE    1000 	/*侦察yaw轴角度左右边界*/

#define SCOUT_PITCH_UP_ANGLE     400
#define SCOUT_PITCH_DOWN_ANGLE  -400	/*侦察pitch轴角度上下边界*/

#define ATTACK_RAMP              200    //切换到打击模式下的斜坡参数

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t  Yaw_RCratio = 1;
uint16_t  Pitch_RCratio = 1;
float  Gimbal_yaw_Q = 1,Gimbal_yaw_R = 3,Gimbal_pitch_Q = 1,Gimbal_pitch_R = 2;
extKalman_t RC_yaw_p,RC_pitch_p;
uint16_t attack_switch_ramp=0;//记录斜坡步数，实际斜坡过程中不起作用
/* Exported variables --------------------------------------------------------*/
Gimbal_t  Gimbal_process = {
    . Gimbal_queue = {
        .nowLength = 0,
        .queue = {0},
        .is_queue_full = false
    }
};
/* Private functions ---------------------------------------------------------*/
static void Gimbal_RCdata()
{
    static float yaw_target_last;
    static float yaw_target_now;
    static float pitch_target_last;
    static float pitch_target_now;//记录遥控值
    yaw_target_now = rc_sensor.info->ch0 * Yaw_RCratio;
    yaw_target_now=KalmanFilter(&RC_yaw_p,yaw_target_now);
    pitch_target_now = rc_sensor.info->ch1  * Pitch_RCratio;
    pitch_target_now = KalmanFilter(&RC_pitch_p,pitch_target_now);

    if(yaw_target_now>0)
    {
        if((yaw_target_now-yaw_target_last)>0)
        {
            Gimbal_process.Yaw_taget = yaw_target_now+motor[GIMBAL_YAW].info->angle_sum;
        }

    }
    if(yaw_target_now<0)
    {
        if((yaw_target_now-yaw_target_last)<0)
        {
            Gimbal_process.Yaw_taget = yaw_target_now+motor[GIMBAL_YAW].info->angle_sum;
        }
    }

    if(pitch_target_now>0)
    {
        if((pitch_target_now-pitch_target_last)>0)
        {
            Gimbal_process.Pitch_taget = pitch_target_now-motor[GIMBAL_PITCH].info->angle_sum;
        }

    }
    if(pitch_target_now<0)
    {
        if((pitch_target_now-pitch_target_last)<0)
        {
            Gimbal_process.Pitch_taget = pitch_target_now-motor[GIMBAL_PITCH].info->angle_sum;
        }
    }

    yaw_target_last   = yaw_target_now;
    pitch_target_last = pitch_target_now;
}

static void Gimbal_Measure_Data()
{
    static float gg;
    static float roll_cos,roll_sin;
    gg= (imu_sensor.info->roll*3.1415f)/180.0f;
    /*世界Yaw轴速度合成*/
    roll_cos = arm_cos_f32(gg);
    roll_sin = arm_sin_f32(gg);
    Gimbal_process.RealYaw_speed=imu_sensor.info->rate_yaw*roll_cos+imu_sensor.info->rate_roll*roll_sin;
    /*..........................................*/
    Gimbal_process.YAW_PPM.measure = motor[GIMBAL_YAW].info->angle_sum;
    Gimbal_process.PITCH_PPM.measure = -motor[GIMBAL_PITCH].info->angle_sum;
    Gimbal_process.YAW_PVM.measure = Gimbal_process.RealYaw_speed;
//	Gimbal_process.YAW_PVM.measure = motor[GIMBAL_YAW].info->speed;
    Gimbal_process.PITCH_PVM.measure = -imu_sensor.info->rate_pitch;
}

static void Gimbal_control()
{
    Gimbal_process.YAW_PPM.target = Gimbal_process.Yaw_taget;
    Gimbal_process.PITCH_PPM.target = Gimbal_process.Pitch_taget;
    pid_calculate(&Gimbal_process.YAW_PPM);
    pid_calculate(&Gimbal_process.PITCH_PPM);

    Gimbal_process.YAW_PVM.target = Gimbal_process.YAW_PPM.out;
    Gimbal_process.PITCH_PVM.target = Gimbal_process.PITCH_PPM.out;
    pid_calculate (&Gimbal_process.YAW_PVM);
    pid_calculate(&Gimbal_process.PITCH_PVM);

    NormalData_0x2FF[0] =  (int16_t)(-Gimbal_process.PITCH_PVM.out);
    NormalData_0x2FF[1] =  (int16_t)(Gimbal_process.YAW_PVM.out);
	
//	CAN2_SendDataBuff(0x2ff,NormalData_0x2FF);
}
/**
* @brief 侦察模式下的侦察函数
* @param void
* @return void
*	该函数作用在期望值上.
*/
static  void Scout()
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw轴和pitch轴的两个方向变量*/
    if(yaw_dir == 0)//向右侦察
    {
        Gimbal_process.Yaw_taget -= SCOUT_YAW_SPEED;
    } else if(yaw_dir == 1) //向左侦察
    {
        Gimbal_process.Yaw_taget += SCOUT_YAW_SPEED;
    }
    /*	判断边界  */
    if(Gimbal_process.Yaw_taget <= SCOUT_YAW_RIGHT_ANGLE)
    {
        Gimbal_process.Yaw_taget = SCOUT_YAW_RIGHT_ANGLE;
        yaw_dir=1;
    } else if(Gimbal_process.Yaw_taget >= SCOUT_YAW_LEFT_ANGLE)
    {
        Gimbal_process.Yaw_taget = SCOUT_YAW_LEFT_ANGLE;
        yaw_dir=0;
    }

    /*		↑以上是yaw轴的侦察处理↑			*/

    if(pitch_dir == 0)//向下侦察
    {
        Gimbal_process.Pitch_taget -= SCOUT_PITCH_SPEED;
    } else if(pitch_dir == 1) //向上侦察
    {
        Gimbal_process.Pitch_taget += SCOUT_PITCH_SPEED;
    }
    /*	判断边界  */
    if(Gimbal_process.Pitch_taget >= SCOUT_PITCH_UP_ANGLE)
    {
        Gimbal_process.Pitch_taget = SCOUT_PITCH_UP_ANGLE;
        pitch_dir=0;
    } else if(Gimbal_process.Pitch_taget <= SCOUT_PITCH_DOWN_ANGLE)
    {
        Gimbal_process.Pitch_taget = SCOUT_PITCH_DOWN_ANGLE;
        pitch_dir=1;
    }
}

static void Gimbal_line()
{
//    Gimbal_process.Pitch_taget=constrain(Gimbal_process.Pitch_taget,-500.f,400.f);
//	if(Gimbal_pitch_PPM.PID_PPM.measure>0)
//	  Gimbal_yaw_PPM.PID_PPM.target=constrain(Gimbal_yaw_PPM.PID_PPM.target,-1987.f,1987.f);
//	else
//	Gimbal_process.Yaw_taget=constrain(Gimbal_process.Yaw_taget,-3000.f,3000.f);
}

static void AUTOMode_switch()
{
    if(sys.auto_mode == AUTO_MODE_SCOUT)
    {
        Gimbal_process.Yaw_taget = Gimbal_process.YAW_PPM.measure;
        Gimbal_process.Pitch_taget = Gimbal_process.PITCH_PPM.measure;
        sys.fire_state.FIRE_OPEN = false;
    }
    if(sys.auto_mode == AUTO_MODE_ATTACK)
    {
        sys.fire_state.FIRE_OPEN = false;
        attack_switch_ramp = ATTACK_RAMP;
    }
}

static void Gimbal_reset()
{
//    static float yaw_ramp,pitch_ramp;
//    static float delta_yaw,delta_pitch;
//    if(sys.switch_state.RESET_CAL)
//    {

//        delta_yaw = 5553.f - motor[GIMBAL_YAW].info->angle;
//        delta_pitch = 6680.f - motor[GIMBAL_PITCH].info->angle;
//        if(abs(delta_yaw)<4096)
//        {
//            delta_yaw = delta_yaw ;
//        } else if(delta_yaw>=4096)
//        {
//            delta_yaw -=8192;
//        } else if(delta_yaw<=-4096)
//        {
//            delta_yaw +=8192;
//        }
//        yaw_ramp = delta_yaw/1000.f;
//        pitch_ramp = delta_pitch/1000.f;
        sys.switch_state.RESET_CAL = false;
//    }
//    Gimbal_process.Yaw_taget =  RampFloat( delta_yaw,Gimbal_process.Yaw_taget, yaw_ramp);
//    Gimbal_process.Pitch_taget =  RampFloat( -delta_pitch, Gimbal_process.Pitch_taget, -pitch_ramp);
//    if( (abs(motor[GIMBAL_YAW].info->angle-5553)<=5)&&(abs(motor[GIMBAL_PITCH].info->angle-6680)<=5) )
//    {
//        if(sys.switch_state.SYS_RESET)
//        {
            sys.switch_state.SYS_RESET = false;
//            motor[GIMBAL_YAW].info->angle_sum = 0;
//        }
//        else if(sys.switch_state.REMOTE_SWITCH)
//        {
            sys.switch_state.REMOTE_SWITCH = false;
            motor[GIMBAL_YAW].info->angle_sum = 0;
//        }
//    }
}
static void Attack()
{
    if(sys.predict_state.PREDICT_ACTION)
    {


    } else if(sys.predict_state.PREDICT_OPEN)
    {
        Gimbal_process.Yaw_taget = Vision_process.data_kal.YawTarget_KF;
        Gimbal_process.Pitch_taget = Vision_process.data_kal.PitchTarget_KF;
    } else
    {
        Gimbal_process.Yaw_taget =  RampFloat(Vision_process.data_kal.YawTarget_KF, Gimbal_process.Yaw_taget, (Vision_process.data_kal.YawTarget_KF-Gimbal_process.Yaw_taget)/attack_switch_ramp );
        Gimbal_process.Pitch_taget =  RampFloat(Vision_process.data_kal.PitchTarget_KF, Gimbal_process.Pitch_taget, (Vision_process.data_kal.PitchTarget_KF-Gimbal_process.Pitch_taget)/attack_switch_ramp );
        attack_switch_ramp--;
        sys.predict_state.PREDICT_OPEN = false;
        if(attack_switch_ramp == 0 )
            sys.predict_state.PREDICT_OPEN = true;
    }//只是跟随
}

/* Exported functions --------------------------------------------------------*/
float Get_Queue_Angle(uint8_t time_dis)
{
    if(time_dis<=Gimbal_process.Gimbal_queue.nowLength)
        return Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength-time_dis];
    else
        return Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength+200-time_dis];
}

void Update_Gimbal_Angle_Queue(int16_t now_angle)
{
    if( Gimbal_process.Gimbal_queue.nowLength<199)
        Gimbal_process.Gimbal_queue.is_queue_full=false;
    else
        Gimbal_process.Gimbal_queue.is_queue_full=true;

    if(Gimbal_process.Gimbal_queue.is_queue_full==false)
    {
        Gimbal_process.Gimbal_queue.queue[Gimbal_process.Gimbal_queue.nowLength]=now_angle;
        Gimbal_process.Gimbal_queue.nowLength++;
    }
    if(Gimbal_process.Gimbal_queue.is_queue_full==true)
    {
        Gimbal_process.Gimbal_queue.nowLength=0;
    }
}

void Gimbal_Init()
{
    KalmanCreate(&RC_yaw_p,Gimbal_yaw_Q,Gimbal_yaw_R);
    KalmanCreate(&RC_pitch_p,Gimbal_pitch_Q,Gimbal_pitch_R);


    Gimbal_process.YAW_PPM.target=0;
    Gimbal_process.YAW_PPM.kp=15;//20
    Gimbal_process.YAW_PPM.ki=0;
    Gimbal_process.YAW_PPM.kd=0;
    Gimbal_process.YAW_PPM.integral_max=8000;
    Gimbal_process.YAW_PPM.out_max=16000;
    Gimbal_process.YAW_PPM.out=0;//yaw位置环

    Gimbal_process.YAW_PVM.target=0;
    Gimbal_process.YAW_PVM.kp=16;//16
    Gimbal_process.YAW_PVM.ki=0.3;//
    Gimbal_process.YAW_PVM.kd=0;
    Gimbal_process.YAW_PVM.integral_max=26000;
    Gimbal_process.YAW_PVM.out_max=28000;
    Gimbal_process.YAW_PVM.out=0;//yaw速度环


    Gimbal_process.PITCH_PPM.target=0;
    Gimbal_process.PITCH_PPM.kp=15;
    Gimbal_process.PITCH_PPM.ki=0;
    Gimbal_process.PITCH_PPM.kd=0;
    Gimbal_process.PITCH_PPM.integral_max=8000;
    Gimbal_process.PITCH_PPM.out_max=16000;
    Gimbal_process.PITCH_PPM.out=0;//pitch位置环

    Gimbal_process.PITCH_PVM.target=0;
    Gimbal_process.PITCH_PVM.kp=13;//
    Gimbal_process.PITCH_PVM.ki=0.6;//
    Gimbal_process.PITCH_PVM.kd=0;
    Gimbal_process.PITCH_PVM.integral_max=26000;
    Gimbal_process.PITCH_PVM.out_max=28000;
    Gimbal_process.PITCH_PVM.out=0;//pitch速度环
}

void StartGimbalTask(void const * argument)
{
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL)
        {
            Gimbal_Measure_Data();
            if(sys.switch_state.ALL_READY)
            {
                if(sys.remote_mode == RC)
                {
                    Gimbal_RCdata();
                }
                else if(sys.remote_mode == AUTO)
                {
                    if(sys.switch_state.AUTO_MODE_SWITCH)
                    {
                        AUTOMode_switch();
                        sys.switch_state.AUTO_MODE_SWITCH = false;
                    }
                    else if(sys.auto_mode == AUTO_MODE_SCOUT)
                    {
//					Scout();
                    } else if(sys.auto_mode == AUTO_MODE_ATTACK)
                    {
                        Attack();
                    }
                }
            }
            else
            {
                Gimbal_reset();
            }
            Gimbal_line();
            Gimbal_control();
        }
		
        osDelay(2);
    }
}


