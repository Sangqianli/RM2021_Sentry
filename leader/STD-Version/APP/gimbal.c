#include "gimbal.h"

float yaw_PPM_ratio=0.8;
float yaw_PVM_ratio=4;
float pitch_PVM_ratio=2;
float pitch_PPM_ratio=0.8;//陀螺仪0.2
float yaw_error;
float yaw_round;
float pitch_base;

CRUISE_Mode Cruise_mode;

PPM_TypeDef Gimbal_yaw_PPM;
PPM_TypeDef Gimbal_pitch_PPM;
PVM_TypeDef Gimbal_yaw_PVM;
PVM_TypeDef Gimbal_pitch_PVM;

void Gimbal_Init()
{
    Gimbal_yaw_PPM.PID_PPM.target=0;
    Gimbal_yaw_PPM.PID_PPM.kp=15;//15（水平）
    Gimbal_yaw_PPM.PID_PPM.ki=0;
    Gimbal_yaw_PPM.PID_PPM.kd=0;
    Gimbal_yaw_PPM.PID_PPM.IntegralLimit=8000;
    Gimbal_yaw_PPM.PID_PPM.MaxOutput=16000;
    Gimbal_yaw_PPM.PID_PPM.output=0;
    Gimbal_yaw_PPM.yaw_target=0;
 //   Gimbal_yaw_PPM.position_round=0;//yaw位置环
	
    Gimbal_yaw_PVM.PID_PVM.target=0;
    Gimbal_yaw_PVM.PID_PVM.kp=14;//水平14，垂直3.2
    Gimbal_yaw_PVM.PID_PVM.ki=0.6;//水平0.6，垂直0.3
    Gimbal_yaw_PVM.PID_PVM.kd=0;
    Gimbal_yaw_PVM.PID_PVM.IntegralLimit=26000;
    Gimbal_yaw_PVM.PID_PVM.MaxOutput=28000;
    Gimbal_yaw_PVM.PID_PVM.output=0;
    Gimbal_yaw_PVM.speed_set=0;//yaw速度环

    Gimbal_pitch_PVM.PID_PVM.target=0;
    Gimbal_pitch_PVM.PID_PVM.kp=0.8;//机械4
    Gimbal_pitch_PVM.PID_PVM.ki=0.1;//机械0.2
    Gimbal_pitch_PVM.PID_PVM.kd=0;
    Gimbal_pitch_PVM.PID_PVM.IntegralLimit=26000;
    Gimbal_pitch_PVM.PID_PVM.MaxOutput=28000;
    Gimbal_pitch_PVM.PID_PVM.output=0;
    Gimbal_pitch_PVM.speed_set=0;//pitch速度环

    Gimbal_pitch_PPM.PID_PPM.target=0;
    Gimbal_pitch_PPM.PID_PPM.kp=12;//机械15
    Gimbal_pitch_PPM.PID_PPM.ki=0;
    Gimbal_pitch_PPM.PID_PPM.kd=0;
    Gimbal_pitch_PPM.PID_PPM.IntegralLimit=8000;
    Gimbal_pitch_PPM.PID_PPM.MaxOutput=16000;
    Gimbal_pitch_PPM.PID_PPM.output=0;//pitch位置环

    Cruise_mode = SCOUT;
}


float yaw_target_last;
float yaw_target_now;
float pitch_target_last;
float pitch_target_now;

void Gimbal_Remote_Data()
{

    float  Gimbal_yaw_Q=1,Gimbal_yaw_R=3,Gimbal_pitch_Q=1,Gimbal_pitch_R=2;
    static extKalman_t Gimbal_yaw_p,Gimbal_pitch_p;
    static int8_t GK_create=0;
    if(GK_create==0)
    {
        KalmanCreate(&Gimbal_yaw_p,Gimbal_yaw_Q,Gimbal_yaw_R);
        KalmanCreate(&Gimbal_pitch_p,Gimbal_pitch_Q,Gimbal_pitch_R);
        GK_create=1;
    }
    if(Remote_Mode)
    {
        yaw_target_now=(-(RC_Ctl.rc.ch0-1024))*yaw_PPM_ratio;
        yaw_target_now=KalmanFilter(&Gimbal_yaw_p,yaw_target_now);
        if(yaw_target_now>0)
        {
            if((yaw_target_now-yaw_target_last)>0)
            {
                Gimbal_yaw_PPM.yaw_target=yaw_target_now+Gimbal_yaw_PPM.PID_PPM.measure;
                Gimbal_yaw_PPM.PID_PPM.target=Gimbal_yaw_PPM.yaw_target;
            }
            else
            {
                Gimbal_yaw_PPM.yaw_target=Gimbal_yaw_PPM.yaw_target;
                Gimbal_yaw_PPM.PID_PPM.target=Gimbal_yaw_PPM.yaw_target;
            }
        }
        if(yaw_target_now<0)
        {
            if((yaw_target_now-yaw_target_last)<0)
            {
                Gimbal_yaw_PPM.yaw_target=yaw_target_now+Gimbal_yaw_PPM.PID_PPM.measure;
                Gimbal_yaw_PPM.PID_PPM.target=Gimbal_yaw_PPM.yaw_target;
            }
            else
            {
                Gimbal_yaw_PPM.yaw_target=Gimbal_yaw_PPM.yaw_target;
                Gimbal_yaw_PPM.PID_PPM.target=Gimbal_yaw_PPM.yaw_target;
            }
        }
        yaw_target_last=yaw_target_now;
    }
    if(Remote_Mode)
    {
        pitch_target_now=(RC_Ctl.rc.ch1-1024)*pitch_PPM_ratio;
        pitch_target_now=KalmanFilter(&Gimbal_pitch_p,pitch_target_now);
        if(pitch_target_now>0)
        {
            if((pitch_target_now-pitch_target_last)>0)
            {
                Gimbal_pitch_PPM.position_set=pitch_target_now+Gimbal_pitch_PPM.PID_PPM.measure;
                if(Gimbal_pitch_PPM.position_set>2850)
                {
                    Gimbal_pitch_PPM.position_set=2850;
                }
                Gimbal_pitch_PPM.PID_PPM.target=Gimbal_pitch_PPM.position_set;
            }
            else
            {
                Gimbal_pitch_PPM.position_set=Gimbal_pitch_PPM.position_set;
                Gimbal_pitch_PPM.PID_PPM.target=Gimbal_pitch_PPM.position_set;
            }
        }
        if(pitch_target_now<0)
        {
            if((pitch_target_now-pitch_target_last)<0)
            {
                Gimbal_pitch_PPM.position_set=pitch_target_now+Gimbal_pitch_PPM.PID_PPM.measure;
                if(Gimbal_pitch_PPM.position_set<(-500))
                {
                    Gimbal_pitch_PPM.position_set=(-500);
                }
                Gimbal_pitch_PPM.PID_PPM.target=Gimbal_pitch_PPM.position_set;
            }
            else
            {
                Gimbal_pitch_PPM.position_set=Gimbal_pitch_PPM.position_set;
                Gimbal_pitch_PPM.PID_PPM.target=Gimbal_pitch_PPM.position_set;
            }
        }
        if(pitch_target_now==0)
        {
            Gimbal_pitch_PPM.position_set=Gimbal_pitch_PPM. position_set;
            Gimbal_pitch_PPM.PID_PPM.target=Gimbal_pitch_PPM.position_set;
        }
        pitch_target_last=pitch_target_now;
    }
}


float RealYaw_speed;
float roll_cos,roll_sin;

void Gimbal_Measure_Data()
{
    static float gg;
    gg= (roll*3.1415f)/180.0f;
    /*世界Yaw轴速度合成*/
    roll_cos = arm_cos_f32(gg);
    roll_sin = arm_sin_f32(gg);
    RealYaw_speed=gyroz*roll_cos+gyroy*roll_sin;
    /*..........................................*/
    Gimbal_yaw_PVM.PID_PVM.measure=RealYaw_speed;
    Gimbal_pitch_PVM.PID_PVM.measure=gyrox;
	/*PID参数调节*/
//	if(roll>0)
////	{
	    Gimbal_yaw_PVM.PID_PVM.kp=abs(3.2+(14-3.2)*roll_cos);
	    Gimbal_yaw_PVM.PID_PVM.ki=abs(0.3+(0.6-0.3)*roll_cos);//cos处理
//		Gimbal_yaw_PVM.PID_PVM.kp=14.f-p_step*roll;
//		Gimbal_yaw_PVM.PID_PVM.ki=0.6f-i_step*roll;//线性处理
//	}

}


/**
* @brief 侦察模式下的侦察函数
* @param void
* @return void
*	该函数作用在期望值上.
*/

const uint8_t SCOUT_PITCH_SPEED = 1,SCOUT_YAW_SPEED = 2; 	/*侦察速度*/

const int16_t SCOUT_YAW_RIGHT_ANGLE=-1000,SCOUT_YAW_LEFT_ANGLE=1000; 	/*侦察yaw轴角度左右边界*/

const int16_t SCOUT_PITCH_UP_ANGLE=0,SCOUT_PITCH_DOWN_ANGLE=-400;	/*侦察pitch轴角度左右边界*/

uint16_t scout_yaw_speed = SCOUT_YAW_SPEED;			/*侦察速度*/

uint16_t scout_pitch_speed = SCOUT_PITCH_SPEED; 	 	/*侦察速度*/

void Scout(void)
{
    static uint8_t yaw_dir = 1,pitch_dir = 1;	/*yaw轴和pitch轴的两个方向变量*/

    if(yaw_dir == 0)//向右侦察
    {
        Gimbal_yaw_PPM.PID_PPM.target -= scout_yaw_speed;
    } else if(yaw_dir == 1) //向左侦察
    {
        Gimbal_yaw_PPM.PID_PPM.target += scout_yaw_speed;
    }
    /*	判断边界  */
    if(Gimbal_yaw_PPM.PID_PPM.target <= SCOUT_YAW_RIGHT_ANGLE)
    {
        Gimbal_yaw_PPM.PID_PPM.target = SCOUT_YAW_RIGHT_ANGLE;
        yaw_dir=1;
    } else if(Gimbal_yaw_PPM.PID_PPM.target >= SCOUT_YAW_LEFT_ANGLE)
    {
        Gimbal_yaw_PPM.PID_PPM.target = SCOUT_YAW_LEFT_ANGLE;
        yaw_dir=0;
    }

    /*		↑以上是yaw轴的侦察处理↑			*/

    if(pitch_dir == 0)//向下侦察
    {
        Gimbal_pitch_PPM.PID_PPM.target -= scout_pitch_speed;
    } else if(pitch_dir == 1) //向上侦察
    {
        Gimbal_pitch_PPM.PID_PPM.target += scout_pitch_speed;
    }
    /*	判断边界  */
    if(Gimbal_pitch_PPM.PID_PPM.target >= SCOUT_PITCH_UP_ANGLE)
    {
        Gimbal_pitch_PPM.PID_PPM.target = SCOUT_PITCH_UP_ANGLE;
        pitch_dir=0;
    } else if(Gimbal_pitch_PPM.PID_PPM.target <= SCOUT_PITCH_DOWN_ANGLE)
    {
        Gimbal_pitch_PPM.PID_PPM.target = SCOUT_PITCH_DOWN_ANGLE;
        pitch_dir=1;
    }
    /*		↑以上是pitch轴的侦察处理↑			*/
}


void Gimbal_line()
{
	Gimbal_pitch_PPM.PID_PPM.target=constrain(Gimbal_pitch_PPM.PID_PPM.target,-500.f,2850.f);
	if(Gimbal_pitch_PPM.PID_PPM.measure>0)
	  Gimbal_yaw_PPM.PID_PPM.target=constrain(Gimbal_yaw_PPM.PID_PPM.target,-1987.f,1987.f);
	else
		Gimbal_yaw_PPM.PID_PPM.target=constrain(Gimbal_yaw_PPM.PID_PPM.target,-30.f,1600.f);
}



float ScoutYaw_target,ScoutPitch_target;//侦察模式时云台双轴的目标值
float AttackYaw_target,AttackPitch_target;
uint16_t attack_switch_ramp=0;
void GimbalMode_Switch()
{
    static int16_t Cruise_record_mode=0;
    if(Cruise_record_mode!=Cruise_mode)//巡航模式下的模式切换时
    {
        pid_clear(&Gimbal_pitch_PPM.PID_PPM);
        pid_clear(&Gimbal_pitch_PVM.PID_PVM);
        pid_clear(&Gimbal_yaw_PPM.PID_PPM);
        pid_clear(&Gimbal_yaw_PVM.PID_PVM);
        if(Cruise_mode == SCOUT)
        {
            ScoutYaw_target=Gimbal_yaw_PPM.PID_PPM.measure;
            ScoutPitch_target=Gimbal_pitch_PPM.PID_PPM.measure;
        }
        if(Cruise_mode == ATTACK)
        {
            AttackYaw_target=Gimbal_yaw_PPM.PID_PPM.measure;
            AttackPitch_target=Gimbal_pitch_PPM.PID_PPM.measure;

            ScoutYaw_target=0;
            ScoutPitch_target=0;

            attack_switch_ramp = ATTACK_SWITCH_RAMP;
            //切换后需要斜坡
        }
    }
    Cruise_record_mode=Cruise_mode;
// }
}

void Gimbal_task()
{
    Gimbal_Remote_Data();
    Gimbal_Measure_Data();
    if(Cruise_Mode)
    {
        GimbalMode_Switch();
        if(Cruise_mode == SCOUT)
        {
            Gimbal_yaw_PPM.PID_PPM.target=ScoutYaw_target;
            Gimbal_pitch_PPM.PID_PPM.target=ScoutPitch_target;
//		 Scout();
        }
        if(Cruise_mode == ATTACK)
        {
            if(attack_switch_ramp == 0)		/*完成了斜坡过程*/
            {
                AttackYaw_target=VisualProcess.YawTarget_KF;
                AttackPitch_target=VisualProcess.PitchTarget_KF;
                predict_flag=true;
            }
            else
            {
                AttackYaw_target = RampFloat(abs(VisualProcess.YawTarget_KF - AttackYaw_target)/attack_switch_ramp,VisualProcess.YawTarget_KF,AttackYaw_target);		/*切换后的斜坡处理*/
                AttackPitch_target=RampFloat(abs(VisualProcess.PitchTarget_KF - AttackPitch_target)/attack_switch_ramp,VisualProcess.PitchTarget_KF,AttackPitch_target);;
                attack_switch_ramp--;
                predict_flag=false;
            }
            Gimbal_yaw_PPM.PID_PPM.target=AttackYaw_target;
            Gimbal_pitch_PPM.PID_PPM.target=AttackPitch_target;
        }
    }
	Gimbal_line();
	
    pid_calculate(&Gimbal_yaw_PPM.PID_PPM);
    Gimbal_yaw_PVM.PID_PVM.target=Gimbal_yaw_PPM.PID_PPM.output;
    pid_calculate(&Gimbal_yaw_PVM.PID_PVM);

    pid_calculate(&Gimbal_pitch_PPM.PID_PPM);
    Gimbal_pitch_PVM.PID_PVM.target=Gimbal_pitch_PPM.PID_PPM.output;
    pid_calculate(&Gimbal_pitch_PVM.PID_PVM);
}


