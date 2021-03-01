#include "launcher.h"


PVM_TypeDef Launcher_Dial;
PPM_TypeDef An_Bullet_PPM;//单发位置外环
//PVM_TypeDef An_Bullet_PVM;//单发

int S1_last,S1_now,S2_last,S2_now;
int Friction_Open=0;
int Shoot_Open=0;
int Dail_Open=0;

int16_t Magezine_target;


void Launcher_Init()
{
    S1_last=3;
    S2_last=3;
    S1_now=3;
    S2_now=3;

    Launcher_Dial.PID_PVM.target=0;
    Launcher_Dial.PID_PVM.target=0;
    Launcher_Dial.PID_PVM.kp=4;
    Launcher_Dial.PID_PVM.ki=0.1;
    Launcher_Dial.PID_PVM.kd=0;
    Launcher_Dial.PID_PVM.output=0;
    Launcher_Dial.PID_PVM.IntegralLimit=3000;
    Launcher_Dial.PID_PVM.MaxOutput=8000;

    An_Bullet_PPM.PID_PPM.target=0;
    An_Bullet_PPM.PID_PPM.kp=0.1;
    An_Bullet_PPM.PID_PPM.ki=0;
    An_Bullet_PPM.PID_PPM.kd=0;
    An_Bullet_PPM.PID_PPM.output=0;
    An_Bullet_PPM.PID_PPM.IntegralLimit=3000;
    An_Bullet_PPM.PID_PPM.MaxOutput=8000;
	
	 Dail_Open=0;
}

void Friction_Control()
{
    static int Open_Line=1695;//1695
    static int Close_Line=1000;
    if( (S1_last==3)&&
            (S2_now==3)&&
            (S1_last!=S1_now)&&
            (S1_now==1))
    {
        if(Friction_Open==0)
            Friction_Open=1;
        else if(Friction_Open)
            Friction_Open=0;
    }
    if(Friction_Open==1)
    {
        PWM3_Target+=5;
        if(PWM3_Target>Open_Line)
        {
            PWM3_Target=Open_Line;
        }
    }
    if(Friction_Open==0)
    {
        PWM3_Target-=5;
        if(PWM3_Target<Close_Line)
        {
            PWM3_Target=Close_Line;
        }
    }
    Feeding_Bullet_PWM(PWM3_Target);
}

void Dial_Remote_An()
{
    static int16_t stuck_flag=0;
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;
    if( (S1_last==3)&&
            (S1_last!=S1_now)&&
            (S1_now==2))
    {
        Dail_Open=1;
        An_Bullet_PPM.PID_PPM.target=An_Bullet_PPM.PID_PPM.measure+36859.5f;

    }//测试
    if((Dail_Open==1)&&(stuck_flag==0))
    {
        Launcher_Dial.PID_PVM.target=An_Bullet_PPM.PID_PPM.output;
    }
    if((Dail_Open==0)&&(stuck_flag==0))
    {
        An_Bullet_PPM.PID_PPM.target=An_Bullet_PPM.PID_PPM.measure;
        Launcher_Dial.PID_PVM.target=An_Bullet_PPM.PID_PPM.output;
    }
    pid_calculate(&An_Bullet_PPM.PID_PPM);
    pid_calculate(&Launcher_Dial.PID_PVM);

    if(((abs(An_Bullet_PPM.PID_PPM.err))>20000)&&(reverse_cnt==0))
    {
        stuck_cnt++;
        if(stuck_cnt>500)
        {
            stuck_flag=1;
            stuck_cnt=0;
        }
    }
    if(stuck_flag)
    {
        Launcher_Dial.PID_PVM.target=-400;
        reverse_cnt++;
        if(reverse_cnt>300)
        {
            stuck_flag=0;
            reverse_cnt=0;
        }
    }
}


void  Dial_Remote_Continue()
{
	static int16_t stuck_flag=0;
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;
    if( (S1_last==3)&&
            (S1_last!=S1_now)&&
            (S1_now==2))
    {
		if(Dail_Open==1)
            Dail_Open=0;
		else
			Dail_Open=1;
    }
    if((Dail_Open==1)&&(stuck_flag==0))
    {
        Launcher_Dial.PID_PVM.target=1080;//4射频
    }	
	if((Dail_Open==0)&&(stuck_flag==0))
    {
        Launcher_Dial.PID_PVM.target=0;
    }
    pid_calculate(&Launcher_Dial.PID_PVM);

    if(((abs(An_Bullet_PPM.PID_PPM.err))>1000)&&(reverse_cnt==0))
    {
        stuck_cnt++;
        if(stuck_cnt>500)
        {
            stuck_flag=1;
            stuck_cnt=0;
        }
    }
    if(stuck_flag)
    {
        Launcher_Dial.PID_PVM.target=-400;
        reverse_cnt++;
        if(reverse_cnt>300)
        {
            stuck_flag=0;
            reverse_cnt=0;
        }
    }	
	
	
}
bool Fire_the_hole=false;//开火标志位
/**
* @brief 开火判断决策
* @param void
* @return void
* 对开火的条件判断
*/
void Fire_Judge(void)
{
    static uint16_t Fire_cnt=0;
    static float Fly_time=0,Real_Speed=0;
    Fly_time=VisualProcess.DistanceGet_KF/20;//30
    Real_Speed=(1000.f/vision_update_fps)*target_speed_raw;
    if((Cruise_mode == ATTACK)
            &&(target_speed_raw*VisualProcess.YawGet_KF<=0)
            &&((abs(Gimbal_yaw_PPM.PID_PPM.measure-YawTarget_now)<=abs(Real_Speed*Fly_time))|| (abs(predict_angle_raw*use_predic)<1))
            && (abs(VisualProcess.PitchGet_KF)<=10)	)
    {
        Fire_the_hole=true;
        Fire_cnt=0;
    }
    else
    {
        Fire_cnt++;
        if(Fire_cnt>100)
            Fire_the_hole=false;
    }
}
void Dial_Auto(void)
{
    static int16_t stuck_flag=0;
    static int32_t stuck_cnt=0;
    static int32_t reverse_cnt=0;

    Fire_Judge();

    if((Fire_the_hole==true)&&(stuck_flag==0))
    {
        Launcher_Dial.PID_PVM.target=1080;	//24射频6480，8射频2160,4射频1080
    }
    if(Fire_the_hole==false)
    {
        Launcher_Dial.PID_PVM.target=0;
    }
    pid_calculate(&Launcher_Dial.PID_PVM);

    if(((abs(Launcher_Dial.PID_PVM.err))>1000)&&(reverse_cnt==0))
    {
        stuck_cnt++;
        if(stuck_cnt>400)
        {
            stuck_flag=1;
            stuck_cnt=0;
        }
    }

    if(stuck_flag)
    {
        Launcher_Dial.PID_PVM.target=-400;
        reverse_cnt++;
        if(reverse_cnt>300)
        {
            stuck_flag=0;
            reverse_cnt=0;
        }
    }
}

void Launcher_task()
{
    S1_last=S1_now;
    S1_now=RC_Ctl.rc.s1;
    S2_now=RC_Ctl.rc.s2;

    Friction_Control();
    if(Remote_Mode)
	{	
//        Dial_Remote_An();
	    Dial_Remote_Continue();
	}
    if(Cruise_Mode)
        Dial_Auto();
}

