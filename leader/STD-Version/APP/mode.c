#include "mode.h"

int RobotMonitor;
extern uint32_t Remote_time;
extern CRUISE_Mode Cruise_mode;
extern float ScoutYaw_target,ScoutPitch_target;
extern float yaw_target_last;
extern float yaw_target_now;
extern float pitch_target_last;
extern float pitch_target_now;

void Mode_Init()
{
    pid_clear(&Gimbal_pitch_PPM.PID_PPM);
    pid_clear(&Gimbal_pitch_PVM.PID_PVM);
    pid_clear(&Gimbal_yaw_PPM.PID_PPM);
    pid_clear(&Gimbal_yaw_PVM.PID_PVM);
    yaw_round=0;
//    Gimbal_yaw_PPM.position_round=0;
}

void Data_clear()
{
    pid_clear(&Gimbal_pitch_PPM.PID_PPM);
    pid_clear(&Gimbal_pitch_PVM.PID_PVM);
    pid_clear(&Gimbal_yaw_PPM.PID_PPM);
    pid_clear(&Gimbal_yaw_PVM.PID_PVM);

    ScoutYaw_target=0;
    ScoutPitch_target=0;	//侦察时的云台值

    yaw_target_last=0;
    yaw_target_now=0;
    pitch_target_last=0;
    pitch_target_now=0;//遥控器给定及记录值

    Friction_Open=0;//摩擦轮开启标志位清零
}



void Mode_switch()
{
    static int16_t Mode_ID_Now;
    static int16_t Mode_ID_Last;

    Mode_ID_Last=Mode_ID_Now;
    Mode_ID_Now=RC_Ctl.rc.s2;
    if(Mode_ID_Last!=Mode_ID_Now)
    {
        Data_clear();

        RobotMonitor=prepare_Mode;
        Blue_Off;

        while(RobotMonitor==prepare_Mode)
        {

            Recover_Slowly();
            if((sysTickUptime >= Remote_time))//如果遥控器一段时间内没有收到信息，强行将遥控器复位为没有遥控的状态
            {

                RC_Ctl.rc.ch0 = 1024;
                RC_Ctl.rc.ch1 = 1024;
                RC_Ctl.rc.ch2 = 1024;
                RC_Ctl.rc.ch3 = 1024;
                RC_Ctl.rc.s1 = 3;
                RC_Ctl.rc.s2 = 3;
                Stop();
                SystemMonitor = Error_Mode;
                RobotMonitor  =prepare_Mode;
                delay_ms(1);
                break;
            }


            if((abs(Gimbal_yaw_PPM.PID_PPM.err)<5)&&(abs(Gimbal_pitch_PPM.PID_PPM.err)<10))
            {
                RobotMonitor=control_Mode;

                Gimbal_yaw_PPM.PID_PPM.MaxOutput=16000;
                Gimbal_pitch_PPM.PID_PPM.MaxOutput=16000;

                Blue_On;
                break;
            }
        }


        if(Remote_Mode)
        {
//			Gimbal_yaw_PPM.position_round=0;
//			yaw_round=0;

            delay_ms(10);

            Gimbal_yaw_PPM.PID_PPM.kp=15;
            Gimbal_yaw_PPM.PID_PPM.ki=0;
            Gimbal_yaw_PPM.PID_PPM.kd=0;
            Gimbal_yaw_PVM.PID_PVM.kp=12.8;
            Gimbal_yaw_PVM.PID_PVM.ki=0.28;
            Gimbal_yaw_PVM.PID_PVM.kd=0;
            Gimbal_yaw_PPM.PID_PPM.target=Gimbal_yaw_PPM.PID_PPM.measure;
        }
        if(Cruise_Mode)
        {
//		Gimbal_yaw_PPM.position_round=0;
//		yaw_round=0;

            Gimbal_yaw_PPM.PID_PPM.kp=15;
            Gimbal_yaw_PPM.PID_PPM.ki=0;
            Gimbal_yaw_PPM.PID_PPM.kd=0;
            Gimbal_yaw_PVM.PID_PVM.kp=12.8;
            Gimbal_yaw_PVM.PID_PVM.ki=0.2;
            Gimbal_yaw_PVM.PID_PVM.kd=0;
            Cruise_mode=SCOUT;
        }

    }
}
void Recover_Slowly()
{

    Gimbal_yaw_PPM.PID_PPM.MaxOutput=800;
    Gimbal_pitch_PPM.PID_PPM.MaxOutput=800;
    Gimbal_yaw_PPM.PID_PPM.target=0;
    Gimbal_pitch_PPM.PID_PPM.target=0;

    BMI_GET_DATA(&gyrox,&gyroy,&gyroz,&accx,&accy,&accz);
    BMI_Get_data(&pitch,&roll,&yaw,&gyrox,&gyroy,&gyroz,&accx,&accy,&accz);//读取欧拉角

    Gimbal_yaw_PVM.PID_PVM.measure=gyroz;
    Gimbal_pitch_PVM.PID_PVM.measure=gyrox;

    pid_calculate(&Gimbal_yaw_PPM.PID_PPM);
    Gimbal_yaw_PVM.PID_PVM.target=Gimbal_yaw_PPM.PID_PPM.output;
    pid_calculate(&Gimbal_yaw_PVM.PID_PVM);

    pid_calculate(&Gimbal_pitch_PPM.PID_PPM);
    Gimbal_pitch_PVM.PID_PVM.target=Gimbal_pitch_PPM.PID_PPM.output;
    pid_calculate(&Gimbal_pitch_PVM.PID_PVM);

    CAN2_Send(0x1FF,(int16_t)(Gimbal_yaw_PVM.PID_PVM.output),(int16_t)(Gimbal_pitch_PVM.PID_PVM.output),0x0000,0x0000);
    delay_ms(2);
}
void Robot_Monitor()
{
    static int system_now;
    static int system_last;
    system_last=system_now;
    system_now=SystemMonitor;
    if(system_last!=system_now)
    {
        Mode_Init();
        Data_clear();
        if(system_now==Normal_Mode)
        {
            RobotMonitor=prepare_Mode;
            while(RobotMonitor==prepare_Mode)
            {
                Recover_Slowly();
                if((sysTickUptime >= Remote_time))//如果遥控器一段时间内没有收到信息，强行将遥控器复位为没有遥控的状态
                {
                    RC_Ctl.rc.ch0 = 1024;
                    RC_Ctl.rc.ch1 = 1024;
                    RC_Ctl.rc.ch2 = 1024;
                    RC_Ctl.rc.ch3 = 1024;
                    RC_Ctl.rc.s1 = 3;
                    RC_Ctl.rc.s2 = 3;
                    Stop();
                    SystemMonitor = Error_Mode;
                    RobotMonitor  =prepare_Mode;
                    delay_ms(1);
                    break;
                }


                if((abs(Gimbal_yaw_PPM.PID_PPM.err)<10)&&(abs(Gimbal_pitch_PPM.PID_PPM.err)<10))
                {
                    RobotMonitor=control_Mode;

                    Gimbal_yaw_PPM.PID_PPM.MaxOutput=16000;
                    Gimbal_pitch_PPM.PID_PPM.MaxOutput=16000;

                    Blue_On;
                    break;
                }
            }
        }
    }
    Mode_switch();
}


