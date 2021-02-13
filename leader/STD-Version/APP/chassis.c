#include <chassis.h>

float chassis_position;

PVM_TypeDef Chassis;

int32_t mileage;//里程
int32_t mileage_max;//总里程



bool init_flag=false;		/*底盘里程数初始化*/
bool left_PES=false;		/*左边的光电开关状态*/
bool right_PES=false;		/*右边的光电开关状态*/

float rotate_ratio=8;

int8_t Derection_Flag;//1为左，-1为右

void Chassis_Init()
{
    Chassis.PID_PVM.kp=8;
    Chassis.PID_PVM.ki=0.15;
    Chassis.PID_PVM.kd=0;
    Chassis.PID_PVM.IntegralLimit=8000;
    Chassis.PID_PVM.MaxOutput=12000;

    Derection_Flag=1;

}

int32_t Get_Position(void)
{
    int32_t tim_cnt=0;	/*读取定时器计数器的变量*/

    tim_cnt = (int32_t)TIM1->CNT;			/*读取定时器1的计数值*/

    if(tim_cnt>1200)
        tim_cnt -= 2400;		/*定时器的计数值为2400，因为使用了定时器的编码器接口，所以是会根据AB相相位关系来进行加减。所以当数值大于1200时，认为其为反向加即在0的左边*/

    TIM1->CNT = 0;		/*计数值需要清0*/

    return tim_cnt;	/*返回本次采样数据，左负右正*/


}

/**
* @brief 获取光电开关的状态
* @param void
* @return void
*	获取状态
*/
void Get_Switch_Status()
{
    if(Left_PES == 1)
        left_PES=false;
    else
        left_PES=true;//光电开关碰到就是true

    if(Right_PES == 1)
        right_PES=false;
    else
        right_PES=true;
}


void Chassis_Data()
{

    mileage += Get_Position();

    Get_Switch_Status();

}

void Cruise_Normal()
{
    static uint8_t delay_dir=0;
    if(Derection_Flag==1)
    {
        if(left_PES==true)//向左运动时碰到光电开关
        {
            delay_dir++;
            if(delay_dir>50)
            {
                delay_dir=0;
                mileage=0;//清空里程数
            }

        }

        if(mileage<100)
        {
            Derection_Flag=-1;
        }
        else
        {
            Chassis.PID_PVM.target=Derection_Flag*1000;
        }
    }//左

    if(Derection_Flag==-1)
    {
        if(right_PES==true)//向右运动时碰到光电开关
        {
            delay_dir++;
            if(delay_dir>50)
            {
                delay_dir=0;
                mileage_max=mileage;//记录最大里程数
            }

        }

        if(mileage>(mileage_max-100))
        {
            Derection_Flag=1;
        }
        else
        {
            Chassis.PID_PVM.target=Derection_Flag*1000;
        }
    }//右

}

float  Chassis_Q=2,Chassis_R=4;
int see;

void Chassis_task()
{
    static extKalman_t Chassis_p;
    static int8_t K_create=0;
    if(K_create==0)
    {
        KalmanCreate(&Chassis_p,Chassis_Q,Chassis_R);
        K_create=1;
    }

    Chassis_Data();


    if(Remote_Mode)
    {

        Chassis.PID_PVM.target=-(RC_Ctl.rc.ch2-1024)*rotate_ratio;
        Chassis.PID_PVM.target=KalmanFilter(&Chassis_p,Chassis.PID_PVM.target);
    }//遥控控制

    if(Cruise_Mode)//巡航状态
    {
        static uint8_t step=0;	/*初始化步骤*/
        if(init_flag == false)		/*未对里程完成初始化*/
        {
            switch(step)
            {
            case 0:			/*初始启动先向左边靠*/
            {
                static uint8_t delay0=0;
                if(left_PES==true)		/*左边的光电开关*/
                {
                    Chassis.PID_PVM.target=0;
                    delay0++;
                    if(delay0>=50)		/*防止误触 -- 延时确认*/
                    {
                        delay0=0;
                        step=1;
                    }
                }
                else	/*未识别到 -- 向左运动*/
                {
                    Chassis.PID_PVM.target = 800;	/*向左运动*/
                    delay0=0;
                }
                break;
            }
            case 1:
            {
                if(left_PES==true)
                {
                    Chassis.PID_PVM.target=-100;	/*向右微调，调节到刚好不触发光电开关的位置*/
                }
                else			/*微调完成后先停住，进入下一个调节阶段*/
                {
                    Chassis.PID_PVM.target=0;
                    mileage=0;	/*清除里程数 -- 开始记录*/
                    step=2;
                }
                break;
            }
            case 2:
            {
                static uint8_t delay2=0;
                if(right_PES==true)	/*已经识别到右轨道*/
                {
                    Chassis.PID_PVM.target=0;					/*防止误触 -- 延时确认*/
                    delay2++;
                    if(delay2>=50)
                        step=3;
                }
                else if(right_PES==false)		/*未识别到*/
                {
                    Chassis.PID_PVM.target=-800;			/*向右运动*/
                    delay2=0;												/*防止误触 -- 延时确认*/
                }
                break;
            }
            case 3:
            {
                if(right_PES==true)
                {
                    Chassis.PID_PVM.target=100;		/*已经识别到右光电开关，现在微调到刚好未识别到的状态*/
                }
                else			/*微调完成*/
                {
                    Chassis.PID_PVM.target=0;															/*停止运动*/

                    mileage_max=mileage;	/*记录总里程数*/

                    step=0;																							/*复位step*/
                    init_flag=true;											/*更新标志位*/
                }
                break;
            }
            }
        }
        if(init_flag == true)		/*对里程完成初始化*/
        {
            Cruise_Normal();

        }
    }
    pid_calculate(&Chassis.PID_PVM);


}



