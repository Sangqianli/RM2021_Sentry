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
#define High_Speed 3600  //3600 ， 4600 ,3000
#define Normal_Speed 2500  //对抗赛3000,联盟赛2000
#define Attack_Speed 1200  //
#define Cover_Speed  2500
#define Escape_Speed 3200
#define First_Speed  -1000
#define High_Swerve_Ratio 0.6  //0.6
#define Normal_Swerve_Ratio 0.2  //对抗赛0.1
#define Atrip_Num 10.f  //轨道分段数
#define Atrip_Err 100  //误差范围
#define PowerBuff_Hot    60.F
#define PowerLimit_Normal 12000U
#define HP_HURT   -10
#define HP_VERYHURT   -60
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float swerve_ratio = 0.1;
/* Exported variables --------------------------------------------------------*/
Chassis_t Chassis_process;
/* Private functions ---------------------------------------------------------*/

/**
* @brief 初始巡航
* @param void
* @return void
* 用于第一遍跑轨时对轨道长度的校定，跑轨速度较慢，一般在3分钟准备中完成
*/
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
                Chassis_process.Derection_flag=-1;//向左运动标志位
            }
            break;
        }
        }
    }
}

/**
  * @brief  血量检查函数
	* @note   每500ms检测一次血量变化，若血量降低
	*					说明装甲受到伤害
  * @params  void
  * @retval bool，值为1表示受到伤害，值为0表示未受到伤害
  */
bool HP_Check(void)
{
    static  uint16_t last_HP = 600;
    static  uint16_t cur_HP = 0;
    static  int16_t delta_HP;
    static  uint32_t HP_check_time = 1;
    static  bool result = 0;

    HP_check_time++;

    if(HP_check_time % 100 == 0)                        //每200ms检测一次血量变化值
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_HURT && delta_HP > -60)      //每200ms检测一次血量变化值
            result = true;
        else
            result = false;
        HP_check_time = 1;
    }
    else
        result = false;

    return result;
}

bool HP_GOLF_Check(void)
{
    static  uint16_t last_HP = 600;
    static  uint16_t cur_HP = 0;
    static  int16_t delta_HP;
    static  uint32_t HP_check_time = 1;
    static  bool result = 0;

    HP_check_time++;

    if(HP_check_time % 100 == 0)                        //每200ms检测一次血量变化值
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_VERYHURT)                        //每200ms检测一次血量变化值
            result = true;
        else
            result = false;
        HP_check_time = 1;
    }
    else
        result = false;

    return result;
}

/**
  * @brief  受攻击判断函数
	* @note 受到击打3次，判断为进入危险状态，
	*					若6s以内没有受到攻击，进入安全
	*					状态。每次重新运行程序，第一次受
	*					到2次攻击才会进入危险状态，而后
	*					每受到1次攻击就会进入危险状态。
  * @params ishurt：是否受到攻击
  * @retval bool，值为1表示进入危险状态，值为0表示未进入安全状态
  */
bool IS_Danger(bool ishurt, uint8_t hurt_times)
{
    static bool hurtflag = 0;
    static bool result = 0;
    static uint32_t safe_cnt,hurt_cnt;

    if(ishurt)                //如果受到攻击
    {
        safe_cnt = 0;
        if(hurtflag != 1)
        {
            hurtflag = 1;
            hurt_cnt++;           //记录受到伤害次数
            if(hurt_cnt >= hurt_times)     //受到n次伤害以上，判断为进入危险
            {
                hurt_cnt = 0;
                result = true;
            }
        }
    }
    else
    {
        hurtflag = 0;
        safe_cnt++;
        if(safe_cnt > 6000)
            result = false;
    }
    return result;
}

bool IS_GOLF_Danger(bool ishurt, uint8_t hurt_times)
{
    static bool hurtflag = 0;
    static bool result = 0;
    static uint32_t safe_cnt,hurt_cnt;

    if(ishurt)                //如果受到攻击
    {
        safe_cnt = 0;
        if(hurtflag != 1)
        {
            hurtflag = 1;
            hurt_cnt++;           //记录受到伤害次数
            if(hurt_cnt >= hurt_times)     //受到n次伤害以上，判断为进入危险
            {
                hurt_cnt = 0;
                result = true;
            }
        }
    }
    else
    {
        hurtflag = 0;
        safe_cnt++;
        if(safe_cnt > 10000)
            result = false;
    }
    return result;
}
/**
* @brief 反弹起始点判断
* @param bool
* @return bool
* 分别判定是否达到反弹流程起始点
*/
static bool Is_LeftSwerve_Spot()
{
    if(path_sensor.info->mileage_total < (Chassis_process.Mileage_atrip * swerve_ratio) )
        return true;
    else
        return false;
}

static bool Is_RightSwerve_Spot()
{
    if(path_sensor.info->mileage_total >= (Chassis_process.Mileage_atrip * (1-swerve_ratio)) )
        return true;
    else
        return false;
}

/**
* @brief 半轨判断
* @param bool
* @return bool
* 判断是否到达轨道中点、左半段、右半段
*/
static bool Is_PathMid_Spot()
{
    if(abs( path_sensor.info->mileage_total- Chassis_process.Mileage_atrip * 0.5) <= 10 )
        return true;
    else
        return false;
}

static bool Is_LeftHalf_Spot()
{
    if(path_sensor.info->mileage_total < (Chassis_process.Mileage_atrip * 0.5) )
        return true;
    else
        return false;
}

static bool Is_RightHalf_Spot()
{
    if(path_sensor.info->mileage_total >= (Chassis_process.Mileage_atrip * (1-0.5)) )
        return true;
    else
        return false;
}
/**
* @brief 四分之一轨判断
* @param bool
* @return bool
* 分别判定是否达到四分之一段
*/
static bool Is_LeftQuarter_Spot()
{
    if(path_sensor.info->mileage_total < (Chassis_process.Mileage_atrip * 0.25) )
        return true;
    else
        return false;
}

static bool Is_RightQuarter_Spot()
{
    if(path_sensor.info->mileage_total >= (Chassis_process.Mileage_atrip * (1-0.25)) )
        return true;
    else
        return false;
}

/**
* @brief 逃跑判断
* @param bool
* @return bool
* 判定是否残血，残血高速全程跑轨
*/
static bool Is_TimeToRun()
{
    if(judge_sensor.info->GameRobotStatus.remain_HP <= HP_Danger)
        return true;
    else
        return false;
}

/**
* @brief 位置判断
* @param bool
* @return bool
* 判断当前位于哪段，左半段返回1，右半段返回2
*/
static bool Where_SpotNow( )
{
    int8_t Now;

    if( Is_LeftHalf_Spot() )
        Now = 1;
    else if( Is_RightHalf_Spot() )
        Now = 2;

    return Now;
}
/**
* @brief 到达判断
* @param bool
* @return bool
* 判定轨道目标位置是否到达
*传入整数，表示轨道分段定点位置
*/
static bool Is_SpotArrive( int32_t spot_target)
{
    int32_t err;
    err = abs(spot_target - path_sensor.info->mileage_total);
    if(err < Atrip_Err)
        return true;
    else
        return false;
}
/**
* @brief 速度设置
* @param void
* @return void
* 根据情况修改底盘速度，包括反弹路程的比例
*/
static void Chassis_Speed_Set()
{
    if( Is_TimeToRun() )
    {
        Chassis_process.Speed_taget=Chassis_process.Derection_flag * High_Speed;
        swerve_ratio = High_Swerve_Ratio;
    }
    else
    {
        Chassis_process.Speed_taget=Chassis_process.Derection_flag * Normal_Speed;
        swerve_ratio = Normal_Swerve_Ratio ;
    }
}
/**
* @brief 全轨巡航
* @param void
* @return void
* 完成轨道初始校准后的巡航，可以慢一点速度
* -1为左，1为右
*/
static void Cruise_Normal()
{
    Chassis_process.getchange_flag = true;//进入变向时获取位置

    if(Chassis_process.Derection_flag == -1)
    {
//        if( Is_LeftSwerve_Spot()&& (!Chassis_process.overbuff_flag) )
//            Chassis_process.swerve_flag = true;//向左运动到达卸力点
//        else
//            Chassis_process.swerve_flag = false;

        if(path_sensor.info->left_touch)//向左运动时碰到点触开关
        {
            Chassis_process.swerve_judge = true;//开始反弹判断
            Chassis_process.swerve_flag = true;
        }

        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->left_touch == false)//反弹至点触开关释放
            {
                Chassis_process.Trip_times ++;     //记录往返次数
                Chassis_process.Derection_flag = 1;//向右跑轨
                path_sensor.info->mileage_total = 0;//清空里程数
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//反弹完成
            }
        }
    }//左

    if(Chassis_process.Derection_flag == 1)
    {
//        if( Is_RightSwerve_Spot()&& (!Chassis_process.overbuff_flag) )
//            Chassis_process.swerve_flag = true;//向左运动到达卸力点,且没被功率控制
//        else
//            Chassis_process.swerve_flag = false;

        if(path_sensor.info->right_touch)//向右运动时碰到点触开关
        {
            Chassis_process.swerve_judge = true;//开始反弹判断
            Chassis_process.swerve_flag = true;
        }
        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->right_touch == false)//反弹至点触开关释放
            {
                Chassis_process.Derection_flag = -1;//向左跑轨
                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//反弹完成
            }
        }
    }//右
}

/**
* @brief 反弹处理
* @param void
* @return void
* 进入能量回收范围的处理
*/
static void Chassis_Rebound()
{
    if(Chassis_process.swerve_judge)
    {
        Chassis_process.swerve_flag = true; //反弹时标志位
    }
    else//没有反弹，正常巡航
    {
        Chassis_process.Speed_taget = Chassis_process.Speed_taget;
    }
}

/**
* @brief 遥控控制
* @param void
* @return void
*/
static void Chassis_RCcontrol()
{
//    Chassis_process.init_flag = false;
    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;

    Chassis_process.Speed_taget = rc_sensor.info->ch2*Chassis_process.rotate_ratio;
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    NormalData_0x200[0] = Chassis_process.PVM.out;
}

/**
  * @brief 	底盘电机静止处理：
	*					适用于点触开关失灵或者暴走卡在
	*					轨道一端时，执行底盘往另一个方
	*					向移动，跑到轨道另一端一个光电
	*					开关校准编码器位置后继续运行。
  * @retval void
  */
void Chassis_Stuck_Handle(void)
{
    static int16_t static_cnt = 0;
    static int16_t cold_cnt = 0;
    bool cold_judge = false;
    if( abs(motor[CHASSIS].info->speed) < 40 )
    {
        if(cold_judge == false)
        {
            static_cnt ++;
        } else
        {
            cold_cnt ++;
            static_cnt = 0;
            if(cold_cnt > 200)
            {
                cold_cnt = 0;
                cold_judge = false;
            }
        }
    }
    if( static_cnt>500 )
    {
        Chassis_process.Derection_flag = (-Chassis_process.Derection_flag);
        Chassis_process.swerve_flag = false;
        Chassis_process.swerve_judge = false;
        static_cnt = 0;
        cold_judge = true;
    }
}
/**
* @brief 变向跑轨
* @param void
* @return void
* 直接反向
*/
static void Chassis_Change_Dir(void)
{
    Chassis_process.Derection_flag = -Chassis_process.Derection_flag;

}

/**
* @brief 变向速度
* @param void
* @return void
* 自动变向跑轨的速度获取
*/
static void Chassis_Change_Speed(void)
{
    if(Chassis_process.Mode == CHASSIS_NORMAL)
        Chassis_process.Speed_taget = Chassis_process.Derection_flag * Normal_Speed;
    else if(Chassis_process.Mode == CHASSIS_ATTACK)
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Attack_Speed;
    else if(Chassis_process.Mode == CHASSIS_COVER)
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Cover_Speed;
    else if(Chassis_process.Mode == CHASSIS_ESCAPE)
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Escape_Speed;

}

/**
* @brief 变向距离
* @param void
* @return void
* 自动变向跑轨的距离获取,到达上次目标点后再调用
*/
static int8_t Chassis_Change_Dis(void)
{
    static uint8_t remain;
    int8_t dis;

    if(Chassis_process.Mode == CHASSIS_ATTACK)
    {
        remain = 9;
    } else
    {
        remain = 5;
    }
    dis = (rand() % remain) + 1;//生成1到9 的随机数

    return dis;
}

/**
* @brief 自动跑轨控制
* @param void
* @return void
* 反弹流程时直接让输出为0
*/
static void Chassis_AUTOcontrol()
{
    if(Chassis_process.init_flag)
    {
        Chassis_Speed_Set(); //设定速度
        Chassis_Stuck_Handle();
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.target = 0;
        Chassis_process.PVM.out = 0;  //反弹时输出为0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}

/**
* @brief 自动跑轨
* @param void
* @return void
*/
static void Chassis_AUTO()
{
    if(Chassis_process.init_flag)
    {
        Cruise_Normal();
        Chassis_Rebound();
    }
    else
    {
        Cruise_First();
    }
    Chassis_AUTOcontrol();
//				Chassis_process.init_flag = true;
//                Chassis_RCcontrol();

}

/**
* @brief 底盘状态获取
* @param void
* @return void
* 根据情况切换模式
*/
static void Chassis_GetMode()
{
    if( IS_GOLF_Danger(HP_GOLF_Check(), 1) )
    {
        Chassis_process.Safe = CHASSIS_DANGER;
    } else if( IS_Danger(HP_Check(), 3) )
    {
        Chassis_process.Safe = CHASSIS_HURT;
    } else
    {
        Chassis_process.Safe = CHASSIS_SAFE;
    }

    if( sys.auto_mode == AUTO_MODE_SCOUT)
    {
        Chassis_process.Mode = CHASSIS_NORMAL;
    } else if( sys.auto_mode == AUTO_MODE_ATTACK)
    {
        if( Chassis_process.Safe == CHASSIS_SAFE )
        {
            Chassis_process.Mode = CHASSIS_ATTACK;
        }
        else if( Chassis_process.Safe == CHASSIS_HURT )
        {
            Chassis_process.Mode = CHASSIS_COVER;
        }
        else if( Chassis_process.Safe == CHASSIS_DANGER )
        {
            Chassis_process.Mode = CHASSIS_ESCAPE;
        }
    }
}

/**
* @brief 变向流程
* @param void
* @return void
*/
int8_t Aim_dis;
static void Show_Time()
{
    if(Chassis_process.getchange_flag)
    {
        Chassis_Change_Dir(); //反向
        Aim_dis = Chassis_Change_Dis() * Chassis_process.Derection_flag ;//获取正负距离段数
        Chassis_process.Spot_taget = path_sensor.info->mileage_total + (int32_t)(Chassis_process.Mileage_atrip * Aim_dis/Atrip_Num);

        Chassis_process.Spot_taget = constrain(Chassis_process.Spot_taget, 4000,Chassis_process.Mileage_atrip - 4000);   //可以根据情况做不同的限位

        Chassis_process.getchange_flag = false;
    }
    if( Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.getchange_flag = true;  //到达再进入获取变向参数
    }


}
/**
* @brief 自动跑轨控制2.0
* @param void
* @return void
* 控制输出
*/
static void Chassis_AUTOcontrol_2_0()
{
    if(Chassis_process.init_flag)
    {
        Chassis_Change_Speed();
//        Chassis_Speed_Set(); //设定速度
        Chassis_Stuck_Handle();
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.target = 0;
        Chassis_process.PVM.out = 0;  //反弹时输出为0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief 自动跑轨2.0
* @param void
* @return void
*加入了变向跑轨
*/
static void Chassis_AUTO_2_0()
{
//    Chassis_GetMode(); //获取状态
//    if(Chassis_process.init_flag)
//    {
//		if( Chassis_process.Mode == CHASSIS_NORMAL)
//		{
//            Cruise_Normal();
//		}
//		else{
    Show_Time();
//		}
//		Chassis_Rebound();
//    }
//    else
//    {
//        Cruise_First();
//    }
//    Chassis_AUTOcontrol_2_0();
//				Chassis_process.init_flag = true;
//                Chassis_RCcontrol();

}

/**
* @brief 底盘的功率环模式控制
* @param void
* @return void
* @berif  K_limit = (J剩余/J最大)。  Out_final = K_limit^2*Out_PID
*/
float K_limit = 1;
static void Chassis_Power_Control()
{
    static int16_t Cold_time = 0;
    if(judge_sensor.info->power_heat_update) //功率数据更新正常时进行功率控制，发送频率50hz
    {
        if(judge_sensor.info->PowerHeatData.chassis_power_buffer < PowerBuff_Hot)
        {
            Cold_time = 0;
            Chassis_process.overbuff_flag = true;
            K_limit =(float)(judge_sensor.info->PowerHeatData.chassis_power_buffer / PowerBuff_Hot);
            Chassis_process.PVM.out_max = PowerLimit_Normal * K_limit * K_limit;
        }
        else
        {
            Cold_time++;
            if(Cold_time >= 100)
            {
                Cold_time = 0;
                Chassis_process.overbuff_flag = false;
                Chassis_process.PVM.out_max = PowerLimit_Normal;
                K_limit = 1;
            }
        }
        judge_sensor.info->power_heat_update = false;
    }
//	else  //没功率数据
//	{
//		Chassis_process.PVM.out_max = PowerLimit_Normal;
//		//K_limit = 1;
//	}
}
/* Exported functions --------------------------------------------------------*/

/**
* @brief 底盘参数初始化
* @param void
* @return void
* @berif  直观一点
*/
void Chassis_Init()
{
    Chassis_process.PVM.kp = 10;//18
    Chassis_process.PVM.ki = 0.02;//0.07
    Chassis_process.PVM.kd = 0;
    Chassis_process.PVM.integral_max = 8000;
    Chassis_process.PVM.out_max = 12000;
    Chassis_process.init_flag = false;
    Chassis_process.Derection_flag = -1;//初始向左
    Chassis_process.rotate_ratio = 8;
    Chassis_process.Trip_times = 1;
}
/**
* @brief 底盘任务
* @param void
* @return void
* @berif  2ms
*/
uint8_t dis;
void StartChassisTask(void const * argument)
{
    for(;;)
    {
		    dis = (rand() % 3) + 1;//生成1到9 的随机数
//        if(sys.state == SYS_STATE_NORMAL)
//        {
//            if( (sys.remote_mode == RC)||(sys.remote_mode == INSPECTION) )
//            {
//                Chassis_RCcontrol();
//            }
//            else if( sys.remote_mode == AUTO)
//            {
//                Chassis_AUTO();
//		Chassis_process.getchange_flag = true;
//				Chassis_AUTO_2_0();
//            }
//            Chassis_Power_Control();
//        } else
//        {
//            Chassis_process.init_flag = false;
//        }
        osDelay(2);
    }
}


