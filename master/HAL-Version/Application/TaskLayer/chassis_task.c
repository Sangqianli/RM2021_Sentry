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
#define SPEED_MAX  4600
#define High_Speed 3600  //3600 ， 4600 ,3000
#define Normal_Speed 2400//对抗赛3000,联盟赛2000
#define Attack_Speed 1500  //1500
#define Cover_Speed  3000
#define Escape_Speed 3000 //2500
#define First_Speed  -1000
#define Base_Speed   1000
#define More_Speed   200
#define High_Swerve_Ratio 0.6  //0.6
#define Normal_Swerve_Ratio 0.2  //对抗赛0.1
#define Remain_ATTACK 6U   //取余数，rand%9+1 = 1~9
#define Remain_COVER  8U   //rand%5 + 1 = 1~5
#define Remain_ESCAPE 8U
#define Atrip_Num 10.f  //轨道分段数
#define Atrip_Err  200U  //误差范围
#define Atrip_Line 0U  //变向限位,实测要比误差范围大
#define Atrip_length 50000U
#define PowerBuff_Hot    60.F
#define PowerLimit_Normal 12000U
#define HP_HURT   -10
#define HP_VERYHURT   -60
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float swerve_ratio = 0.1;
int8_t Aim_dis;
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
                Chassis_process.Speed_taget = -1600;	/*向左运动*/
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
                Chassis_process.Speed_taget=1600;			/*向右运动*/
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
                Chassis_process.Spot_taget = 0;//位置环
                Chassis_process.getchange_flag = true;
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

        if( (judge_sensor.info->hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0) )
        {
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
            judge_sensor.info->hurt_data_update = false;
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

        if( (judge_sensor.info->hurt_data_update)&&(judge_sensor.info->RobotHurt.hurt_type == 0) )
        {
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
            judge_sensor.info->hurt_data_update = false;
        }
    }
    else
    {
        hurtflag = 0;
        safe_cnt++;
        if(safe_cnt > 12000)
            result = false;
    }
    return result;
}

/**
* @brief 进攻判断
* @param bool
* @return bool
* 判定正处于攻击状态
*/
static bool Is_Attack()
{
    static bool result = false;
    static uint32_t lost_cnt;

    if( (sys.auto_mode == AUTO_MODE_ATTACK)||(leader_sensor.info->data.attack_now == 1) )
        result = true;
    else
    {
        lost_cnt ++;
        if( lost_cnt >= 1000 ) //1s
        {
            result = false;
            lost_cnt = 0;
        }
    }
    return result;
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
* @brief 到达判断
* @param bool
* @return bool
* 判定轨道目标位置是否到达
*传入整数，表示轨道分段定点位置
*/
static bool Is_SpotArrive( int32_t spot_target)
{
    uint32_t err;
    err = abs(spot_target - path_sensor.info->mileage_total);
    if(err <= Atrip_Err)
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
        Chassis_process.Spot_taget = 0;//向左位置环给定

//		if(path_sensor.info->mileage_total < 0)
//		{
//			Chassis_process.Derection_flag = 1;//向右跑轨
//			Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//向右位置环给定
//		}//防止误差导致停在轨道一边


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
                Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//向右位置环给定
                path_sensor.info->mileage_total = 0;//清空里程数
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//反弹完成
            }
        }
    }//左

    if(Chassis_process.Derection_flag == 1)
    {
        Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//向右位置环给定
//		if(path_sensor.info->mileage_total > Chassis_process.Mileage_atrip)
//		{
//			Chassis_process.Derection_flag = -1;//向左跑轨
//			Chassis_process.Spot_taget = 0;//向左位置环给定
//		}//防止误差导致停在轨道一边

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
                Chassis_process.Spot_taget = 0;//向左位置环给定
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
  * @brief 	底盘电机静止处理：电机版本
  * @retval void
  */
void Chassis_Stuck_Handle_1_0(void)
{
    static int16_t static_cnt = 0;
    static int16_t cold_cnt = 0;
    bool cold_judge = false;
    if( (motor[CHASSIS].info->speed < 1000)&&(Chassis_process.Fire == FIRE_RUN)&&(Chassis_process.PVM.out >6000) )//abs(Chassis_process.PVM.out)>4000
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
    } else
    {
        static_cnt = 0;
        cold_cnt = 0;
        cold_judge = false;
    }
    if( static_cnt>500 )
    {
        if(Chassis_process.Derection_flag == 1)
        {
            path_sensor.info->mileage_total = Atrip_length + 200;//校正此时的位置
        }
        if(Chassis_process.Derection_flag == -1)
        {
            path_sensor.info->mileage_total = 0 - 200;//校正此时的位置
        }

        Chassis_process.getchange_flag = true;

        Chassis_process.Derection_flag = - Chassis_process.Derection_flag;

        Chassis_process.swerve_flag = false;
        Chassis_process.swerve_judge = false;
        static_cnt = 0;
        cold_judge = true;
    }
}

/**
  * @brief 	底盘电机静止处理：编码器版本
  * @retval void
  */
void Chassis_Stuck_Handle(void)
{
    static int16_t static_cnt = 0;
    static int16_t cold_cnt = 0;
    bool cold_judge = false;
    if( (abs(path_sensor.info->mileage_dif)<20)&&(Chassis_process.Fire == FIRE_RUN) )//abs(Chassis_process.PVM.out)>4000
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
    } else
    {
        static_cnt = 0;
        cold_cnt = 0;
        cold_judge = false;
    }
    if( static_cnt>500 )
    {
        Chassis_process.getchange_flag = true;
        if(Chassis_process.Mode == CHASSIS_NORMAL)
        {
            if(path_sensor.info->mileage_total < (Chassis_process.Mileage_atrip*0.5) )
            {
                Chassis_process.Derection_flag = 1;
                Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//向右位置环给定
            }
            else
            {
                Chassis_process.Derection_flag = -1;
                Chassis_process.Spot_taget = 0;//向左位置环给定
            }
        }
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
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag * Normal_Speed;
    }
    else if(Chassis_process.Mode == CHASSIS_ATTACK)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Cover_Speed ;
    }
    else if(Chassis_process.Mode == CHASSIS_COVER)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Escape_Speed;
    }
    else if(Chassis_process.Mode == CHASSIS_ESCAPE)
    {
        Chassis_process.Speed_taget = Chassis_process.Derection_flag *  Escape_Speed;
    }
    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -3000, 3000);  //速度限幅
}

/**
* @brief 变向距离
* @param void
* @return void
* 自动变向跑轨的距离获取,到达上次目标点后再调用
*/
static uint8_t Chassis_Change_Dis(void)
{
    static uint8_t remain;
    uint8_t dis;

    if(Chassis_process.Mode == CHASSIS_ATTACK)
    {
        remain = Remain_ATTACK;
        dis = (rand() % remain) + 1;//生成1到9 的随机数
    } else if(Chassis_process.Mode == CHASSIS_COVER)
    {
        remain = Remain_COVER;
        dis = (rand() % remain) + 2;//生成3到9 的随机数
    } else if(Chassis_process.Mode == CHASSIS_ESCAPE)
    {
        remain = Remain_ESCAPE;
        dis = (rand() % remain) + 2;//生成3到9 的随机数
    }

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
//    if(Chassis_process.init_flag)
//    {
//        Cruise_Normal();
//        Chassis_Rebound();
//    }
//    else
//    {
//        Cruise_First();
//    }
//    Chassis_AUTOcontrol();
    Chassis_process.init_flag = true;
    Chassis_RCcontrol();

}

/**
* @brief 云台手逃跑信息
* @param void
* @return void
*提供检测发送按键来判定是否进入逃跑模式
*返回true表示逃跑，返回flase表示不干涉
*/
static bool Run_Key_info()
{
    static	bool result = false;
    if(judge_sensor.info->AerialData.cmd & escape)
        result	= true;
    else
        result	= false;
    return	result;
}

/**
* @brief 云台手公路信息
* @param void
* @return void
*提供检测发送按键来判定是否停在公路端，只有在前哨站存活时才有效
*返回true表示公路端，返回flase表示不干涉
*/
static bool Highway_Key_info()
{
    static	bool result = false;
    if(judge_sensor.info->AerialData.cmd & check_road)
        result	= true;
    else
        result	= false;
    return	result;
}

/**
* @brief 云台手接管信息
* @param void
* @return void
*提供检测发送按键确定是否由云台手发送接管
*返回true表示接管，返回flase表示不干涉
*/
static bool Aerial_Control_info()
{
    static	bool result = false;
    static uint32_t wait_cnt;
    if(judge_sensor.info->AerialData.cmd & control_aerial)
        result	= true;
    else
        result	= false;

//    if(judge_sensor.info->communication_data_update)
//    {
//        wait_cnt = 0;
        judge_sensor.info->communication_data_update = false;
//    }
//    else
//    {
//        wait_cnt ++;  
//        if(wait_cnt >= 1000)
//        {
//            judge_sensor.info->AerialData.cmd = 0;
//            judge_sensor.info->AerialData.control_dir = 0;
//            wait_cnt = 0;
//        }
//    }
    return	result;
}

/**
* @brief 底盘状态获取
* @param void
* @return void
* 根据情况切换模式
*/
static void Chassis_GetMode()
{
    if( IS_GOLF_Danger(HP_GOLF_Check(), 1) )//||(vision_sensor.info->RxPacket.RxData.identify_hero == 1)
    {
        Chassis_process.Safe = CHASSIS_DANGER;
    } else if( IS_Danger(HP_Check(), 1) )
    {
        Chassis_process.Safe = CHASSIS_HURT;
    } else
    {
        Chassis_process.Safe = CHASSIS_SAFE;
    }

    if( Is_Attack() == false )
    {
        if( Chassis_process.Safe == CHASSIS_SAFE )
        {
            Chassis_process.Mode = CHASSIS_NORMAL;
        }
        else if( Chassis_process.Safe == CHASSIS_HURT )
        {
            Chassis_process.Mode = CHASSIS_COVER;
        }
        else if( Chassis_process.Safe == CHASSIS_DANGER|| Run_Key_info() )
        {
            Chassis_process.Mode = CHASSIS_ESCAPE;
        }
    } else if(  Is_Attack() == true )
    {
        if( Chassis_process.Safe == CHASSIS_SAFE )
        {
            Chassis_process.Mode = CHASSIS_ATTACK;
        }
        else if( Chassis_process.Safe == CHASSIS_HURT )
        {
            Chassis_process.Mode = CHASSIS_COVER;
        }
        else if( (Chassis_process.Safe == CHASSIS_DANGER)|| Run_Key_info() )
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
static void Show_Time()
{
    static bool left_err_judge = false; //用于左运动碰到左开关
    static bool right_err_judge = false;//用于右运动碰到右开关

    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;//清空巡航模式时的反弹动作


    if(Chassis_process.getchange_flag)
    {
        Chassis_Change_Dir(); //反向
        Aim_dis = Chassis_Change_Dis() * Chassis_process.Derection_flag ;//获取正负距离段数
        Chassis_process.Spot_taget = path_sensor.info->mileage_total + (int32_t)(Chassis_process.Mileage_atrip * (Aim_dis/Atrip_Num) );

        Chassis_process.Spot_taget = constrain(Chassis_process.Spot_taget, Atrip_Line,Chassis_process.Mileage_atrip - Atrip_Line );   //可以根据情况做不同的限位

        Chassis_process.getchange_flag = false;
    }

    if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
    {
        Chassis_process.Derection_flag = 1;
    } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) >= 0 )
    {
        Chassis_process.Derection_flag = -1;
    }

    if( Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.getchange_flag = true;  //到达再进入获取变向参数
    }
    //刹不住车也没事，此时反向判断开关，同时校准编码器参数
    if(Chassis_process.Derection_flag == -1)//向左运动碰开关
    {
        if(path_sensor.info->right_touch)//碰到右开关
        {
            Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
        }
//            right_err_judge = true;
//        }
//        if(right_err_judge)
//        {
//            if(path_sensor.info->right_touch == false)//反弹至点触开关释放
//            {
//                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
//                right_err_judge = false;
//            }
//        }
        if(path_sensor.info->left_touch)//向左运动时碰到左开关
        {
            path_sensor.info->mileage_total = 0;//清空里程数
        }

//            Chassis_process.swerve_judge = true;//开始反弹判断
//            Chassis_process.swerve_flag = true;
//        }
//        if(Chassis_process.swerve_judge)
//        {
//            if(path_sensor.info->left_touch == false)//反弹至点触开关释放
//            {
////                Chassis_process.getchange_flag = true; //重新获取距离
//                path_sensor.info->mileage_total = 0;//清空里程数
//                Chassis_process.swerve_judge = false;
//                Chassis_process.swerve_flag = false;//反弹完成
//            }
//        }
    }//左

    if(Chassis_process.Derection_flag == 1)//向右运动碰开关
    {
        if(path_sensor.info->left_touch)//碰到左开关
        {
            path_sensor.info->mileage_total = 0;//清空里程数

        }

//            left_err_judge = true;
//        }
//        if(left_err_judge)
//        {
//            if(path_sensor.info->left_touch == false)//反弹至点触开关释放
//            {
//                path_sensor.info->mileage_total = 0;//清空里程数
//                left_err_judge = false;
//            }
//        }

        if(path_sensor.info->right_touch)//碰到右开关
        {
            Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
        }

//            Chassis_process.swerve_judge = true;//开始反弹判断
//            Chassis_process.swerve_flag = true;
//        }
//        if(Chassis_process.swerve_judge)
//        {
//            if(path_sensor.info->right_touch == false)//反弹至点触开关释放
//            {
////                Chassis_process.getchange_flag = true;//重新获取距离
//                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
//                Chassis_process.swerve_judge = false;
//                Chassis_process.swerve_flag = false;//反弹完成
//            }
//        }
    }//右
}

/**
* @brief 定点打击
* @param void
* @return void
* 前哨站存活时静止于靠近轨道右侧，但不接触柱子
*/
static void Static_shoot()
{
    Chassis_process.Mode = CHASSIS_NORMAL;//一般为正常状态

    if( Highway_Key_info() )
        Chassis_process.Spot_taget = 2000;
    else
        Chassis_process.Spot_taget = (Chassis_process.Mileage_atrip - 2000);
    if(Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.static_want =true;
    } else
    {
        if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
        {
            Chassis_process.Derection_flag = 1;
        } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) > 0 )
        {
            Chassis_process.Derection_flag = -1;
        }
        Chassis_process.static_want = false;
    }
}

/**
* @brief 底盘运动模式获取
* @param void
* @return void
* 根据前哨站状态决定是否静止或者正常跑轨
*/
static void Outpost_get()
{
    static uint16_t  tum_cnt;

    if(judge_sensor.info->EventData.outpost == 1) //前哨站存活时，bit10 == 1
    {
        Chassis_process.Fire = FIRE_ALL;
    } else {
        /***************遥控器拨轮控制模拟前哨站，主要用于训练方便，比赛时去掉*********/
        if( abs(rc_sensor.info->thumbwheel) >=630 )
        {
            tum_cnt++;
            if(tum_cnt > 100)
            {
                if(rc_sensor.info->thumbwheel >= 630)
                {
                    Chassis_process.Fire = FIRE_ALL;
                } else if(rc_sensor.info->thumbwheel <= -630)
                {
                    Chassis_process.Fire = FIRE_RUN;
                }
                tum_cnt = 0;
            }
        }
        /******************************************************/
//        Chassis_process.Fire = FIRE_RUN;//比赛时恢复
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
        if(Chassis_process.Fire == FIRE_RUN)
        {
            Chassis_Change_Speed();
            Chassis_Stuck_Handle_1_0();
        } else if(Chassis_process.Fire == FIRE_ALL)
        {
            Chassis_process.Speed_taget = 0;
        }
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.out = 0;  //反弹时输出为0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}

/**
* @brief 自动跑轨控制3.0
* @param void
* @return void
* 全程双环PID控制输出，希望编码器永远不会有事...
*/
static void Chassis_AUTOcontrol_3_0()
{
    if(Chassis_process.init_flag)
    {
        if(Chassis_process.Mode == CHASSIS_NORMAL)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = 4000;
        }
        else if(Chassis_process.Mode == CHASSIS_ATTACK)
        {
            Chassis_process.PPM.kp = 0.3;
            Chassis_process.PPM.out_max = 3000;
        }
        else if(Chassis_process.Mode == CHASSIS_COVER)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = SPEED_MAX;
        }
        else if(Chassis_process.Mode == CHASSIS_ESCAPE)
        {
            Chassis_process.PPM.kp = 0.4;
            Chassis_process.PPM.out_max = SPEED_MAX;
        }

        Chassis_Stuck_Handle(); // 底盘静止处理



        Chassis_process.PPM.target = Chassis_process.Spot_taget;
        Chassis_process.PPM.measure = path_sensor.info->mileage_total;
        pid_calculate(&Chassis_process.PPM);

        if( (Chassis_process.Fire == FIRE_ALL)&&(Chassis_process.static_want == true) )
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total;
//            Chassis_process.Speed_taget = Chassis_process.PPM.out;
            Chassis_process.Speed_taget = 0;
        } else
        {
            Chassis_process.Speed_taget = Chassis_process.PPM.out;

            if(Chassis_process.Mode == CHASSIS_NORMAL)
            {
                if(Chassis_process.Spot_taget == 0)//方向向左
                { 
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -4000, -Chassis_process.PPM.integral_max);
                } else if(Chassis_process.Spot_taget == Chassis_process.Mileage_atrip)//方向向右
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, Chassis_process.PPM.integral_max, 4000);
                }
            }
            else
            {
                if(Chassis_process.Derection_flag == 1)//向右
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, Chassis_process.PPM.integral_max, SPEED_MAX);
                } else if(Chassis_process.Derection_flag == -1)//向左
                {
                    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -SPEED_MAX, -Chassis_process.PPM.integral_max);
                }//加速度偏置，防止速度太慢停下来
            }
        }
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.out = 0;  //反弹时输出为0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief 触碰开关模式自动跑轨
* @param void
* @return void
*
*/
static void Chassis_AUTO_TOUCH()
{
    Outpost_get();//获取前哨站状态，改变底盘模式

    Chassis_process.init_flag = true;

    if(Chassis_process.Fire == FIRE_ALL)
    {
        Chassis_process.Speed_taget = 0;
    } else if(Chassis_process.Fire == FIRE_RUN)
    {
        Chassis_GetMode(); //获取状态
        if(Chassis_process.Derection_flag == -1)
        {
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
                    Chassis_process.Spot_taget = Chassis_process.Mileage_atrip;//向右位置环给定
                    path_sensor.info->mileage_total = 0;//清空里程数
                    Chassis_process.swerve_judge = false;
                    Chassis_process.swerve_flag = false;//反弹完成
                }
            }
        }//左

        if(Chassis_process.Derection_flag == 1)
        {
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
                    Chassis_process.Spot_taget = 0;//向左位置环给定
                    Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//记录最大里程数
                    Chassis_process.swerve_judge = false;
                    Chassis_process.swerve_flag = false;//反弹完成
                }
            }
        }//右
    }
    Chassis_Rebound();
    Chassis_AUTOcontrol_2_0();
}

/**
* @brief 编码器模式跑轨
* @param void
* @return void
*
*/
static void Chassis_AUTO_ENCODER()
{
    static float static_pot;
    static uint16_t err_cnt;

    Outpost_get();//获取前哨站状态，改变底盘模式

    Chassis_process.init_flag = true;

    if(Chassis_process.Fire == FIRE_ALL)
    {
        Chassis_process.Speed_taget = 0;
        static_pot = path_sensor.info->mileage_total;//记录此时的位置
        path_sensor.info->mileage_total = Atrip_length;
    } else if(Chassis_process.Fire == FIRE_RUN)
    {
        Chassis_GetMode(); //获取状态

        if(Chassis_process.Derection_flag == -1)
        {
            if(path_sensor.info->mileage_total <= 0 )//到达位置释放
            {
                Chassis_process.Derection_flag = 1;//向右跑轨
            }
        }//左

        if(Chassis_process.Derection_flag == 1)
        {
            if(path_sensor.info->mileage_total  >= static_pot )//到达位置释放
            {
                Chassis_process.Derection_flag = -1;//向左跑轨
            }
        }//右
    }
    Chassis_AUTOcontrol_2_0();
}

/**
* @brief 自动跑轨3.0
* @param void
* @return void
*前哨战存活时于轨道右侧停止移动，前哨战被击毁后较高速全轨巡航，进入打击模式后根据自身状态决定跑轨策略
*/
static void Chassis_AUTO_3_0()
{
    Outpost_get();//获取前哨站状态，改变底盘模式
    if(Chassis_process.init_flag)
    {
        if(Chassis_process.Fire == FIRE_ALL)
        {
            Static_shoot();
        } else
        {
            Chassis_GetMode(); //获取状态
            if( Chassis_process.Mode == CHASSIS_NORMAL)
            {
                Cruise_Normal();
            }
            else {
                Show_Time();
            }
        }
        Chassis_Rebound();
    }
    else
    {
        Cruise_First();
    }
    Chassis_AUTOcontrol_3_0();
}

/**
* @brief 自检控制
* @param void
* @return void
*
*/
static void Chassis_INSPECTIONcontrol()
{
    Chassis_process.PPM.kp = 1;
    Chassis_process.PPM.out_max = 3000;

    Chassis_process.PPM.target = Chassis_process.Spot_taget;
    Chassis_process.PPM.measure = path_sensor.info->mileage_total;
    pid_calculate(&Chassis_process.PPM);

    if( abs(Chassis_process.Spot_taget - path_sensor.info->mileage_total) < 100 )
    {
        Chassis_process.Spot_taget = path_sensor.info->mileage_total;
//            Chassis_process.Speed_taget = Chassis_process.PPM.out;
        Chassis_process.Speed_taget = 0;
    } else
    {
        Chassis_process.Speed_taget = Chassis_process.PPM.out;

        if( (Chassis_process.Spot_taget - path_sensor.info->mileage_total) > 0 )
        {
            Chassis_process.Derection_flag = 1;
        } else if( (path_sensor.info->mileage_total - Chassis_process.Spot_taget) > 0 )
        {
            Chassis_process.Derection_flag = -1;
        }

        if(Chassis_process.Derection_flag == 1)//向右
        {
            Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, 0, 2000);
        } else if(Chassis_process.Derection_flag == -1)//向左
        {
            Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -2000, 0);
        }//加速度偏置，防止速度太慢停下来

    }

    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);

    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief 底盘自检
* @param void
* @return void
*底盘自检，用于检查微动开关和编码器是否正常，包含控制流程
*/
static void Chassis_INSPECTION()
{
    static bool right_judge,left_judge;

    if(path_sensor.info->left_touch)//碰到左开关
    {
        left_judge = true;
    }
    if(path_sensor.info->right_touch)//碰到右开关
    {
        right_judge = true;
    }

    if(left_judge == true)
    {
        if(path_sensor.info->left_touch == false)//左开关松开
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total + 4000;
            left_judge = false;
        }
    } else if(right_judge == true)
    {
        if(path_sensor.info->right_touch == false)//右开关松开
        {
            Chassis_process.Spot_taget = path_sensor.info->mileage_total - 4000;
            right_judge = false;
        }
    }
    Chassis_INSPECTIONcontrol();
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

/**
* @brief 自动模式获取
* @param void
* @return void
*用于决定使用哪种自动模式，以遥控器控制
*/
static void Chassis_AutoWayGet()
{
    if(rc_sensor.info->s1 == 3)
        Chassis_process.Way = WAY_NORMAL;
    else if(rc_sensor.info->s1 == 1)
        Chassis_process.Way = WAY_TOUCH;
    else if(rc_sensor.info->s1 == 2)
        Chassis_process.Way = WAY_ENCODER;
}

/**
* @brief 自动模式获取
* @param void
* @return void
*用于决定使用哪种自动模式，以遥控器控制
*/
static void Aerial_Control()
{
    static int8_t record_dir;

    Chassis_process.swerve_judge = false;
    Chassis_process.swerve_flag = false;//清空巡航模式时的反弹动作
	
	Chassis_process.static_want = false;//清除静止标志位

    if(judge_sensor.info->AerialData.control_dir != record_dir)
    {
        if(judge_sensor.info->AerialData.control_dir == -1)
            Chassis_process.Speed_taget = -2000;
        else if(judge_sensor.info->AerialData.control_dir == 1)
            Chassis_process.Speed_taget = 2000;
    }

    if(judge_sensor.info->AerialData.control_dir == -1)
    {
        Chassis_process.Speed_taget -= 5;
    } else if(judge_sensor.info->AerialData.control_dir == 1)
    {
        Chassis_process.Speed_taget += 5;
    }

    record_dir = judge_sensor.info->AerialData.control_dir;

    Chassis_process.Speed_taget = constrain(Chassis_process.Speed_taget, -SPEED_MAX, SPEED_MAX);

    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    NormalData_0x200[0] = Chassis_process.PVM.out;
}

/**
* @brief 所有自动模式的集合
* @param void
* @return void
*包含纯自动和云台手操控的内容
*/
static void AUTO_ALL()
{
    if( Aerial_Control_info() &&  Chassis_process.init_flag )
    {
        Aerial_Control();
    } else
    {
//        Chassis_AutoWayGet();
//        if( Chassis_process.Way == WAY_NORMAL )
//            Chassis_AUTO_3_0();	//混合跑轨
//        else if( Chassis_process.Way == WAY_TOUCH )
//            Chassis_AUTO_TOUCH(); //纯触碰跑轨
//        else if( Chassis_process.Way == WAY_ENCODER )
//            Chassis_AUTO_ENCODER();//纯编码器跑轨

                Chassis_AUTO();//遥控
    }
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
    Chassis_process.PPM.kp = 0.4;
    Chassis_process.PPM.ki = 0;//0.001
    Chassis_process.PPM.kd = 0;
    Chassis_process.PPM.integral_max = 1600;
    Chassis_process.PPM.out_max = 5000;

    Chassis_process.PVM.kp = 12;//10
    Chassis_process.PVM.ki = 0.05;//0.02
    Chassis_process.PVM.kd = 0;
    Chassis_process.PVM.integral_max = 8000;
    Chassis_process.PVM.out_max = 12000;
    Chassis_process.init_flag = false;
    Chassis_process.Derection_flag = -1;//初始向左
    Chassis_process.rotate_ratio = 8;
    Chassis_process.Trip_times = 1;

    Chassis_process.Fire = FIRE_ALL;;
}
/**
* @brief 底盘任务
* @param void
* @return void
* @berif  2ms
*/
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
                AUTO_ALL();
            } else if(sys.remote_mode == INSPECTION)
            {
                Chassis_INSPECTION();
            }
            Chassis_Power_Control();
        } else
        {
            Chassis_process.init_flag = false;
        }
        osDelay(2);
    }
}


