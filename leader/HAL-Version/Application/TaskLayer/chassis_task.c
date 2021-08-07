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
#define High_Speed 3600  //3600 �� 4600 ,3000
#define Normal_Speed 2500  //�Կ���3000,������2000
#define Attack_Speed 1200  //
#define Cover_Speed  2500
#define Escape_Speed 3200
#define First_Speed  -1000
#define High_Swerve_Ratio 0.6  //0.6
#define Normal_Swerve_Ratio 0.2  //�Կ���0.1
#define Atrip_Num 10.f  //����ֶ���
#define Atrip_Err 100  //��Χ
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
* @brief ��ʼѲ��
* @param void
* @return void
* ���ڵ�һ���ܹ�ʱ�Թ�����ȵ�У�����ܹ��ٶȽ�����һ����3����׼�������
*/
static void Cruise_First()
{
    static uint8_t step=0;	/*��ʼ������*/
    if(Chassis_process.init_flag == false)		/*δ�������ɳ�ʼ��*/
    {
        switch(step)
        {
        case 0:			/*��ʼ����������߿�*/
        {
            if(path_sensor.info->left_touch)		/*��ߵĵ㴥����*/
            {
                Chassis_process.Speed_taget=0;
                step=1;
            }
            else	/*δʶ�� -- �����˶�*/
            {
                Chassis_process.Speed_taget = -1000;	/*�����˶�*/
            }
            break;
        }
        case 1:
        {
            if(path_sensor.info->left_touch)
            {
                Chassis_process.Speed_taget=100;	/*����΢�������ڵ��պò������㴥���ص�λ��*/
            }
            else			/*΢����ɺ���ͣס��������һ�����ڽ׶�*/
            {
                Chassis_process.Speed_taget=0;
                path_sensor.info->mileage_total=0;	/*�������� -- ��ʼ��¼*/
                step=2;
            }
            break;
        }
        case 2:
        {
            if(path_sensor.info->right_touch)	/*�Ѿ�ʶ���ҹ��*/
            {
                Chassis_process.Speed_taget=0;
                step=3;
            }
            else 		/*δʶ��*/
            {
                Chassis_process.Speed_taget=1000;			/*�����˶�*/
            }
            break;
        }
        case 3:
        {
            if(path_sensor.info->right_touch)
            {
                Chassis_process.Speed_taget=-100;		/*�Ѿ�ʶ���ҵ㴥���أ�����΢�����պ�δʶ�𵽵�״̬*/
            }
            else			/*΢�����*/
            {
                Chassis_process.Speed_taget=0;															/*ֹͣ�˶�*/
                Chassis_process.Mileage_atrip=path_sensor.info->mileage_total;	/*��¼�������*/
                step=0;																							/*��λstep*/
                Chassis_process.init_flag=true;											/*���±�־λ*/
                Chassis_process.Derection_flag=-1;//�����˶���־λ
            }
            break;
        }
        }
    }
}

/**
  * @brief  Ѫ����麯��
	* @note   ÿ500ms���һ��Ѫ���仯����Ѫ������
	*					˵��װ���ܵ��˺�
  * @params  void
  * @retval bool��ֵΪ1��ʾ�ܵ��˺���ֵΪ0��ʾδ�ܵ��˺�
  */
bool HP_Check(void)
{
    static  uint16_t last_HP = 600;
    static  uint16_t cur_HP = 0;
    static  int16_t delta_HP;
    static  uint32_t HP_check_time = 1;
    static  bool result = 0;

    HP_check_time++;

    if(HP_check_time % 100 == 0)                        //ÿ200ms���һ��Ѫ���仯ֵ
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_HURT && delta_HP > -60)      //ÿ200ms���һ��Ѫ���仯ֵ
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

    if(HP_check_time % 100 == 0)                        //ÿ200ms���һ��Ѫ���仯ֵ
    {
        cur_HP = judge_sensor.info->GameRobotStatus.remain_HP;
        delta_HP = cur_HP - last_HP;
        last_HP = cur_HP;
        if(delta_HP <= HP_VERYHURT)                        //ÿ200ms���һ��Ѫ���仯ֵ
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
  * @brief  �ܹ����жϺ���
	* @note �ܵ�����3�Σ��ж�Ϊ����Σ��״̬��
	*					��6s����û���ܵ����������밲ȫ
	*					״̬��ÿ���������г��򣬵�һ����
	*					��2�ι����Ż����Σ��״̬������
	*					ÿ�ܵ�1�ι����ͻ����Σ��״̬��
  * @params ishurt���Ƿ��ܵ�����
  * @retval bool��ֵΪ1��ʾ����Σ��״̬��ֵΪ0��ʾδ���밲ȫ״̬
  */
bool IS_Danger(bool ishurt, uint8_t hurt_times)
{
    static bool hurtflag = 0;
    static bool result = 0;
    static uint32_t safe_cnt,hurt_cnt;

    if(ishurt)                //����ܵ�����
    {
        safe_cnt = 0;
        if(hurtflag != 1)
        {
            hurtflag = 1;
            hurt_cnt++;           //��¼�ܵ��˺�����
            if(hurt_cnt >= hurt_times)     //�ܵ�n���˺����ϣ��ж�Ϊ����Σ��
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

    if(ishurt)                //����ܵ�����
    {
        safe_cnt = 0;
        if(hurtflag != 1)
        {
            hurtflag = 1;
            hurt_cnt++;           //��¼�ܵ��˺�����
            if(hurt_cnt >= hurt_times)     //�ܵ�n���˺����ϣ��ж�Ϊ����Σ��
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
* @brief ������ʼ���ж�
* @param bool
* @return bool
* �ֱ��ж��Ƿ�ﵽ����������ʼ��
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
* @brief ����ж�
* @param bool
* @return bool
* �ж��Ƿ񵽴����е㡢���Ρ��Ұ��
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
* @brief �ķ�֮һ���ж�
* @param bool
* @return bool
* �ֱ��ж��Ƿ�ﵽ�ķ�֮һ��
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
* @brief �����ж�
* @param bool
* @return bool
* �ж��Ƿ��Ѫ����Ѫ����ȫ���ܹ�
*/
static bool Is_TimeToRun()
{
    if(judge_sensor.info->GameRobotStatus.remain_HP <= HP_Danger)
        return true;
    else
        return false;
}

/**
* @brief λ���ж�
* @param bool
* @return bool
* �жϵ�ǰλ���ĶΣ����η���1���Ұ�η���2
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
* @brief �����ж�
* @param bool
* @return bool
* �ж����Ŀ��λ���Ƿ񵽴�
*������������ʾ����ֶζ���λ��
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
* @brief �ٶ�����
* @param void
* @return void
* ��������޸ĵ����ٶȣ���������·�̵ı���
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
* @brief ȫ��Ѳ��
* @param void
* @return void
* ��ɹ����ʼУ׼���Ѳ����������һ���ٶ�
* -1Ϊ��1Ϊ��
*/
static void Cruise_Normal()
{
    Chassis_process.getchange_flag = true;//�������ʱ��ȡλ��

    if(Chassis_process.Derection_flag == -1)
    {
//        if( Is_LeftSwerve_Spot()&& (!Chassis_process.overbuff_flag) )
//            Chassis_process.swerve_flag = true;//�����˶�����ж����
//        else
//            Chassis_process.swerve_flag = false;

        if(path_sensor.info->left_touch)//�����˶�ʱ�����㴥����
        {
            Chassis_process.swerve_judge = true;//��ʼ�����ж�
            Chassis_process.swerve_flag = true;
        }

        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->left_touch == false)//�������㴥�����ͷ�
            {
                Chassis_process.Trip_times ++;     //��¼��������
                Chassis_process.Derection_flag = 1;//�����ܹ�
                path_sensor.info->mileage_total = 0;//��������
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//�������
            }
        }
    }//��

    if(Chassis_process.Derection_flag == 1)
    {
//        if( Is_RightSwerve_Spot()&& (!Chassis_process.overbuff_flag) )
//            Chassis_process.swerve_flag = true;//�����˶�����ж����,��û�����ʿ���
//        else
//            Chassis_process.swerve_flag = false;

        if(path_sensor.info->right_touch)//�����˶�ʱ�����㴥����
        {
            Chassis_process.swerve_judge = true;//��ʼ�����ж�
            Chassis_process.swerve_flag = true;
        }
        if(Chassis_process.swerve_judge)
        {
            if(path_sensor.info->right_touch == false)//�������㴥�����ͷ�
            {
                Chassis_process.Derection_flag = -1;//�����ܹ�
                Chassis_process.Mileage_atrip = path_sensor.info->mileage_total;//��¼��������
                Chassis_process.swerve_judge = false;
                Chassis_process.swerve_flag = false;//�������
            }
        }
    }//��
}

/**
* @brief ��������
* @param void
* @return void
* �����������շ�Χ�Ĵ���
*/
static void Chassis_Rebound()
{
    if(Chassis_process.swerve_judge)
    {
        Chassis_process.swerve_flag = true; //����ʱ��־λ
    }
    else//û�з���������Ѳ��
    {
        Chassis_process.Speed_taget = Chassis_process.Speed_taget;
    }
}

/**
* @brief ң�ؿ���
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
  * @brief 	���̵����ֹ������
	*					�����ڵ㴥����ʧ����߱��߿���
	*					���һ��ʱ��ִ�е�������һ����
	*					���ƶ����ܵ������һ��һ�����
	*					����У׼������λ�ú�������С�
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
* @brief �����ܹ�
* @param void
* @return void
* ֱ�ӷ���
*/
static void Chassis_Change_Dir(void)
{
    Chassis_process.Derection_flag = -Chassis_process.Derection_flag;

}

/**
* @brief �����ٶ�
* @param void
* @return void
* �Զ������ܹ���ٶȻ�ȡ
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
* @brief �������
* @param void
* @return void
* �Զ������ܹ�ľ����ȡ,�����ϴ�Ŀ�����ٵ���
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
    dis = (rand() % remain) + 1;//����1��9 �������

    return dis;
}

/**
* @brief �Զ��ܹ����
* @param void
* @return void
* ��������ʱֱ�������Ϊ0
*/
static void Chassis_AUTOcontrol()
{
    if(Chassis_process.init_flag)
    {
        Chassis_Speed_Set(); //�趨�ٶ�
        Chassis_Stuck_Handle();
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.target = 0;
        Chassis_process.PVM.out = 0;  //����ʱ���Ϊ0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}

/**
* @brief �Զ��ܹ�
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
* @brief ����״̬��ȡ
* @param void
* @return void
* ��������л�ģʽ
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
* @brief ��������
* @param void
* @return void
*/
int8_t Aim_dis;
static void Show_Time()
{
    if(Chassis_process.getchange_flag)
    {
        Chassis_Change_Dir(); //����
        Aim_dis = Chassis_Change_Dis() * Chassis_process.Derection_flag ;//��ȡ�����������
        Chassis_process.Spot_taget = path_sensor.info->mileage_total + (int32_t)(Chassis_process.Mileage_atrip * Aim_dis/Atrip_Num);

        Chassis_process.Spot_taget = constrain(Chassis_process.Spot_taget, 4000,Chassis_process.Mileage_atrip - 4000);   //���Ը����������ͬ����λ

        Chassis_process.getchange_flag = false;
    }
    if( Is_SpotArrive(Chassis_process.Spot_taget) )
    {
        Chassis_process.getchange_flag = true;  //�����ٽ����ȡ�������
    }


}
/**
* @brief �Զ��ܹ����2.0
* @param void
* @return void
* �������
*/
static void Chassis_AUTOcontrol_2_0()
{
    if(Chassis_process.init_flag)
    {
        Chassis_Change_Speed();
//        Chassis_Speed_Set(); //�趨�ٶ�
        Chassis_Stuck_Handle();
    }
    Chassis_process.PVM.target = Chassis_process.Speed_taget;
    Chassis_process.PVM.measure = motor[CHASSIS].info->speed;
    pid_calculate(&Chassis_process.PVM);
    if(Chassis_process.swerve_flag)
    {
        Chassis_process.PVM.target = 0;
        Chassis_process.PVM.out = 0;  //����ʱ���Ϊ0
    }
    NormalData_0x200[0] = (int16_t)Chassis_process.PVM.out;
}
/**
* @brief �Զ��ܹ�2.0
* @param void
* @return void
*�����˱����ܹ�
*/
static void Chassis_AUTO_2_0()
{
//    Chassis_GetMode(); //��ȡ״̬
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
* @brief ���̵Ĺ��ʻ�ģʽ����
* @param void
* @return void
* @berif  K_limit = (Jʣ��/J���)��  Out_final = K_limit^2*Out_PID
*/
float K_limit = 1;
static void Chassis_Power_Control()
{
    static int16_t Cold_time = 0;
    if(judge_sensor.info->power_heat_update) //�������ݸ�������ʱ���й��ʿ��ƣ�����Ƶ��50hz
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
//	else  //û��������
//	{
//		Chassis_process.PVM.out_max = PowerLimit_Normal;
//		//K_limit = 1;
//	}
}
/* Exported functions --------------------------------------------------------*/

/**
* @brief ���̲�����ʼ��
* @param void
* @return void
* @berif  ֱ��һ��
*/
void Chassis_Init()
{
    Chassis_process.PVM.kp = 10;//18
    Chassis_process.PVM.ki = 0.02;//0.07
    Chassis_process.PVM.kd = 0;
    Chassis_process.PVM.integral_max = 8000;
    Chassis_process.PVM.out_max = 12000;
    Chassis_process.init_flag = false;
    Chassis_process.Derection_flag = -1;//��ʼ����
    Chassis_process.rotate_ratio = 8;
    Chassis_process.Trip_times = 1;
}
/**
* @brief ��������
* @param void
* @return void
* @berif  2ms
*/
uint8_t dis;
void StartChassisTask(void const * argument)
{
    for(;;)
    {
		    dis = (rand() % 3) + 1;//����1��9 �������
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

