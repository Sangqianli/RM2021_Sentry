/**
 * @file        monitor_task.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-November-2020
 * @brief       Monitor&Test Center
 */

/* Includes ------------------------------------------------------------------*/
#include "vision_task.h"
#include "device.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define ACTIVE_MAX_CNT  2
#define LOST_MAX_CNT    10	/*对于识别和丢失判定的阈值*/
#define CONVER_SCALE_YAW    21.1f//20.86
#define CONVER_SCALE_PITCH  22.9f
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float vision_mach_yaw,vision_mach_pitch,vision_dis_meter;//视觉数据转换

extKalman_t kalman_visionYaw,kalman_targetYaw,kalman_visionPitch,kalman_targetPitch,kalman_visionDistance,kalman_targetDistance;
extKalman_t kalman_accel,kalman_speedYaw;
float visionYaw_R=1,targetYaw_R=2000,visionPitch_R=0.5,targetPitch_R=3000,visionDis_R=1,targetDis_R=1000;//0.2,2000,0.5,3000
float predictAccel_R=100,speedYaw_R=400;//400,1200

uint8_t Vision_SentData[60];//发送给视觉的数组

float YawTarget_now,PitchTarget_now;//实际视觉给定角度
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*记录视觉更新数据时的云台数据，给下次接收用*/
/* Exported variables --------------------------------------------------------*/
Vision_process_t Vision_process;
/* Private functions ---------------------------------------------------------*/
static void Sent_to_Vision_Version2_1()
{
    static uint8_t Sent_cnt=0;//发送间隔
    static uint32_t now_time;
    uint8_t *time;
    now_time=xTaskGetTickCount();
    time=(uint8_t*)&now_time;

    Append_CRC8_Check_Sum(Vision_SentData, 3);
    Append_CRC16_Check_Sum(Vision_SentData,22);

    Vision_SentData[0]=0xA5;
    Vision_SentData[1]=1;
    /*小端发送，高字节是高位*/
    Vision_SentData[3]= *time;
    Vision_SentData[4]=*(time+1);
    Vision_SentData[5]=*(time+2);
    Vision_SentData[6]=*(time+3);//时间数据

    Sent_cnt++;
    if(Sent_cnt>=100)
    {
        UART1_SendData(Vision_SentData,23);
        Sent_cnt=0;
    }
}

static void Vision_Normal()
{
    static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/
    if(vision_sensor.info->State.rx_data_update)//数据更新
    {
        if(vision_sensor.info->RxPacket.RxData.identify_target == 1)//识别到目标
        {
            active_cnt++; 	/*活跃计数*/
            if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
            {
                sys.auto_mode = AUTO_MODE_ATTACK;
                sys.switch_state.AUTO_MODE_SWITCH = true;
                active_cnt = 0;
                lost_cnt = 0;
                /*重新确认进来的判断*/
            }
        }
        else
        {
            lost_cnt++;
            if(lost_cnt >= LOST_MAX_CNT) /*达到阈值，认定为丢失*/
            {
                sys.auto_mode = AUTO_MODE_SCOUT;
                sys.switch_state.AUTO_MODE_SWITCH = true;
                active_cnt = 0;
                lost_cnt = 0;
                /*侦察模式的切换*/
                Clear_Queue(&Vision_process.speed_queue);
                Clear_Queue(&Vision_process.accel_queue);
                Clear_Queue(&Vision_process.dis_queue);
                /*清除队列信息，防止下一次数据受到影响*/
                Vision_process.predict_angle = 0;//清0预测角
                sys.predict_state.PREDICT_OPEN = false;			/*关闭预测*/
            }
        }

        vision_sensor.info->RxPacket.RxData.yaw_angle = DeathZoom(vision_sensor.info->RxPacket.RxData.yaw_angle,0,0.2);
        vision_sensor.info->RxPacket.RxData.pitch_angle = DeathZoom(vision_sensor.info->RxPacket.RxData.pitch_angle,0,0.1);

        vision_mach_yaw = vision_sensor.info->RxPacket.RxData.yaw_angle*CONVER_SCALE_YAW;
        vision_mach_pitch = vision_sensor.info->RxPacket.RxData.pitch_angle*CONVER_SCALE_PITCH;		//转换成机械角度
        vision_dis_meter =  vision_sensor.info->RxPacket.RxData.distance/1000.f;

        vision_mach_yaw  = 	DeathZoom(vision_mach_yaw,0,6);
        vision_mach_pitch=  DeathZoom(vision_mach_pitch,0,3);

        Vision_process.data_kal.YawGet_KF = KalmanFilter(&kalman_visionYaw,vision_mach_yaw); 	/*对视觉角度数据做卡尔曼滤波*/
        Vision_process.data_kal.PitchGet_KF = KalmanFilter(&kalman_visionPitch,vision_mach_pitch);
        if(vision_dis_meter>0.0001f)
            Vision_process.data_kal.DistanceGet_KF =KalmanFilter(&kalman_visionDistance,vision_dis_meter);

        YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
        PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

        update_cloud_yaw=motor[GIMBAL_YAW].info->angle_sum;/*视觉数据更新时的云台角度*/
        update_cloud_pitch=motor[GIMBAL_PITCH].info->angle_sum;

        /*视觉数据推演.......................................*/
        Vision_process.speed_get = Get_Diff(20,&Vision_process.speed_queue,YawTarget_now);
        Vision_process.speed_get = KalmanFilter(&kalman_speedYaw,Vision_process.speed_get);
        Vision_process.speed_get = DeathZoom(Vision_process.speed_get,0,0.3);

        Vision_process.accel_get = Get_Diff(10,&Vision_process.accel_queue,Vision_process.speed_get);	 /*新版获取加速度*/
        Vision_process.accel_get = DeathZoom(Vision_process.accel_get,0,0.1);		/*死区处理 - 滤除0点附近的噪声*/
        Vision_process.accel_get = KalmanFilter(&kalman_accel,Vision_process.accel_get);

        Vision_process.distend_get =  Get_Diff(10,&Vision_process.dis_queue,Vision_process.data_kal.DistanceGet_KF);
        Vision_process.distend_get =  DeathZoom(Vision_process.distend_get,0,0.1);
        /*......................................................*/
        vision_sensor.info->State.rx_data_update = false;

    }

    /*直接跟随给定.................................*/
//	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
//	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/
}

static void Vision_Pridict()
{
    static float acc_use = 1.f;
    static float predic_use = 1.f;
    float dir_factor;
    if( (Vision_process.speed_get * Vision_process.accel_get)>=0 )
    {
        dir_factor= 1.f;
    }
    else
    {
        dir_factor= 1.8f;
    }

    Vision_process.feedforwaurd_angle = acc_use * Vision_process.accel_get; 	/*计算前馈角*/

    Vision_process.predict_angle = (0.8f*Vision_process.speed_get*Vision_process.data_kal.DistanceGet_KF+2.2f*dir_factor*Vision_process.feedforwaurd_angle) ;	/*计算预测角度*/

    Vision_process.data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle*predic_use;
    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,Vision_process.data_kal.YawTarget_KF);
    Vision_process.data_kal.PitchTarget_KF=PitchTarget_now+600*Vision_process.distend_get;
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,Vision_process.data_kal.PitchTarget_KF);
}
/* Exported functions --------------------------------------------------------*/
void Vision_Init()
{
    KalmanCreate(&kalman_visionYaw,1,visionYaw_R);
    KalmanCreate(&kalman_targetYaw,1,targetYaw_R);
    KalmanCreate(&kalman_visionPitch,1,visionPitch_R);
    KalmanCreate(&kalman_targetPitch,1,targetPitch_R);
    KalmanCreate(&kalman_visionDistance,1,visionDis_R);
    KalmanCreate(&kalman_targetDistance,1,targetDis_R);
    KalmanCreate(&kalman_accel,1,predictAccel_R);
    KalmanCreate(&kalman_speedYaw,1,speedYaw_R);
}

void StartVisionTask(void const * argument)
{
    for(;;)
    {
        if( (sys.state == SYS_STATE_NORMAL) && (sys.switch_state.ALL_READY) )
        {
            Vision_Normal();
            if(sys.predict_state.PREDICT_OPEN)
            {
                Vision_Pridict();
            }
            Sent_to_Vision_Version2_1();
        }
        osDelay(2);
    }
}
