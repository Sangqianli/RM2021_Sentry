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
#define ACTIVE_MAX_CNT  1
#define LOST_MAX_CNT    2	/*对于识别和丢失判定的阈值*/
#define CONVER_SCALE_YAW    22.463f//20.86
#define CONVER_SCALE_PITCH  22.26f//22.9
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float vision_mach_yaw,vision_mach_pitch,vision_dis_meter;//视觉数据转换

extKalman_t kalman_visionYaw,kalman_targetYaw,kalman_visionPitch,kalman_targetPitch,kalman_visionDistance,kalman_targetDistance;
extKalman_t kalman_accel,kalman_speedYaw;
float visionYaw_R=0,targetYaw_R=400,visionPitch_R=0,targetPitch_R=100,visionDis_R=0,targetDis_R=100;//1,1000,1,1000
float predictAccel_R=1,speedYaw_R=200;//100 400

uint8_t Vision_SentData[60];//发送给视觉的数组

float YawTarget_now,PitchTarget_now;//实际视觉给定角度
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*记录视觉更新数据时的云台数据，给下次接收用*/
float lastupdate_cloud_yaw = 0,lastupdate_cloud_pitch=0; /*前两帧的数据*/
/* Exported variables --------------------------------------------------------*/
Vision_process_t Vision_process;
/* Private functions ---------------------------------------------------------*/
static void Sent_to_Vision_Version2_1()
{
    static uint8_t Sent_cnt=0;//发送间隔
    static uint32_t now_time;
    static uint8_t  colour;
//    uint8_t *time;
//    now_time=xTaskGetTickCount();
//    time=(uint8_t*)&now_time;

    if(master_sensor.info->modes.attack_colour == 1)//红色id
        colour = 1;
    else if(master_sensor.info->modes.attack_colour == 2)//蓝色id
        colour = 0;

    Append_CRC8_Check_Sum(Vision_SentData, 3);
    Append_CRC16_Check_Sum(Vision_SentData,22);

    Vision_SentData[0] = 0xA5;
    Vision_SentData[1] = 0;
    /*小端发送，高字节是高位*/
    Vision_SentData[3] = colour;//颜色识别，1是蓝色，0是红色
    Vision_SentData[4] = 0;//1反陀螺，0关反陀螺

    Sent_cnt++;
    if(Sent_cnt>=100)
    {
        UART1_SendData(Vision_SentData,23);
        Sent_cnt=0;
    }
}

static void Offset_Angle_Get()
{
    if(Vision_process.data_kal.DistanceGet_KF > 1.25f && Vision_process.data_kal.DistanceGet_KF <= 3.25f)
        Vision_process.offset_pitch = 1.6;
    else if(Vision_process.data_kal.DistanceGet_KF > 3.25f && Vision_process.data_kal.DistanceGet_KF <= 4.25f)
        Vision_process.offset_pitch = 1.83;
    else if(Vision_process.data_kal.DistanceGet_KF > 4.25f && Vision_process.data_kal.DistanceGet_KF <= 5.25f)
        Vision_process.offset_pitch = 2.4f;
    else if(Vision_process.data_kal.DistanceGet_KF > 5.25f && Vision_process.data_kal.DistanceGet_KF <= 6.f)
        Vision_process.offset_pitch = 2.6;
    else
        Vision_process.offset_pitch = 0;

    Vision_process.offset_yaw = 0.2;//0.8
}

static void Offset_Angle_Get_2_1()
{
    if(Vision_process.data_kal.DistanceGet_KF > 3.3f && Vision_process.data_kal.DistanceGet_KF <= ANTI_DISTANDCE)// <= 6.25f
        Vision_process.offset_pitch = 0.5459f * Vision_process.data_kal.DistanceGet_KF - 3.829f + 0.6f;//最后一个是手动补偿
    else
        Vision_process.offset_pitch = 0;

    Vision_process.offset_yaw = 2.3f;
}

static void Vision_Normal()
{
    static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/
    static int16_t  Record_Auto_Mode = AUTO_MODE_SCOUT;

    Offset_Angle_Get_2_1();//根据距离获取补偿角

    if(vision_sensor.info->State.rx_data_update)//数据更新
    {
        if(Record_Auto_Mode != sys.auto_mode)
        {
            sys.switch_state.AUTO_MODE_SWITCH = true;
        }
        Record_Auto_Mode = sys.auto_mode;

        if( (vision_sensor.info->RxPacket.RxData.identify_target == 1 )&& ( (vision_dis_meter>0.3f) && (vision_dis_meter < ANTI_DISTANDCE) ) )
        {
            active_cnt++; 	/*活跃计数*/
            if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
            {
                sys.auto_mode = AUTO_MODE_ATTACK;
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
                active_cnt = 0;
                lost_cnt = 0;
                /*侦察模式的切换*/
                Clear_Queue(&Vision_process.speed_queue);
                Clear_Queue(&Vision_process.accel_queue);
                Clear_Queue(&Vision_process.dis_queue);
                /*清除队列信息，防止下一次数据受到影响*/
                Vision_process.data_kal.DistanceGet_KF = 0;//正确更新距离
                Vision_process.feedforwaurd_angle = 0;
                Vision_process.predict_angle = 0;//清0预测角
                sys.predict_state.PREDICT_OPEN = false;			/*关闭预测*/
            }
        }

//        if(vision_sensor.info->RxPacket.RxData.anti_gyro == 1)
//        {
//            Vision_process.gyro_anti = true;
//        }
//        else
//        {
            Vision_process.gyro_anti = false;
//            Vision_process.gyro_judge = true;  //下次再进入反陀螺时的判定
//        }

        if(Vision_process.gyro_anti == false) //没有陀螺的时候进行常规给定
        {
            vision_mach_yaw = (vision_sensor.info->RxPacket.RxData.yaw_angle + Vision_process.offset_yaw) * CONVER_SCALE_YAW;
            vision_mach_pitch = (vision_sensor.info->RxPacket.RxData.pitch_angle + Vision_process.offset_pitch) * CONVER_SCALE_PITCH;		//加上补偿角，转换成机械角度
            vision_dis_meter =  vision_sensor.info->RxPacket.RxData.distance/1000.f;

//            vision_mach_yaw  = 	DeathZoom(vision_mach_yaw,0,0.1);
//            vision_mach_pitch=  DeathZoom(vision_mach_pitch,0,0.1);
        }
        Vision_process.data_kal.YawGet_KF = KalmanFilter(&kalman_visionYaw,vision_mach_yaw); 	/*对视觉角度数据做卡尔曼滤波*/
        Vision_process.data_kal.PitchGet_KF = KalmanFilter(&kalman_visionPitch,vision_mach_pitch);
        if( (vision_dis_meter>0.3f) && (vision_dis_meter<10.f) )
            Vision_process.data_kal.DistanceGet_KF =KalmanFilter(&kalman_visionDistance,vision_dis_meter);

        YawTarget_now=lastupdate_cloud_yaw+Vision_process.data_kal.YawGet_KF;
        PitchTarget_now=lastupdate_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

        lastupdate_cloud_yaw = update_cloud_yaw;
        lastupdate_cloud_pitch = update_cloud_pitch;      /*记录前2帧的数据*/
        update_cloud_yaw = Gimbal_process.YAW_PPM.measure;/*视觉数据更新时的云台角度*/
        update_cloud_pitch = Gimbal_process.PITCH_PPM.measure;

        /*视觉数据推演.......................................*/
        Vision_process.speed_get = Get_Diff(15,&Vision_process.speed_queue,YawTarget_now);//20
        Vision_process.speed_get = KalmanFilter(&kalman_speedYaw,Vision_process.speed_get);
//        Vision_process.speed_get = DeathZoom(Vision_process.speed_get,0,1);
//		Vision_process.speed_get = constrain(Vision_process.speed_get , -60 , 60);

        Vision_process.accel_get = Get_Diff(5,&Vision_process.accel_queue,Vision_process.speed_get);	 /*新版获取加速度10*/
        Vision_process.accel_get = KalmanFilter(&kalman_accel,Vision_process.accel_get);
//        Vision_process.accel_get = DeathZoom(Vision_process.accel_get,0,0.1);		/*死区处理 - 滤除0点附近的噪声*/

        Vision_process.distend_get =  Get_Diff(5,&Vision_process.dis_queue,Vision_process.data_kal.DistanceGet_KF);
        Vision_process.distend_get =  DeathZoom(Vision_process.distend_get,0,0.01);
        /*......................................................*/
        vision_sensor.info->State.rx_data_update = false;
    }

}

static void Vision_Pridict()
{
    static float acc_use = 1.f;
    static float predic_use = 2.f;//1.5
    float dir_factor;
    if( (Vision_process.speed_get * Vision_process.accel_get)>=0 )
    {
        dir_factor= 1.f;
    }
    else
    {
        dir_factor= 1.5f;
    }

    Vision_process.feedforwaurd_angle = acc_use * Vision_process.accel_get; 	/*计算前馈角*/

    Vision_process.predict_angle = predic_use * (1.f*Vision_process.speed_get*Vision_process.data_kal.DistanceGet_KF+3.f*dir_factor*Vision_process.feedforwaurd_angle*Vision_process.data_kal.DistanceGet_KF) ;//速度1.1，加速度0.8
}
static void AntiNormal()
{
    /*直接跟随给定.................................*/
//	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
//	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/

    Vision_process.data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,Vision_process.data_kal.YawTarget_KF);
//    Vision_process.data_kal.PitchTarget_KF=PitchTarget_now+10*Vision_process.distend_get;                               //在这里给pitch轴预测的系数
//    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,Vision_process.data_kal.PitchTarget_KF);
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);//不给预测
}
static void AntiGyro()
{
    static float anti_yaw, anti_pitch, record_yaw, record_pitch;
    static bool judge_flag = true;
    anti_yaw = (vision_sensor.info->RxPacket.RxData.yaw_angle + Vision_process.offset_yaw) * CONVER_SCALE_YAW;
    anti_pitch = (vision_sensor.info->RxPacket.RxData.pitch_angle + Vision_process.offset_pitch) * CONVER_SCALE_PITCH;

    if(judge_flag)
    {
        Vision_process.data_kal.YawTarget_KF = anti_yaw + update_cloud_yaw;
        Vision_process.data_kal.PitchTarget_KF = anti_pitch + update_cloud_pitch;
        Clear_Queue(&Vision_process.speed_queue);
        Clear_Queue(&Vision_process.accel_queue);
        Clear_Queue(&Vision_process.dis_queue);
        judge_flag = false;
    }
    if( (abs(record_yaw-anti_yaw)<0.1f) && (abs(record_pitch-anti_pitch)<0.1f) )
    {
        judge_flag = false;
    } else
    {
        judge_flag = true;
    }

    record_yaw = anti_yaw;
    record_pitch = anti_pitch;
}

static void Anti_Target()
{
//    if(Vision_process.gyro_anti)
//        AntiGyro();
//    else
        AntiNormal();
}

/**
* @brief 雷达站信息处理及给定
* @param void
* @return void
*/
static void Vision_Radar()
{
    static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/
    if( judge_sensor.info->RadarData.identify_target == 1 )
    {
        active_cnt++; 	/*活跃计数*/
        if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
        {
            sys.auto_mode = AUTO_MODE_ATTACK;
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
            active_cnt = 0;
            lost_cnt = 0;
            /*侦察模式的切换*/
            /*清除队列信息，防止下一次数据受到影响*/
            Vision_process.data_kal.DistanceGet_KF = 0;//正确更新距离
            Vision_process.feedforwaurd_angle = 0;
            Vision_process.predict_angle = 0;//清0预测角
            sys.predict_state.PREDICT_OPEN = false;			/*关闭预测*/
        }
    }
    vision_mach_yaw = (judge_sensor.info->RadarData.yaw_angle + 0) * CONVER_SCALE_YAW;
    vision_mach_pitch = (judge_sensor.info->RadarData.pitch_angle + 0) * CONVER_SCALE_PITCH;		//加上补偿角，转换成机械角度
    vision_dis_meter =  judge_sensor.info->RadarData.distance/1000.f;

    vision_mach_yaw  = 	DeathZoom(vision_mach_yaw,0,4);
    vision_mach_pitch=  DeathZoom(vision_mach_pitch,0,2);

    Vision_process.data_kal.YawGet_KF = KalmanFilter(&kalman_visionYaw,vision_mach_yaw); 	/*对视觉角度数据做卡尔曼滤波*/
    Vision_process.data_kal.PitchGet_KF = KalmanFilter(&kalman_visionPitch,vision_mach_pitch);

    YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
    PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

    update_cloud_yaw = Gimbal_process.YAW_PPM.measure;/*视觉数据更新时的云台角度*/
    update_cloud_pitch = Gimbal_process.PITCH_PPM.measure;

    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
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

    Vision_process.speed_queue.queueLength = 60;
    Vision_process.accel_queue.queueLength = 60;
    Vision_process.dis_queue.queueLength = 60;

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
            Anti_Target();
            Sent_to_Vision_Version2_1();
        }
        osDelay(2);
    }
}
