#include "visual_process.h"
/*目标速度的队列数据*/
QueueObj target_speed=
{
    .nowLength =0,
    .queueLength = 100,
    .queue = {0},
    .queueTotal=0,
    .full_flag=0
};
/*目标加速度的队列数据*/
QueueObj target_accel=
{
    .nowLength =0,
    .queueLength = 100,
    .queue = {0},
    .queueTotal = 0,
    .full_flag=0
};
/*目标距离趋势的队列数据*/
QueueObj target_distance_tendency=
{
    .nowLength =0,
    .queueLength = 100,
    .queue = {0},
    .queueTotal = 0,
    .full_flag=0
};


extern bool vision_update_flag;
extern extVisionRecvData_t Vision_receive;//视觉传过来的结构体
extern u8 Vision_SentData[60];//发送给视觉的数组
extern CRUISE_Mode Cruise_mode;
const  uint16_t ACTIVE_MAX_CNT = 2,LOST_MAX_CNT = 10; 	/*对于识别和丢失判定的阈值*/
Visual_TypeDef VisualProcess;//处理后的数据结果结构体
float vision_yaw_raw=0,vision_pitch_raw=0,vision_dis_meter;//视觉数据传过来后换算
float target_yaw_raw=0,target_yaw_kf=0;	/*目标的yaw角原始数据,卡尔曼滤波数据*/	float target_speed_raw=0,target_speed_kf=0;	/*目标的yaw速度原始数据,卡尔曼滤波数据*/
float target_accel_raw=0,target_accel_kf=0;	 /*对目标加速度的预测变量*/
float target_dis_tend;//目标距离变换趋势
float predict_angle=0,k_pre=0.10f,predict_angle_raw=0,use_predic=0.9f;		/*yaw预测超前角度,预测参数0.1*/	float feedforward_angle=0,k_ff=1.f,use_ff=1; 	/*前馈角 -- 加速度*/

uint16_t vision_update_fps = 0; /*视觉数据更新的帧率*/
extKalman_t kalman_visionYaw,kalman_targetYaw,kalman_visionPitch,kalman_targetPitch,kalman_visionDistance,kalman_targetDistance;
extKalman_t kalman_accel,kalman_speedYaw;
float visionYaw_R=1,targetYaw_R=2000,visionPitch_R=0.5,targetPitch_R=3000,visionDis_R=1,targetDis_R=1000;//0.2,2000,0.5,3000
float predictAccel_R=100,speedYaw_R=400;//400,1200
void Vision_Kalman_Init()
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


void Sent_to_Vision()
{
    static u8 Sent_cnt=0;//发送间隔
    u8 *yw,*pw,*ys,*ps;
    static float Yaw_raw,Pitch_raw;
    static  float Yaw_speed,Pitch_speed;

    Yaw_raw=((Gimbal_yaw_PPM.PID_PPM.measure)/8191)*360;
    Pitch_raw=((Gimbal_pitch_PPM.PID_PPM.measure)/8191)*360;//电机角度转换
    Yaw_speed=(Gimbal_yaw_PVM.speed_get*2*3.1415/60);
    Pitch_speed=(Gimbal_pitch_PVM.speed_get*2*3.1415/60);


    yw=(u8*)&Yaw_raw;
    pw=(u8*)&Pitch_raw;
    ys=(u8*)&Yaw_speed;
    ps=(u8*)&Pitch_speed;

    Append_CRC8_Check_Sum(Vision_SentData, 3);
    Append_CRC16_Check_Sum(Vision_SentData,22);

    Vision_SentData[0]=0xA5;
    Vision_SentData[1]=1;
    /*小端发送，高字节是高位*/
    Vision_SentData[3]= *yw;
    Vision_SentData[4]=*(yw+1);
    Vision_SentData[5]=*(yw+2);
    Vision_SentData[6]=*(yw+3);//Yaw轴角度数据

    Vision_SentData[7]= *pw;
    Vision_SentData[8]=*(pw+1);
    Vision_SentData[9]=*(pw+2);
    Vision_SentData[10]=*(pw+3);//Pitch轴角度数据

    Vision_SentData[11]= *ys;
    Vision_SentData[12]=*(ys+1);
    Vision_SentData[13]=*(ys+2);
    Vision_SentData[14]=*(ys+3);//Yaw轴角速度数据

    Vision_SentData[15]= *ps;
    Vision_SentData[16]=*(ps+1);
    Vision_SentData[17]=*(ps+2);
    Vision_SentData[18]=*(ps+3);//Yaw轴角速度数据

    Sent_cnt++;

    if(Sent_cnt>=5)
    {
        Vision_Sent(Vision_SentData);
        Sent_cnt=0;
    }


}

uint32_t vision_this_time,vision_last_time;/*视觉接受的时间*/
float YawTarget_now,PitchTarget_now;//实际视觉给定角度
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*记录视觉更新数据时的云台数据，给下次接收用*/
bool predict_flag = false;	/*预测开启的标志位*/
void Vision_task()
{

//	static uint8_t last_identify_flag=0;/*上一次的接收标志位*/
    static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/

//update_target_yaw=0;	/*数据更新时的目标数据	*/
    if(vision_update_flag == true)	/*视觉更新了数据*/
    {
        if(Vision_receive.identify_target == 1)		/*继续判断当前是否识别*/
        {
            active_cnt++; 	/*活跃计数*/

            if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
            {
                Cruise_mode = ATTACK;

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
                Cruise_mode = SCOUT;
                active_cnt = 0;
                lost_cnt = 0;
                /*侦察模式的切换*/
                Clear_Queue(&target_speed);
                Clear_Queue(&target_accel);
                Clear_Queue(&target_distance_tendency);
                /*清除队列信息，防止下一次数据受到影响*/
//				predict_delay_cnt = 0;	 	/*重置预测计数*/
                predict_angle_raw=0;//清0预测角
                predict_flag = false;			/*关闭预测*/
            }
        }



//		Vision_receive.yaw_angle=myDeathZoom(0,0.2,Vision_receive.yaw_angle);
//		Vision_receive.pitch_angle=myDeathZoom(0,0.1,Vision_receive.pitch_angle);
        vision_yaw_raw = (Vision_receive.yaw_angle-3.5f) * CONVER_SCALE;	/*变换为机械角度的尺度,理论上应24.756..24.2*/
        vision_pitch_raw=(Vision_receive.pitch_angle+6.99f)* 22.9f;//补偿角可以在这里直接加，注意单位6.69

        vision_dis_meter=Vision_receive.distance/1000.f;

        vision_yaw_raw=myDeathZoom(0,6,vision_yaw_raw);
        vision_pitch_raw=myDeathZoom(0,3,vision_pitch_raw);//死区处理10,5

        VisualProcess.YawGet_KF = KalmanFilter(&kalman_visionYaw,vision_yaw_raw); 	/*对视觉角度数据做卡尔曼滤波*/
        VisualProcess.PitchGet_KF = KalmanFilter(&kalman_visionPitch,vision_pitch_raw);
        if(vision_dis_meter>0.0001f)
            VisualProcess.DistanceGet_KF =KalmanFilter(&kalman_visionDistance,vision_dis_meter);
//		VisualProcess.DistanceGet_KF=vision_dis_meter;
        YawTarget_now=update_cloud_yaw+VisualProcess.YawGet_KF;
        PitchTarget_now=update_cloud_pitch+VisualProcess.PitchGet_KF;

        update_cloud_yaw=Gimbal_yaw_PPM.PID_PPM.measure;/*视觉数据更新时的云台角度*/
        update_cloud_pitch=Gimbal_pitch_PPM.PID_PPM.measure;

        /*视觉数据推演.......................................*/
//		target_speed_raw = Get_Target_Speed(10,YawTarget_now);

        target_speed_raw = Get_Diff(20,&target_speed,YawTarget_now);//新版

        target_speed_raw = KalmanFilter(&kalman_speedYaw,target_speed_raw);
        target_speed_raw=myDeathZoom(0,0.3,target_speed_raw);
        /*利用视觉更新的样本点来计算速度-由于更新周期不稳定，故采用离散方式处理*/
//		target_accel_raw = Get_Target_Accel(5,target_speed_raw);	 /*获取加速度*/

        target_accel_raw = Get_Diff(10,&target_accel,target_speed_raw);	 /*新版获取加速度*/
        target_accel_raw = myDeathZoom(0,1,target_accel_raw);		/*死区处理 - 滤除0点附近的噪声*/
        target_accel_raw = KalmanFilter(&kalman_accel,target_accel_raw);

        target_dis_tend =  Get_Diff(10,&target_distance_tendency,	VisualProcess.DistanceGet_KF);
        target_dis_tend =  myDeathZoom(0,0.1,target_dis_tend);

        /*......................................................*/
//		last_identify_flag = Vision_receive.identify_target;//记录识别标志位
        vision_update_flag = false;		/*清除标志位*/
    }

    /*直接跟随给定.................................*/
	VisualProcess.YawTarget_KF=KalmanFilter(&kalman_targetYaw,YawTarget_now);
	VisualProcess.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/

    /*以下是预测方面的代码...................................................................................*/
//    if(predict_flag==true)
//    {
//        float dir_factor;
//        if((target_speed_raw*target_accel_raw)>=0)
//        {
//            dir_factor= 1.f;//4

//        }
//        else
//        {
//            dir_factor= 1.8f;//4.5
//        }

//        feedforwaurd_angle = k_ff * target_accel_raw; 	/*计算前馈角*/

//        //	feedforward_angle = constrain(feedforward_angle,-28.f,28.f);	 /*前馈角16*/

//        predict_angle_raw = (0.8f*target_speed_raw*VisualProcess.DistanceGet_KF+2.2f*dir_factor*feedforward_angle) ;	/*计算预测角度*/

//        //	predict_angle_raw = constrain(predict_angle_raw,-60,60);	/*限幅28*/
//        //	predict_angle = RampFloat((abs(predict.0_angle_raw - predict_angle)/200),predict_angle_raw,predict_angle); 	/*斜坡处理*/
//    }
//    VisualProcess.YawTarget_KF=YawTarget_now+predict_angle_raw*use_predic;
//    VisualProcess.YawTarget_KF=KalmanFilter(&kalman_targetYaw,VisualProcess.YawTarget_KF);
//    VisualProcess.PitchTarget_KF=PitchTarget_now+600*target_dis_tend;
//    VisualProcess.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,VisualProcess.PitchTarget_KF);

    /*...................................................................................................*/

//	Sent_to_Vision();//发送给视觉

}

void Clear_Queue(QueueObj* queue)
{
    for(uint16_t i=0; i<queue->queueLength; i++)
    {
        queue->queue[i]=0;
    }
    queue->nowLength = 0;
    queue->queueTotal = 0;
    queue->aver_num=0;
    queue->Diff=0;
    queue->full_flag=0;
}
/**
* @brief 获取目标的速度
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Target_Speed(uint8_t queue_len,float angle)
{
    float sum=0;
    float tmp=0;
	
    if(queue_len>target_speed.queueLength)
        queue_len=target_speed.queueLength;
    //防止溢出
    if(target_speed.nowLength<queue_len)
    {
        //队列未满，只进不出
        target_speed.queue[target_speed.nowLength] = angle;
        target_speed.nowLength++;
    }
    else
    {
//		target_speed.nowLength=0;
        //队列已满，FIFO。
        for(uint16_t i=0; i<queue_len-1; i++)
        {
            target_speed.queue[i] = target_speed.queue[i+1];
            //更新队列
        }
        target_speed.queue[queue_len-1] = angle;
    }

    //更新完队列
    for(uint16_t j=0; j<queue_len; j++)
    {
        sum+=target_speed.queue[j];
    }
    tmp = sum/(queue_len/1.f);
    tmp = (angle - tmp);
    return tmp;
}



/**
* @brief 获取目标的加速度
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Target_Accel(uint8_t queue_len,float speed)
{
    float sum=0;
    float tmp=0;
	
    if(queue_len>target_accel.queueLength)
        queue_len=target_accel.queueLength;
    //防止溢出

    if(target_accel.nowLength<queue_len)
    {
        //队列未满，只进不出
        target_accel.queue[target_accel.nowLength] = speed;
        target_accel.nowLength++;
    }
    else
    {
        //队列已满，FIFO。
        for(uint16_t i=0; i<queue_len-1; i++)
        {
            target_accel.queue[i] = target_accel.queue[i+1];
            //更新队列

        }
        target_accel.queue[queue_len-1] = speed;
    }

    //更新完队列


    for(uint16_t j=0; j<queue_len; j++)
    {
        sum+=target_accel.queue[j];
    }
    tmp = sum/(queue_len/1.f);

    tmp = (speed - tmp);

    return tmp;
}


/**
* @brief 获取目标的距离变化趋势
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Distance_Tendency(uint8_t queue_len,float dis)
{
    float sum=0;
    float tmp=0;

    if(queue_len>target_distance_tendency.queueLength)
        queue_len=target_distance_tendency.queueLength;
    //防止溢出


    if(target_distance_tendency.nowLength<queue_len)
    {
        //队列未满，只进不出
        target_distance_tendency.queue[target_accel.nowLength] = dis;
        target_distance_tendency.nowLength++;
    }
    else
    {
        //队列已满，FIFO。
        for(uint16_t i=0; i<queue_len-1; i++)
        {
            target_distance_tendency.queue[i] = target_distance_tendency.queue[i+1];
            //更新队列
        }
        target_distance_tendency.queue[queue_len-1] = dis;
    }

    //更新完队列
    for(uint16_t j=0; j<queue_len; j++)
    {
        sum+=target_distance_tendency.queue[j];
    }
    tmp = sum/(queue_len/1.f);

    tmp = (dis - tmp);

    return tmp;
}


/**
* @brief 获取目标的差分
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    if(queue_len>=Data->queueLength)
        queue_len=Data->queueLength;
    //防止溢出
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

    Data->queue[Data->nowLength]=add_data;

    Data->aver_num=Data->queueTotal/queue_len;
    Data->nowLength++;

    if(Data->full_flag==0)//初始队列未满
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data-Data->aver_num;
    return Data->Diff;
}
