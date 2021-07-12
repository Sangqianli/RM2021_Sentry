#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "system_task.h"
/* Exported macro ------------------------------------------------------------*/
#define HP_Danger   300
/* Exported types ------------------------------------------------------------*/
typedef enum {
    FIRE_ALL,
    FIRE_RUN,
} chassis_fire_t; //底盘配合射击的模式

typedef enum {
    CHASSIS_SAFE,
    CHASSIS_HURT,
    CHASSIS_DANGER,
} chassis_safe_mode_t; //自身是否安全的状态标志位

typedef enum {
    CHASSIS_NORMAL,	// 侦察模式
    CHASSIS_ATTACK, //打击模式
    CHASSIS_COVER,  //掩护模式
    CHASSIS_ESCAPE, //逃跑模式
} chassis_mode_t;

typedef enum {
    WAY_NORMAL,	// 正常模式
    WAY_TOUCH,   //开关模式
    WAY_ENCODER,   //编码器模式
} chassis_way_t;

typedef struct Chassis {
    pid_ctrl_t	 PPM;
    pid_ctrl_t	 PVM;
    float        Speed_taget;
    int32_t		 Mileage_atrip;//轨道长度
    uint8_t		 init_flag;
    int8_t       Derection_flag;//1为左，-1为右
    int16_t      Trip_times;   //底盘往返次数
    uint8_t      rotate_ratio;//遥控灵敏度
    bool         swerve_judge;//反弹是否完成的判断
    bool         swerve_flag;//反弹流程标志位
    bool         overbuff_flag;//超缓冲能量标志位
    bool         getchange_flag;//获取轨道目标位置标志位
    bool         static_want;//底盘目标静止
    int32_t      Spot_taget;//轨道目标位置
    chassis_mode_t  Mode ; //各种模式
    chassis_safe_mode_t Safe;
    chassis_fire_t Fire;
    chassis_way_t Way;
} Chassis_t;
/* Exported functions --------------------------------------------------------*/
extern Chassis_t Chassis_process;
void   Chassis_Init(void);
void   StartChassisTask(void const * argument);
#endif
