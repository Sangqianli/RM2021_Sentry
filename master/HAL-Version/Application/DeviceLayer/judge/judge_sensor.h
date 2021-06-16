#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H
/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "drv_uart.h"
/* Exported macro ------------------------------------------------------------*/
// 机器人间交互数据段长度
#define INTERACT_DATA_LEN	10

/* Exported types ------------------------------------------------------------*/
/* 自定义帧头 Byte: 5 */
typedef __packed struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} std_frame_header_t;


/* ID: 0x0001	Byte: 	11	比赛状态数据 */
typedef __packed struct
{
    uint8_t game_type : 4;			// 比赛类型
    uint8_t game_progress : 4;		// 比赛阶段
    uint16_t stage_remain_time;		// 当前阶段剩余时间(单位:s)
    uint64_t SyncTimeStamp;       //机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_status_t;


/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;


/* ID: 0x0003	Byte:	36	机器人血量数据数据 */
typedef __packed struct
{
    uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
    uint16_t red_2_robot_HP;	// 红2工程机器人血量
    uint16_t red_3_robot_HP;	// 红3步兵机器人血量
    uint16_t red_4_robot_HP;	// 红4步兵机器人血量
    uint16_t red_5_robot_HP;	// 红5步兵机器人血量
    uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
    uint16_t red_outpost_HP;	// 红方前哨站血量
    uint16_t red_base_HP;		// 红方基地血量
    uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
    uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
    uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
    uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
    uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
    uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
    uint16_t blue_outpost_HP;	// 蓝方前哨站血量
    uint16_t blue_base_HP;		// 蓝方基地血量
} ext_game_robot_HP_t;


/* ID: 0x0004 	Byte:	3	飞镖发射状态 */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005 	Byte:	3	人工智能挑战赛加成与惩罚区状态 */
typedef __packed struct
{
    uint8_t F1_zone_status:1;
    uint8_t F1_zone_buff_debuff_status:3;
    uint8_t F2_zone_status:1;
    uint8_t F2_zone_buff_debuff_status:3;
    uint8_t F3_zone_status:1;
    uint8_t F3_zone_buff_debuff_status:3;
    uint8_t F4_zone_status:1;
    uint8_t F4_zone_buff_debuff_status:3;
    uint8_t F5_zone_status:1;
    uint8_t F5_zone_buff_debuff_status:3;
    uint8_t F6_zone_status:1;
    uint8_t F6_zone_buff_debuff_status:3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct
{
    uint32_t others_1 : 10;
    uint32_t outpost  : 1;
    uint32_t others_2 : 21;
//	uint32_t event_type;
} ext_event_data_t;


/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


///* ID: 0X0103  Byte:  3    请求补给站补弹子弹数据 */
//typedef __packed struct
//{
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;
//} ext_supply_projectile_booking_t;


/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0105  Byte:  1	   飞镖发射口倒计时 */
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 18    机器人状态数据 */
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct
{
    uint16_t chassis_volt;   		// 底盘输出电压，单位：mV
    uint16_t chassis_current;		// 底盘输出电流，单位：mA
    float chassis_power;   			// 瞬时功率，单位：W
    uint16_t chassis_power_buffer;	// 底盘功率缓冲
    uint16_t  shooter_id1_17mm_cooling_heat;		// 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;	// 机动17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;	//  42mm 枪口热量
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct
{
    uint8_t attack_time;
} ext_aerial_robot_energy_t;


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct
{
    uint8_t armor_id : 4; 	// 装甲伤害时代表装甲ID
    uint8_t hurt_type : 4; 	// 0x0装甲伤害 0x1模块掉线 0x2超射速 0x3超热量 0x4超功率 0x5撞击
} ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct
{
    uint8_t bullet_type; 	// 子弹类型(1-17mm, 2-42mm)
    uint8_t shooter_id;
    uint8_t bullet_freq;  	// 子弹射频(Hz)
    float bullet_speed;		// 子弹射速(m/s)
} ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    子弹剩余发射数数据 */
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* ID: 0x0200  Byte:  12 	雷达站交互 */
typedef __packed struct
{
    uint16_t cmd_id;
    uint16_t send_id;
    uint16_t receive_id;
    float 	pitch_angle;	// pitch偏差角度/像素点	单位：角度/像素点
    float 	yaw_angle;		// yaw偏差角度/像素点	单位：角度/像素点
    float 	distance;			// 距离				单位：mm
    uint8_t    identify_target;
    uint8_t    BBB;
    uint8_t    CCC;
    uint8_t    DDD;
    uint8_t    EEE;
    uint8_t    FFF;
} ext_radar_data_t;

/* ID: 0x0303  Byte:  15 	 */
typedef __packed struct
{
    float target_x;
    float target_y;
    float target_z;
    uint8_t commd_keyboard;
    uint16_t target_ID;
} ext_robot_command_t;

typedef struct
{
    uint16_t frame_length;	// 整帧长度(调试时使用)
    uint16_t cmd_id;		// 命令码(调试时使用)
    uint16_t err_cnt;		// 错帧数(调试时使用)
    bool	 data_valid;	// 数据有效性
    uint16_t self_client_id;// 发送者机器人对应的客户端ID
    bool	power_heat_update;	// 功率热量数据更新
    bool	shoot_update;		// 射击数据更新
    bool	 hurt_data_update;	// 伤害数据更新
    bool	dart_data_update;	// 飞镖数据更新
    bool	supply_data_update;	// 补给站数据更新
    std_frame_header_t				FrameHeader;				// 帧头信息
    ext_game_status_t 				GameStatus;					// 0x0001
    ext_game_result_t 				GameResult;					// 0x0002
    ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
    ext_dart_status_t				DartStatus;					// 0x0004
    //ext_ICRA_buff_debuff_zone_status_t

    ext_event_data_t				EventData;					// 0x0101
    ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
    //ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
    ext_referee_warning_t			RefereeWarning;				// 0x0104
    ext_dart_remaining_time_t		DartRemainingTime;			// 0x0105

    ext_radar_data_t                RadarData;                  //0x200

    ext_game_robot_status_t			GameRobotStatus;			// 0x0201
    ext_power_heat_data_t			PowerHeatData;				// 0x0202
    ext_game_robot_pos_t			GameRobotPos;				// 0x0203
    ext_buff_t						Buff;						// 0x0204
    ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
    ext_robot_hurt_t				RobotHurt;					// 0x0206
    ext_shoot_data_t				ShootData;					// 0x0207
    ext_bullet_remaining_t			BulletRemaining;			// 0x0208
    ext_rfid_status_t				RfidStatus;					// 0x0209


    ext_robot_command_t             command;
    int16_t		offline_cnt;
    int16_t		offline_max_cnt;

} judge_info_t;

typedef struct judge_sensor_struct {
    judge_info_t	*info;
    drv_uart_t			*driver;
    void				(*init)(struct judge_sensor_struct *self);
    void				(*update)( uint8_t *rxBuf);
    void				(*check)(struct judge_sensor_struct *self);
    void				(*heart_beat)(struct judge_sensor_struct *self);
    dev_work_state_t	work_state;
    dev_errno_t			errno;
    dev_id_t			id;
} judge_sensor_t;

extern judge_sensor_t	judge_sensor;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif
