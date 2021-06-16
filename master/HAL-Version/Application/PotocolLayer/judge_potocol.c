/**
 * @file        judge_potocol.c
 * @author      Sentry@2021
 * @Version     V1.0
 * @date        19-February-2021
 * @brief       .
 */

/* Includes ------------------------------------------------------------------*/
#include  "judge_potocol.h"
#include  "judge_sensor.h"
/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER		(0xA5)
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* 帧字节偏移 */
typedef enum {
    FRAME_HEADER	= 0,
    CMD_ID			= 5,
    DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* 帧头字节偏移 */
typedef enum {
    J_SOF			= 0,
    DATA_LENGTH	= 1,
    SEQ			= 3,
    J_CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
    ID_GAME_STATUS 					= 0x0001,	// 比赛状态
    ID_GAME_RESULT 					= 0x0002,	// 比赛结果
    ID_GAME_ROBOT_HP 				= 0x0003,	// 机器人血量数据
    ID_DART_STATUS					= 0x0004,	// 飞镖发射状态
    ID_ICRA_BUFF_DEBUFF_ZONE_STATUS = 0x0005,	// 人工智能挑战赛加成与惩罚区状态

    ID_EVENT_DATA 					= 0x0101,	// 场地事件数据
    ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// 补给站动作标识
    //ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// 请求补给站补弹子弹
    ID_REFEREE_WARNING 				= 0x0104,	// 裁判警告信息
    ID_DART_REMAINING_TIME			= 0x0105,	// 飞镖发射口倒计时

    ID_GAME_ROBOT_STATUS 			= 0x0201,	// 比赛机器人状态
    ID_POWER_HEAT_DATA 				= 0x0202,	// 实时功率热量数据
    ID_GAME_ROBOT_POS				= 0x0203,	// 机器人位置
    ID_BUFF							= 0x0204,	// 机器人增益
    ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// 空中机器人能量状态
    ID_ROBOT_HURT					= 0x0206,	// 机器人伤害状态
    ID_SHOOT_DATA					= 0x0207,	// 实时射击信息
    ID_BULLET_REMAINING				= 0x0208,	// 子弹剩余发射数
    ID_RFID_STATUS					= 0x0209,	// 机器人RFID状态

    ID_COMMUNICATION				= 0x0301,	// 机器人间交互数据(发送方触发发送)
} Judge_Cmd_ID_t;

typedef enum {
    /* Std */
    LEN_FRAME_HEAD 	= 5,	// 帧头长度
    LEN_CMD_ID 		= 2,	// 命令码长度
    LEN_FRAME_TAIL 	= 2,	// 帧尾CRC16
    /* Ext */
    // 0x000x
    LEN_GAME_STATUS 				= 11,
    LEN_GAME_RESULT 				= 1,
    LEN_GAME_ROBOT_HP 				= 32,
    LEN_DART_STATUS					= 3,
    LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS= 11,

    // 0x010x
    LEN_EVENT_DATA					= 4,
    LEN_SUPPLY_PROJECTILE_ACTION	= 4,
    LEN_REFEREE_WARNING				= 2,
    LEN_DART_REMAINING_TIME			= 1,

    // 0x020x
    LEN_RADAR_DATA			        = 24,
    LEN_GAME_ROBOT_STATUS			= 27,
    LEN_POWER_HEAT_DATA 			= 16,
    LEN_GAME_ROBOT_POS				= 16,
    LEN_BUFF		 				= 1,
    LEN_AERIAL_ROBOT_ENERGY 		= 1,
    LEN_ROBOT_HURT					= 1,
    LEN_SHOOT_DATA					= 7,
    LEN_BULLET_REMAINING	 		= 6,
    LEN_RFID_STATUS					= 4,
} Judge_Data_Length_t;
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void judge_init(judge_sensor_t *judge)
{
    // 初始化为离线状态
    judge->info->offline_cnt = judge->info->offline_max_cnt + 1;
    judge->work_state = DEV_OFFLINE;

    if(judge->id == DEV_ID_VISION)
        judge->errno = NONE_ERR;
    else
        judge->errno = DEV_ID_ERR;
}
void judge_update(uint8_t *rxBuf)
{

    uint8_t  res = false;
    uint16_t frame_length;
    uint16_t cmd_id;
    judge_info_t *judge_info = judge_sensor.info;
    memcpy(&judge_info->FrameHeader, rxBuf, LEN_FRAME_HEAD);

    /* 帧首字节是否为0xA5 */
    if(rxBuf[J_SOF] == 0xa5)
    {
        /* 帧头CRC8校验 */
        if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true)
        {
            /* 统计一帧的总数据长度，用于CRC16校验 */
            frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->FrameHeader.data_length + LEN_FRAME_TAIL;
            judge_info->frame_length = frame_length;

            if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
            {
                res = true;
                judge_info->offline_cnt = 0;
                cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
                judge_info->cmd_id = cmd_id;

                switch(cmd_id)
                {
                case ID_GAME_STATUS: {
                    memcpy(&judge_info->GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
                }
                break;

                case ID_GAME_RESULT: {
                    memcpy(&judge_info->GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
                }
                break;

                case ID_GAME_ROBOT_HP: {
                    memcpy(&judge_info->GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
                }
                break;

                case ID_DART_STATUS: {
                    memcpy(&judge_info->DartStatus, (rxBuf+DATA_SEG), LEN_DART_STATUS);
                    judge_info->dart_data_update = true;	// 飞镖数据更新
                }
                break;

                case ID_EVENT_DATA: {
                    memcpy(&judge_info->EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
                }
                break;

                case ID_SUPPLY_PROJECTILE_ACTION: {
                    memcpy(&judge_info->SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
                    judge_info->supply_data_update = true;	// 补给站数据更新
                }
                break;

                case ID_REFEREE_WARNING: {
                    memcpy(&judge_info->RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
                }
                break;

                case ID_DART_REMAINING_TIME: {
                    memcpy(&judge_info->DartRemainingTime, (rxBuf+DATA_SEG), LEN_DART_REMAINING_TIME);
                }
                break;

                case ID_GAME_ROBOT_STATUS: {
                    memcpy(&judge_info->GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
                }
                break;

                case ID_POWER_HEAT_DATA: {
                    memcpy(&judge_info->PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
                    judge_info->power_heat_update = true;
                }
                break;

                case ID_GAME_ROBOT_POS: {
                    memcpy(&judge_info->GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
                }
                break;

                case ID_BUFF: {
                    memcpy(&judge_info->Buff, (rxBuf+DATA_SEG), LEN_BUFF);
                }
                break;

                case ID_AERIAL_ROBOT_ENERGY: {
                    memcpy(&judge_info->AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
                }
                break;

                case ID_ROBOT_HURT: {
                    memcpy(&judge_info->RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
                    judge_info->hurt_data_update = true;	// 伤害数据更新
                }
                break;

                case ID_SHOOT_DATA: {
                    memcpy(&judge_info->ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
//							JUDGE_ShootNumCount();	// 计算发弹量
                    judge_info->shoot_update = true;
                }
                break;

                case ID_BULLET_REMAINING: {
                    memcpy(&judge_info->BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
                }
                break;

                case ID_RFID_STATUS: {
                    memcpy(&judge_info->RfidStatus, (rxBuf+DATA_SEG), LEN_RFID_STATUS);
                }
                break;

                case ID_COMMUNICATION: {
                    memcpy(&judge_info->RadarData, (rxBuf+DATA_SEG), LEN_RADAR_DATA);  //雷达站数据
                }
                break;
				
				case 0x0303:{
					memcpy(&judge_info->command, (rxBuf+DATA_SEG), 15);  //云台数据
				}
                }
            }
        }
        /* 帧尾CRC16下一字节是否为0xA5 */
        if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
        {
            /* 如果一个数据包出现了多帧数据就再次读取 */
            judge_update( &rxBuf[frame_length] );
        }
    }

    judge_info->data_valid = res;
    if(judge_info->data_valid != true) {
        judge_info->err_cnt++;
        judge_info->data_valid = false;
    }
    else {
        judge_info->data_valid = true;
    }

//	return res;
}
/**
 *	@brief	在串口2中解析遥控数据协议
 */
void USART5_rxDataHandler(uint8_t *rxBuf)
{
    // 更新裁判系统数据
    judge_sensor.update(rxBuf);
    judge_sensor.check(&judge_sensor);
}
