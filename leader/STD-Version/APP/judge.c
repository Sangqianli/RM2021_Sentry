#include "judge.h"


uint8_t USART_STU_BUF[200];     //学生接口接收缓冲,最大200个字节.
uint16_t USART_STU_STA=0;       //学生接口接收状态标记
uint8_t USART_FRAME[46];
uint8_t USART_LEN = 0;
uint8_t recLength = 0;

/*一堆信息*/
typedef __packed struct//比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_status_t;

typedef __packed struct//比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
{
    uint8_t winner;
} ext_game_result_t;

//机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct
{

    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

//人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。
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
} ext_ICRA_buff_debuff_zone_status_t;

//场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

//补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

//飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
    uint8_t shooter_heat0_speed_limit;
    uint8_t shooter_heat1_speed_limit;
    uint8_t max_chassis_power;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

//实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。


//机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

//机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

//空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint16_t energy_point;
    uint8_t attack_time;
} ext_aerial_robot_energy_t;

//伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

//子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人。
typedef __packed struct
{
    uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

//机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

//飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;






//交互数据接收信息：0x0301。
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

//客户端删除图形 机器人间通信：0x0301。
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;
//图形数据
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

//客户端绘制一个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
//客户端绘制二个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//客户端绘制五个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
//客户端绘制七个图形 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
//客户端绘制字符 机器人间通信：0x0301。
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;





//CRC 校验代码示例
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int
                                 dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
                                   0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
//return __FALSE;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
            pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}



typedef __packed struct { //0001 Frame1 比赛机器人状态包
    u8 SOF;
    u16 DataLength;
    u8 Seq;
    u8 CRC8;
    u16 CmdID;
    ext_game_robot_status_t extGameRobotState;
    u16 FrameTail;
} Frame1;

typedef __packed struct { //0002 Frame2 伤害数据包
    u8 SOF;
    u16 DataLength;
    u8 Seq;
    u8 CRC8;
    u16 CmdID;
    ext_robot_hurt_t extRobotHurt;
    u16 FrameTail;
} Frame2;

typedef __packed struct { //0003 Frame3 实时射击数据包
    u8 SOF;
    u16 DataLength;
    u8 Seq;
    u8 CRC8;
    u16 CmdID;
    ext_shoot_data_t extShootData;
    u16 FrameTail;
} Frame3;



Frame1 Robot_State;

Frame4 Power_Heat;

xFrameHeader              FrameHeader;		//发送帧头信息
int Judge_ID;
int J_CMD;
bool Judge_Data_Receive(u8 *ReadFromUsart)
{
    bool retval_tf = 0;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误

    uint16_t judge_length;//统计一帧数据长度

    int CmdID = 0;//数据命令码解析

    retval_tf=retval_tf;//消除警告

    if(ReadFromUsart == NULL)
    {
        return -1;
    }
    memcpy(&FrameHeader,ReadFromUsart,5);   //储存帧头数据



    if(ReadFromUsart[0] == 0xa5)                   //判断帧头是否为0xa5
    {
        J_CMD =(ReadFromUsart[6] << 8 | ReadFromUsart[5]);

        if(Verify_CRC8_Check_Sum( ReadFromUsart, 5 ) == 1)  //帧头CRC校验
        {
            judge_length =(ReadFromUsart[2]<<8|ReadFromUsart[1]) + 5 + 2 + 2;	// ReadFromUsart[1]统计一帧数据长度,用于CR16校验???不是两个字节存储吗(ReadFromUsart[2]<<8|ReadFromUsart[1])
            if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == 1)//帧尾CRC16校验
            {
                retval_tf = 1;//数据可用
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                switch(CmdID)
                {
                case 0x0201:
                    Judge_ID=0x0201;
                    memcpy(&Robot_State,ReadFromUsart,27);
                    break;

                case 0x0206:
                    Judge_ID=0x0206;
                    break;

                case 0x0207:
                    Judge_ID=0x0207;
                    break;

                case 0x0202:
                    Judge_ID=0x0202;
                    memcpy(&Power_Heat,ReadFromUsart,25);
                    break;
                }
            }
        }
        if(*(ReadFromUsart + sizeof(xFrameHeader) + 2 + FrameHeader.DataLength + 2) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Judge_Data_Receive(ReadFromUsart + sizeof(xFrameHeader) + 2 + FrameHeader.DataLength + 2);
        }

    }
    return true;
}



//串口5全局中断服务函数
void UART5_IRQHandler(void)
{
    uint8_t rc_tmp;



    //串口空闲中断
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(DMA1_Stream0,DISABLE); //关闭DMA传输

        rc_tmp=UART5->SR;
        rc_tmp=UART5->DR;//软件序列清除IDLE标志位
        rc_tmp=rc_tmp;

        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF2);

        Judge_Data_Receive(jbus_rx_buffer);
        memset(jbus_rx_buffer, 0, 200);


        DMA_Cmd(DMA1_Stream0,ENABLE);    //接着传输


        USART_ClearFlag(UART5,USART_FLAG_IDLE);

    }
}




