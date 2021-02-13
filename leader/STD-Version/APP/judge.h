#ifndef __JUDGE_H
#define __JUDGE_H
#include "system.h"

typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;

} xFrameHeader;

typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
    uint16_t mobile_shooter_heat2;
} ext_power_heat_data_t;

typedef __packed struct { //0004 Frame4 实时功率热量包
    u8 SOF;
    u16 DataLength;
    u8 Seq;
    u8 CRC8;
    u16 CmdID;
    ext_power_heat_data_t extPowerHeatData;
    u16 FrameTail;
} Frame4;

extern Frame4 Power_Heat;





unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
bool Judge_Data_Receive(u8 *ReadFromUsart);



















#endif
