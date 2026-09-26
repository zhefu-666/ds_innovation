#ifndef MCU_SENSOR_PACKET_H
#define MCU_SENSOR_PACKET_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <float.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Motion protocol requires IEEE754 float32"
#endif

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Packed UART packets require a little-endian target"
#endif
/* 下位机反馈：A6 + gripper_done + CRC16，共4字节，小端、1字节对齐。
 * CRC16/Modbus覆盖0..1，初值FFFF、多项式A001。ToF字段暂禁用。 */
#pragma pack(push, 1)
typedef struct {
    uint8_t start_of_frame;
    uint8_t gripper_done;
    // uint16_t tof_fl_mm;    
    // uint16_t tof_fr_mm;    
    // uint16_t tof_rl_mm;    
    // uint16_t tof_rr_mm;    
    uint16_t crc16;       
} RescueSensorPacket;

/* 上位机控制包：10字节，IEEE754 float32，m/s、rad/s，帧头56，无CRC。 */
typedef struct {
    uint8_t start_of_frame;
    float vx_mps;       
    float wz_rps;
    uint8_t gripper_closed; // 0张开，1框住     
} RescueMotionPacket;
#pragma pack(pop)
typedef char rescue_sensor_size_check[(sizeof(RescueSensorPacket)==4 &&
    offsetof(RescueSensorPacket,crc16)==2)?1:-1];
typedef char rescue_motion_size_check[(sizeof(RescueMotionPacket)==10 && sizeof(float)==4 &&
    offsetof(RescueMotionPacket,vx_mps)==1 && offsetof(RescueMotionPacket,wz_rps)==5 &&
    offsetof(RescueMotionPacket,gripper_closed)==9)?1:-1];

/* 下位机发送与测试共用模板；IMU不通过此包传输。 */
static inline void rescue_pack_sensors(
    uint8_t out[4], uint8_t gripper_done)
{
    RescueSensorPacket packet = {0};
    packet.start_of_frame = 0xA6;
    packet.gripper_done = gripper_done;
    memcpy(out, &packet, sizeof(packet));
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < 2; ++i) {
        crc ^= out[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1) ? 0xA001 : 0));
    }
    packet.crc16 = crc;
    memcpy(out, &packet, sizeof(packet));
}
#endif
