/**
 * @file controller.cpp
 * @author rediduck（39328428@qq.com）
 * @brief 遥控器接收数据解码文件
 * @version 0.1
 * @date 2026-05-01
 *
 * @copyright Copyright (c) 2026
 *
 */

/**************************************************************************************************
 * 说明：
 *
 * 这个接收代码没有采用库里已经有的统一串口接收和环形缓冲区接收机制
 * 因为rediduck有一点自己关于串口的想法，不想在串口接收中断中放任何解码函数
 * 这套接收的逻辑是：遥控器接收串口接收到DMA -> DMA触发接收中断
 *                 -> 中断回调函数把数据放入环形缓冲区
 *                 -> 创建一个专门的任务轮询环形缓冲区解码数据
 *
 * 经过计算，复制14个字节的开销在0.1微妙左右，
 * 远小于115200波特率串口下每个字节约87微秒的接收时间，
 * 因此DMA接收中断中复制数据的开销是完全可以接受的，
 * 不太会遇到溢出的问题，如果你遇到了另说
 *
 *
 * 拨码开关8位表示：矛杆状态（2）| 抽屉升降状态（1）| 气泵前后位置（1）| 十字摇杆模式（1）|
 * 抽屉左右状态（2）
 *
 *
 ***************************************************************************************************/
#include "controller.hpp"
#include "stm32f4xx_hal_uart.h"
#include "watchdog.hpp"
#include <cstdint>
#include <string.h>
#include <cstdlib>

namespace Controller
{

using ChassisConfig = Chassis::ChassisConfig;

#define RAWDATA_SIZE  14 // 每一帧大小
#define RINGBUFF_SIZE 64
#define FRAME_HEADER1 0xAA // 帧头1
#define FRAME_HEADER2 0xBB // 帧头2
#define BUTTON_NUM    12   // 按钮个数

static uint8_t readIndex  = 0;                 // 读指针
static uint8_t writeIndex = 0;                 // 写指针
uint8_t        buffer[RAWDATA_SIZE];           // DMA接收数组
static uint8_t RX_RING_BUFFER[RINGBUFF_SIZE];  // 数据环形缓冲区
uint32_t       decode_count            = 0;    // 解码总次数
uint32_t       decode_error_count      = 0;    // 解码错误次数
uint32_t       decode_success_count    = 0;    // 成功解码次数
bool           is_controller_connected = true; // 遥控器连接状态

static service::Watchdog controller_watchdog; // 遥控器看门狗

float LX_T; // 左摇杆x值数据转换后速度数据
float LY_T; // 左摇杆y值数据转换后速度数据
float RX_T; // 右摇杆x值数据转换后速度数据

int16_t LX; // 左摇杆x值数据原始数据
int16_t LY; // 左摇杆y值数据原始数据
int16_t RX; // 右摇杆x值数据原始数据
int16_t RY; // 右摇杆y值数据原始数据

bool           button[BUTTON_NUM]; // 矩阵键盘按钮
uint8_t        DIP_switch;         // 拨码开关状态
uint8_t        crc = 0;            // CRC校验值
static cmd_vel joystick_vel;
static mode    control_mode = MANUAL;

osThreadId_t         controllerHandle;
const osThreadAttr_t controller_attributes = {
    .name       = "controller",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityHigh,
};

// CRC8校验
static uint8_t CRC8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void Msg_AddReadIndex(uint8_t length)
{
    readIndex = (readIndex + length) % RINGBUFF_SIZE;
}

static uint8_t Msg_Read(uint8_t offset)
{
    uint8_t index = (readIndex + offset) % RINGBUFF_SIZE;
    return RX_RING_BUFFER[index];
}

static uint8_t Msg_GetLength()
{
    return (writeIndex + RINGBUFF_SIZE - readIndex) % RINGBUFF_SIZE;
}

static uint8_t Msg_GetRemain()
{
    return RINGBUFF_SIZE - Msg_GetLength();
}

static uint8_t Msg_Write(uint8_t* data, uint8_t length)
{
    if (Msg_GetRemain() < length)
        return 0;

    if (writeIndex + length < RINGBUFF_SIZE)
    {
        memcpy(RX_RING_BUFFER + writeIndex, data, length);
        writeIndex += length;
    }
    else
    {
        uint8_t firstPart = RINGBUFF_SIZE - writeIndex;
        memcpy(RX_RING_BUFFER + writeIndex, data, firstPart);
        memcpy(RX_RING_BUFFER, data + firstPart, length - firstPart);
        writeIndex = length - firstPart;
    }
    writeIndex %= RINGBUFF_SIZE;
    return length;
}

static bool Msg_SyncToHeader()
{
    while (Msg_GetLength() >= 14)
    {
        if (Msg_Read(0) == FRAME_HEADER1 && Msg_Read(1) == FRAME_HEADER2)
        {
            return true;
        }
        Msg_AddReadIndex(1);
    }
    return false;
}

static void Button_Init(void)
{
    // 清零按键状态
    for (size_t i = 0; i < BUTTON_NUM; i++)
    {
        button[i] = 0;
    }
    // 清零拨码状态
    DIP_switch = 0;
}

static float Joystick2Velocity(int16_t joystick_value)
{
    if (std::abs(joystick_value) < ChassisConfig::JOYSTICK_RAW_MIDDLE)
        return joystick_value * ChassisConfig::MIDDLE_VEL / ChassisConfig::JOYSTICK_RAW_MIDDLE;
    else
        return (joystick_value - (joystick_value > 0 ? ChassisConfig::JOYSTICK_RAW_MIDDLE
                                                     : -ChassisConfig::JOYSTICK_RAW_MIDDLE)) *
                       (ChassisConfig::MAX_VEL - ChassisConfig::MIDDLE_VEL) /
                       (ChassisConfig::JOYSTICK_RAW_MAX - ChassisConfig::JOYSTICK_RAW_MIDDLE) +
               (joystick_value > 0 ? ChassisConfig::MIDDLE_VEL : -ChassisConfig::MIDDLE_VEL);
}

static float Joystick2Wz(int16_t joystick_value)
{
    if (std::abs(joystick_value) < ChassisConfig::JOYSTICK_RAW_MIDDLE)
        return joystick_value * ChassisConfig::MIDDLE_WZ / ChassisConfig::JOYSTICK_RAW_MIDDLE;
    else
        return (joystick_value - (joystick_value > 0 ? ChassisConfig::JOYSTICK_RAW_MIDDLE
                                                     : -ChassisConfig::JOYSTICK_RAW_MIDDLE)) *
                       (ChassisConfig::MAX_WZ - ChassisConfig::MIDDLE_WZ) /
                       (ChassisConfig::JOYSTICK_RAW_MAX - ChassisConfig::JOYSTICK_RAW_MIDDLE) +
               (joystick_value > 0 ? ChassisConfig::MIDDLE_WZ : -ChassisConfig::MIDDLE_WZ);
}

extern "C" void controller_task(void* argument)
{
    while (1)
    {
        // 每次解析函数调用时都会把所有可解的码全解码了
        while (Msg_SyncToHeader())
        {
            decode_count++;           // 解码计数器加1
            uint8_t receive_data[11]; // 从环形缓冲区提取出来的一帧数据(去掉了帧头和CRC)
            for (size_t i = 0; i < RAWDATA_SIZE - 3; i++)
            {
                receive_data[i] = Msg_Read(i + 2); // 跳过帧头
            }
            uint8_t calculate_crc = CRC8(receive_data, RAWDATA_SIZE - 3);
            uint8_t received_crc  = Msg_Read(13);

            if (calculate_crc == received_crc)
            {
                // 校验通过，开始解析
                LX = int16_t((Msg_Read(2) << 8) | Msg_Read(3));
                LY = int16_t((Msg_Read(4) << 8) | Msg_Read(5));
                RX = int16_t((Msg_Read(6) << 8) | Msg_Read(7));
                RY = int16_t((Msg_Read(8) << 8) | Msg_Read(9));

                joystick_vel.vel_x  = Joystick2Velocity(LY);
                joystick_vel.vel_y  = -1.0f * Joystick2Velocity(LX);
                joystick_vel.vel_wz = -1.0f * Joystick2Wz(RX);

                // 解析拨码开关数据
                DIP_switch = Msg_Read(10);

                // 解析按钮数据
                for (size_t i = 0; i < 8; i++)
                {
                    button[i] = (Msg_Read(12) >> i) & 0x01;
                }
                for (size_t i = 0; i < 4; i++)
                {
                    button[8 + i] = (Msg_Read(11) >> i) & 0x01;
                }
                decode_success_count++;
                Msg_AddReadIndex(RAWDATA_SIZE);
            }
            else
            {
                // crc校验不通过
                Msg_AddReadIndex(2); // 说明这个帧头不对，跳过这个帧头
                decode_error_count++;
            }
        }
        osDelay(1);
    }
}

void Controller_Receive_Init(void)
{
    control_mode = MANUAL;
    Button_Init();
    osThreadNew(controller_task, NULL, &controller_attributes);
    HAL_UART_Receive_DMA(&huart1, buffer, RAWDATA_SIZE);
}

void ControllerReceive_OnRxCplt()
{
    Msg_Write(buffer, RAWDATA_SIZE);
}

void softTIM_controller()
{
    switch (control_mode)
    {
    case MANUAL:
        Chassis::chassis_ctrl_->setVelocityInBody({ .vx = joystick_vel.vel_x,
                                                    .vy = joystick_vel.vel_y,
                                                    .wz = joystick_vel.vel_wz },
                                                  false);
        break;
    case AUTO_AIM:

        break;
    default:
        break;
    }
}

} // namespace Controller
