#include "controller.hpp"

#include "flags.hpp"
#include "stm32f4xx_hal_uart.h"
#include "vision_auto_align.hpp"
#include "watchdog.hpp"

#include <cstdlib>
#include <cstdint>
#include <string.h>

namespace Controller
{

using ChassisConfig = Chassis::ChassisConfig;
namespace ProjectControllerConfig = AppConfig::Controller;

static uint8_t readIndex  = 0;
static uint8_t writeIndex = 0;

uint8_t        buffer[ProjectControllerConfig::RawDataSize];
static uint8_t RX_RING_BUFFER[ProjectControllerConfig::RingBufferSize];
uint32_t       decode_count            = 0;
uint32_t       decode_error_count      = 0;
uint32_t       decode_success_count    = 0;
bool           is_controller_connected = true;
static service::Watchdog controller_watchdog;

float LX_T = 0.0f;
float LY_T = 0.0f;
float RX_T = 0.0f;

int16_t LX = 0;
int16_t LY = 0;
int16_t RX = 0;
int16_t RY = 0;

uint32_t button     = 0;
uint8_t  DIP_switch = 0;
uint8_t  crc        = 0;
cmd_vel  joystick_vel{};
static mode control_mode = MANUAL;

static float              g_auto_align_target_x     = 0.0f;
static float              g_auto_align_target_y     = 0.0f;
static float              g_auto_align_target_yaw   = 0.0f;
static Control_Mode       g_auto_align_control_mode = VEL_Control;
static Chassis_Velocity_t g_auto_align_chassis_v    = { 0.0f, 0.0f, 0.0f };
static uint8_t            g_auto_mode_status = 0U; // 自动对准状态：0=未激活, 1=detect, 2=apriltag

static bool g_emergency_hold_active      = false; // 紧急停止锁止状态
static bool g_auto_align_pos_target_sent = false;

static void ResetAutoAlignControlOutput()
{
    g_auto_align_target_x        = 0.0f;
    g_auto_align_target_y        = 0.0f;
    g_auto_align_target_yaw      = 0.0f;
    g_auto_align_control_mode    = VEL_Control;
    g_auto_align_chassis_v       = { 0.0f, 0.0f, 0.0f };
    g_auto_mode_status           = 0U;
    g_auto_align_pos_target_sent = false;
    g_emergency_hold_active      = false;
}

osThreadId_t         controllerHandle;
const osThreadAttr_t controller_attributes = {
    .name       = "controller",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityHigh,
};

static uint8_t CRC8(const uint8_t* data, uint8_t len)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        value ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (value & 0x80U)
            {
                value = (uint8_t)((value << 1U) ^ 0x07U);
            }
            else
            {
                value <<= 1U;
            }
        }
    }

    return value;
}

static void Msg_AddReadIndex(uint8_t length)
{
    readIndex = (uint8_t)((readIndex + length) % ProjectControllerConfig::RingBufferSize);
}

static uint8_t Msg_Read(uint8_t offset)
{
    const uint8_t index =
            (uint8_t)((readIndex + offset) % ProjectControllerConfig::RingBufferSize);
    return RX_RING_BUFFER[index];
}

static uint8_t Msg_GetLength()
{
    return (uint8_t)((writeIndex + ProjectControllerConfig::RingBufferSize - readIndex) %
                     ProjectControllerConfig::RingBufferSize);
}

static uint8_t Msg_GetRemain()
{
    return (uint8_t)(ProjectControllerConfig::RingBufferSize - Msg_GetLength());
}

static uint8_t Msg_Write(uint8_t* data, uint8_t length)
{
    if (Msg_GetRemain() < length)
    {
        return 0;
    }

    if (writeIndex + length < ProjectControllerConfig::RingBufferSize)
    {
        memcpy(RX_RING_BUFFER + writeIndex, data, length);
        writeIndex = (uint8_t)(writeIndex + length);
    }
    else
    {
        const uint8_t firstPart = (uint8_t)(ProjectControllerConfig::RingBufferSize - writeIndex);
        memcpy(RX_RING_BUFFER + writeIndex, data, firstPart);
        memcpy(RX_RING_BUFFER, data + firstPart, length - firstPart);
        writeIndex = (uint8_t)(length - firstPart);
    }

    writeIndex = (uint8_t)(writeIndex % ProjectControllerConfig::RingBufferSize);
    return length;
}

static bool Msg_SyncToHeader()
{
    while (Msg_GetLength() >= ProjectControllerConfig::RawDataSize)
    {
        if (Msg_Read(0) == ProjectControllerConfig::FrameHeader1 &&
            Msg_Read(1) == ProjectControllerConfig::FrameHeader2)
        {
            return true;
        }
        Msg_AddReadIndex(1);
    }

    return false;
}

static void Button_Init()
{
    button     = 0;
    DIP_switch = 0;
}

static bool IsButtonPressed(uint8_t bit)
{
    return (button & (1UL << bit)) != 0U;
}

static float Joystick2Velocity(int16_t joystick_value)
{
    if (std::abs(joystick_value) < ChassisConfig::JOYSTICK_RAW_MIDDLE)
    {
        return joystick_value * ChassisConfig::MIDDLE_VEL / ChassisConfig::JOYSTICK_RAW_MIDDLE;
    }

    return (joystick_value -
            (joystick_value > 0 ? ChassisConfig::JOYSTICK_RAW_MIDDLE
                                : -ChassisConfig::JOYSTICK_RAW_MIDDLE)) *
                   (ChassisConfig::MAX_VEL - ChassisConfig::MIDDLE_VEL) /
                   (ChassisConfig::JOYSTICK_RAW_MAX - ChassisConfig::JOYSTICK_RAW_MIDDLE) +
           (joystick_value > 0 ? ChassisConfig::MIDDLE_VEL : -ChassisConfig::MIDDLE_VEL);
}

static float Joystick2Wz(int16_t joystick_value)
{
    if (std::abs(joystick_value) < ChassisConfig::JOYSTICK_RAW_MIDDLE)
    {
        return joystick_value * ChassisConfig::MIDDLE_WZ / ChassisConfig::JOYSTICK_RAW_MIDDLE;
    }

    return (joystick_value -
            (joystick_value > 0 ? ChassisConfig::JOYSTICK_RAW_MIDDLE
                                : -ChassisConfig::JOYSTICK_RAW_MIDDLE)) *
                   (ChassisConfig::MAX_WZ - ChassisConfig::MIDDLE_WZ) /
                   (ChassisConfig::JOYSTICK_RAW_MAX - ChassisConfig::JOYSTICK_RAW_MIDDLE) +
           (joystick_value > 0 ? ChassisConfig::MIDDLE_WZ : -ChassisConfig::MIDDLE_WZ);
}

extern "C" void controller_task(void* argument)
{
    (void)argument;

    static uint16_t prev_buttons = 0;
    while (1)
    {
        while (Msg_SyncToHeader())
        {
            decode_count++;

            uint8_t receive_data[ProjectControllerConfig::RawDataSize - 3U];
            for (size_t i = 0; i < ProjectControllerConfig::RawDataSize - 3U; i++)
            {
                receive_data[i] = Msg_Read((uint8_t)(i + 2U));
            }

            const uint8_t calculate_crc =
                    CRC8(receive_data, (uint8_t)(ProjectControllerConfig::RawDataSize - 3U));
            const uint8_t received_crc = Msg_Read(13);

            if (calculate_crc == received_crc)
            {
                LX = (int16_t)((Msg_Read(2) << 8) | Msg_Read(3));
                LY = (int16_t)((Msg_Read(4) << 8) | Msg_Read(5));
                RX = (int16_t)((Msg_Read(6) << 8) | Msg_Read(7));
                RY = (int16_t)((Msg_Read(8) << 8) | Msg_Read(9));

                joystick_vel.vel_y  = -1.0f * Joystick2Velocity(LY);
                joystick_vel.vel_x  = Joystick2Velocity(LX);
                joystick_vel.vel_wz = -1.0f * Joystick2Wz(RY);

                // 解析拨码开关数据
                DIP_switch               = Msg_Read(10);
                uint16_t curr_buttons    = (static_cast<uint16_t>(Msg_Read(11)) << 8) |
                                           Msg_Read(12); // 目前按钮状态
                uint16_t falling_buttons = static_cast<uint16_t>(
                        prev_buttons & ~curr_buttons); // 按钮抬起时才会触发
                button               = static_cast<uint32_t>(curr_buttons) |
                                       (static_cast<uint32_t>(DIP_switch) << 16);
                uint32_t event_flags = static_cast<uint32_t>(falling_buttons) |
                                       (static_cast<uint32_t>(DIP_switch) << 16);
                osEventFlagsSet(flags_id, event_flags);
                prev_buttons = curr_buttons;

                decode_success_count++;
                controller_watchdog.feed(500);
                Msg_AddReadIndex(ProjectControllerConfig::RawDataSize);
            }
            else
            {
                Msg_AddReadIndex(2);
                decode_error_count++;
            }
        }

        osDelay(1);
    }
}

void app_ControllerReceive_init(void)
{
    control_mode = MANUAL;
    Button_Init();
    osThreadNew(controller_task, NULL, &controller_attributes);
    HAL_UART_Receive_DMA(&huart1, buffer, ProjectControllerConfig::RawDataSize);
}

void ControllerReceive_OnRxCplt()
{
    Msg_Write(buffer, ProjectControllerConfig::RawDataSize);
}

void softTIM_controller()
{
    // button0x00000008U用于触发切换到自动对准
    if ((osEventFlagsWait(flags_id, 0x00000008U, osFlagsWaitAny, 0) & 0xFF000008U) == 0x00000008U)
    {
        if (control_mode == MANUAL)
        {
            control_mode = AUTO_AIM;
            VisionAutoAlign_OnModeEnter();
            ResetAutoAlignControlOutput();
        }
        else if (control_mode == AUTO_AIM)
        {
            control_mode = MANUAL;
            VisionAutoAlign_ResetState();
            ResetAutoAlignControlOutput();
        }
    }
    switch (control_mode)
    {
    case MANUAL:
        Chassis::chassis_ctrl_->setVelocityInBody({ .vx = joystick_vel.vel_x,
                                                    .vy = joystick_vel.vel_y,
                                                    .wz = joystick_vel.vel_wz },
                                                  false);
        break;

    case AUTO_AIM:
    {
        // 当某一个摇杆映射大于0.1m/s或者0.1rad/s时，认为是人为干预，立即放弃自动对齐，切换回手动模式
        if (std::abs(joystick_vel.vel_x) > 0.1f || std::abs(joystick_vel.vel_y) > 0.1f ||
            std::abs(joystick_vel.vel_wz) > 0.1f)
        {
            control_mode = MANUAL;
            VisionAutoAlign_ResetState();
            ResetAutoAlignControlOutput();
            return;
        }

        // 调用自动对准逻辑（传入真实按钮状态与紧急停止标志）
        VisionAutoAlign_RunMode(button,                  // button_status（uint32_t 按键掩码）
                                g_emergency_hold_active, // button8_pressed（bool 紧急停止标志）
                                &g_auto_align_target_x,
                                &g_auto_align_target_y,
                                &g_auto_align_target_yaw,
                                &g_auto_align_control_mode,
                                &g_auto_align_chassis_v,
                                &g_auto_mode_status);

        // 仅在自动对准模式位置环控制下才应用位置控制，否则保持速度控制
        if (g_auto_align_control_mode == POS_Control)
        {
            // 位置目标只下发一次，后续由 Master 的 profile/error 快环推进和跟踪。
            if (!g_auto_align_pos_target_sent)
            {
                const chassis::Posture target_posture = { .x   = g_auto_align_target_x,
                                                          .y   = g_auto_align_target_y,
                                                          .yaw = g_auto_align_target_yaw };
                Chassis::chassis_ctrl_->setTargetPostureInWorld(
                        target_posture, Chassis::Master::defaultTrajectoryLinkMode);
                g_auto_align_pos_target_sent = true;
            }
        }
        else
        {
            g_auto_align_pos_target_sent = false;
            // 速度环控制：直接使用目标速度（转换类型：Chassis_Velocity_t → chassis::Velocity）
            Chassis::chassis_ctrl_->setVelocityInBody(
                    chassis::Velocity{ .vx = g_auto_align_chassis_v.vx,
                                       .vy = g_auto_align_chassis_v.vy,
                                       .wz = g_auto_align_chassis_v.wz },
                    false);
        }
        break;
    }

    default:
        break;
    }
}

void update_1kHz()
{
    if (!controller_watchdog.isFed())
    {
        is_controller_connected = false;
        joystick_vel.vel_x      = 0.0f;
        joystick_vel.vel_y      = 0.0f;
        joystick_vel.vel_wz     = 0.0f;
    }
    else
    {
        is_controller_connected = true;
    }
}

} // namespace Controller
