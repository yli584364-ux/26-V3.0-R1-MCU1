#pragma once

#include "Master.hpp"
#include "gpio_driver.h"
#include "pid_motor.hpp"

#include <cstdint>

namespace AppConfig
{

namespace Controller
{
// 遥控器一帧原始数据长度。
inline constexpr uint8_t RawDataSize    = 14U;
// 遥控器串口环形缓冲区大小。
inline constexpr uint8_t RingBufferSize = 64U;
// 遥控器协议帧头。
inline constexpr uint8_t FrameHeader1   = 0xAAU;
inline constexpr uint8_t FrameHeader2   = 0xBBU;
} // namespace Controller

namespace Vision
{
// 视觉诊断缓冲区长度。
inline constexpr uint8_t  RxBufferSize        = 128U;
// 视觉数据缓存槽位数量。
inline constexpr int      DataMaxNum          = 10;
// 摄像头是否倒置。
inline constexpr bool     CameraReversed      = true;
// 固定长度视觉帧帧头和帧长度。
inline constexpr uint8_t  FrameHeader         = 0xAAU;
inline constexpr uint8_t  FrameSize           = 15U;
// UART 接收环形缓冲区大小。
inline constexpr uint16_t RxRingSize          = 256U;
// 相机坐标系到车体坐标系的平移偏移，单位米。
inline constexpr float    CameraToBodyOffsetX = 0.28f;
inline constexpr float    CameraToBodyOffsetY = 0.0f;
inline constexpr float    CameraToBodyOffsetZ = 0.0f;
// 机械臂基座到车体坐标系的平移偏移，单位米。
inline constexpr float    ArmToBodyOffsetX    = 0.83f;
inline constexpr float    ArmToBodyOffsetY    = 0.35f;
inline constexpr float    ArmToBodyOffsetZ    = 0.0f;
// 相机/机械臂相对车体的 yaw 角偏移，单位度。
inline constexpr int      YawCameraToBodyDeg  = 0;
inline constexpr int      YawArmToBodyDeg     = 0;
} // namespace Vision

namespace VisionAutoAlign
{
// 连续丢失视觉数据的周期阈值。
inline constexpr uint32_t LostCycleThreshold  = 10U;
// 自动对准滑动窗口容量。
inline constexpr uint32_t WindowCapacity      = 20U;
// 离群点阈值。
inline constexpr float    OutlierThresholdM   = 0.05f;
inline constexpr float    OutlierThresholdDeg = 2.0f;
// 视觉位置/朝向滤波参数。
inline constexpr float    PositionLpfAlpha    = 0.85f;
inline constexpr float    MaxStepPerCycleM    = 0.03f;
inline constexpr float    MaxStepPerCycleDeg  = 12.0f;
inline constexpr float    PositionDeadbandM   = 0.03f;
inline constexpr float    YawLpfAlpha         = 0.60f;
inline constexpr float    YawDeadbandDeg      = 0.8f;
} // namespace VisionAutoAlign

namespace Chassis
{
// 四个光电门输入。
inline const GPIO_t FrontPhotogate{ GPIOB, GPIO_PIN_0 };
inline const GPIO_t LeftPhotogate{ GPIOB, GPIO_PIN_1 };
inline const GPIO_t RearPhotogate{ GPIOA, GPIO_PIN_7 };
inline const GPIO_t RightPhotogate{ GPIOA, GPIO_PIN_6 };
inline constexpr GPIO_PinState PhotogateActiveState = GPIO_PIN_SET;

// 遥控杆映射相关速度参数。
inline constexpr float MiddleVel         = 1.0f;
inline constexpr float MaxVel            = 8.0f;
inline constexpr float MiddleWz          = 90.0f;
inline constexpr float MaxWz             = 360.0f;
inline constexpr float JoystickRawMiddle = 600.0f;
inline constexpr float JoystickRawMax    = 1600.0f;

// 舵向轮速度环 PID。
inline const PIDMotor::Config WheelDirVelocityPid{
    .Kp = 500.0f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .abs_output_max = 8000.0f,
};

// 舵向轮位置环 PID。
inline const PIDMotor::Config WheelDirPositionPid{
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.2f,
    .abs_output_max = 400.0f,
};

// 底盘几何参数与舵向安装角。
inline constexpr float Radius               = 45.0f;
inline constexpr float DistanceX            = 619.8f;
inline constexpr float DistanceY            = 580.0f;
inline constexpr float FrontRightSteerAngle = 135.0f;
inline constexpr float FrontLeftSteerAngle  = -45.0f;
inline constexpr float RearLeftSteerAngle   = 135.0f;
inline constexpr float RearRightSteerAngle  = 135.0f;

// 主控制器参数，包括位姿误差 PD 和轨迹约束。
inline const chassis::controller::Master::Config ControllerCfg = {
    .posture_error_pd_cfg =
            {
                    .vx = { .Kp = 5.0f, .Kd = 3.0f, .abs_output_max = 0.1f },
                    .vy = { .Kp = 5.0f, .Kd = 3.0f, .abs_output_max = 0.1f },
                    .wz = { .Kp = 30.0f, .Kd = 4.0f, .abs_output_max = 25.0f },
            },
    .limit =
            {
                    .x   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
                    .y   = { .max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f },
                    .yaw = { .max_spd = 90.0f, .max_acc = 45.0f, .max_jerk = 90.0f },
            },
};
} // namespace Chassis

} // namespace AppConfig
