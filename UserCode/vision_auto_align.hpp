#ifndef __VISION_AUTO_ALIGN_HPP__
#define __VISION_AUTO_ALIGN_HPP__

#include <stdbool.h>
#include <stdint.h>
#include "chassis.hpp"

// 自动对准控制模式
enum Control_Mode
{
    VEL_Control = 0,  // 速度控制模式
    POS_Control = 1   // 位置控制模式
};

// 底盘速度结构体
struct Chassis_Velocity_t
{
    float vx;  // 前进方向速度 (m/s)
    float vy;  // 左侧方向速度 (m/s)
    float wz;  // 旋转角速度 (deg/s)
};

void VisionAutoAlign_OnModeEnter(void);
void VisionAutoAlign_ResetState(void);

void VisionAutoAlign_RunMode(uint32_t button_status,
                             bool button8_pressed,
                             float *target_x,
                             float *target_y,
                             float *target_yaw,
                             Control_Mode *chassis_control_mode,
                             Chassis_Velocity_t *chassis_v,
                             uint8_t *auto_mode);

#endif
