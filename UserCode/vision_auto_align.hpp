#ifndef __VISION_AUTO_ALIGN_HPP__
#define __VISION_AUTO_ALIGN_HPP__

#include <stdbool.h>
#include <stdint.h>
#include "chassis.hpp"

// 自动对准控制模式
enum Control_Mode
{
    VEL_Control = 0, // 速度控制模式
    POS_Control = 1  // 位置控制模式
};



void VisionAutoAlign_OnModeEnter(void);
void VisionAutoAlign_ResetState(void);

bool VisionAutoAlign_RunMode(float*              target_x,
                             float*              target_y,
                             float*              target_yaw,
                             Control_Mode*       chassis_control_mode,
                             uint8_t*            auto_mode);

#endif
