#ifndef _CONTROLLER_RECEIVE_H_
#define _CONTROLLER_RECEIVE_H_

#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "chassis.hpp"

namespace Controller
{

static constexpr float max_joystick = 2000.0f; // 遥控器摇杆数据最大值

struct cmd_vel
{
    float vel_x;
    float vel_y;
    float vel_wz;
};

enum mode
{
    MANUAL   = 0,
    AUTO_AIM = 1,
};

void ControllerReceive_OnRxCplt();
void Controller_Receive_Init(void);
void softTIM_controller();

} // namespace Controller

#endif // _CONTROLLER_RECEIVE_H_