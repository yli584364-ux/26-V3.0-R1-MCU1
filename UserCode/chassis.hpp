#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "Steering4.hpp"
#include "config.hpp"
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "gpio_driver.h"
#include "JustEncoder.hpp"
#include "Master.hpp"

#pragma once

namespace Chassis
{

extern float vx;
extern float vy;
extern float wz;

using chassis::controller::Master;
using chassis::loc::JustEncoder;
using chassis::motion::Steering4;
using controllers::MotorPosController;
using controllers::MotorVelController;

struct ChassisConfig
{
    static constexpr float MIDDLE_VEL          = AppConfig::Chassis::MiddleVel;
    static constexpr float MAX_VEL             = AppConfig::Chassis::MaxVel;
    static constexpr float MIDDLE_WZ           = AppConfig::Chassis::MiddleWz;
    static constexpr float MAX_WZ              = AppConfig::Chassis::MaxWz;
    static constexpr float JOYSTICK_RAW_MIDDLE = AppConfig::Chassis::JoystickRawMiddle;
    static constexpr float JOYSTICK_RAW_MAX    = AppConfig::Chassis::JoystickRawMax;
};

inline Steering4*   chassis_;
inline JustEncoder* chassis_loc_;
inline Master*      chassis_ctrl_;

void app_chassis_init();
void ctrl_init();
void update_1kHz();

} // namespace Chassis

#endif // _CHASSIS_H_
