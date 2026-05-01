#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "Steering4.hpp"
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

#define GPIO_FRONT (GPIO_t{ GPIOA, GPIO_PIN_6 })
#define GPIO_LEFT  (GPIO_t{ GPIOB, GPIO_PIN_1 })
#define GPIO_REAR  (GPIO_t{ GPIOB, GPIO_PIN_0 })
#define GPIO_RIGHT (GPIO_t{ GPIOA, GPIO_PIN_7 })

// #define GPIO_FRONT (GPIO_t{GPIOA, GPIO_PIN_0})
// #define GPIO_LEFT  (GPIO_t{GPIOA, GPIO_PIN_1})
// #define GPIO_REAR  (GPIO_t{GPIOA, GPIO_PIN_2})
// #define GPIO_RIGHT (GPIO_t{GPIOA, GPIO_PIN_3})

struct ChassisConfig
{
    static constexpr float MIDDLE_VEL = 1.0f;   // 遥控器摇杆数据转换后中位速度值，单位m/s
    static constexpr float MAX_VEL    = 8.0f;   // 遥控器摇杆数据转换后最大速度值，单位m/s
    static constexpr float MIDDLE_WZ  = 90.0f;  // 遥控器摇杆数据转换后中位角速度值，单位deg/s
    static constexpr float MAX_WZ     = 360.0f; // 遥控器摇杆数据转换后最大角速度值，单位deg/s
    static constexpr float JOYSTICK_RAW_MIDDLE = 600.0f;
    static constexpr float JOYSTICK_RAW_MAX    = 1600.0f;
};

inline Steering4*   chassis_;
inline JustEncoder* chassis_loc_;
inline Master*      chassis_ctrl_;

void APP_CHASSIS_Init();

void Ctrl_Init();

void update_1kHz();
void update_100Hz();

} // namespace Chassis

#endif // _CHASSIS_H_