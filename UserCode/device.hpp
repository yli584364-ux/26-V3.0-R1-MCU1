#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "vesc.hpp"
#include "dji.hpp"

namespace Device
{
namespace motor
{
// 底盘电机
inline motors::VESCMotor* motor_wheel_speed[4]; // 底盘轮子电机
inline motors::DJIMotor*  motor_wheel_dir[4];   // 底盘舵向电机
} // namespace motor

void app_device_init();  // 总体设备初始化函数
void update_1kHz();      // 设备状态更新函数，用于在中断中调用
bool isAllConnected();   // 检测所有设备是否连接上
void waitAllConnected(); // 等待设备连接，可放在线程中阻塞线程

} // namespace Device

#endif // _DEVICE_H_