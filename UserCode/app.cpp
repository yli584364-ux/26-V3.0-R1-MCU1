/**
 * @file    app.h
 * @author  rediduck
 * @date    2026-05-1
 */
#include "cmsis_os2.h"
#include "device.hpp"
#include "controller.hpp"
#include "vision_receive.hpp"
#include "tim.h"
#include "main.h"

#include "chassis.hpp"
#include "SteeringWheel.hpp"
#include "flags.hpp"

osThreadId_t         softTIMHandle;
const osThreadAttr_t softTIM_attributes = {
    .name       = "softTIM",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityRealtime7,
};

// 1kHz 软件时基回调：统一驱动看门狗、遥控器、底盘和电机更新。
extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    (void)htim;
    service::Watchdog::EatAll();
    Controller::update_1kHz();
    Chassis::update_1kHz();
    Device::update_1kHz();
}

// 10ms 周期任务：刷新底盘目标速度等较低频控制逻辑。
extern "C" void softTIM(void* argument)
{
    (void)argument;
    while (1)
    {
        Controller::softTIM_controller();
        osDelay(10);
    }
}

// 串口接收完成回调。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Controller::ControllerReceive_OnRxCplt();
    }
    else if (huart->Instance == USART2)
    {
        CammeraReceive_OnRxCplt(huart);
    }
}

// EXTI 回调转发给 GPIO 驱动。
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_EXTI_Callback(GPIO_Pin);
}

// UART 错误回调，当前仅处理视觉串口 USART2。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
        CammeraReceive_OnError(huart);
    }
}

// 应用初始化线程。
extern "C" void Init(void* argument)
{
    (void)argument;

    Controller::app_ControllerReceive_init(); // 初始化遥控器接收。
    CammeraReceive_Init();                    // 初始化视觉接收。
    Device::app_device_init();                // 初始化电机与设备。
    Chassis::app_chassis_init();              // 初始化底盘运动部分。

    // 启动 1kHz 基准定时器。
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    // 等待设备上电完成并建立连接。
    Device::waitAllConnected();
    osDelay(3000);

    // 初始化定位与底盘控制器。
    Chassis::ctrl_init();

    // 使能底盘并执行舵向标定。
    Chassis::chassis_->enable();
    Chassis::chassis_->startCalibration();

    while (!Chassis::chassis_->isReady())
    {
        osDelay(1);
    }

    osDelay(3000);
    Chassis::chassis_ctrl_->enable();
    osThreadNew(softTIM, NULL, &softTIM_attributes); // 启动低频控制任务。
    osThreadExit();
}
