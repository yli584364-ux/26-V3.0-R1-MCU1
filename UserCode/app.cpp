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
#include "device.hpp"
#include "controller.hpp"
#include "flags.hpp"

osThreadId_t         softTIMHandle;
const osThreadAttr_t softTIM_attributes = {
    .name       = "softTIM",
    .stack_size = 256 * 4,
    .priority   = (osPriority_t)osPriorityRealtime7,
};

////////////////////////一些回调处理函数////////////////////////

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    service::Watchdog::EatAll(); // 看门狗吃狗
    Controller::update_1kHz();   // 遥控器状态更新
    Chassis::update_1kHz();      // 底盘控制更新
    Device::update_1kHz();       // 电机更新
}

extern "C" void softTIM(void* argument)
{
    while (1)
    {
        Controller::softTIM_controller(); // 软定时器，用来更新底盘的目标速度
        osDelay(10);
    }
}

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

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_EXTI_Callback(GPIO_Pin); // 光电门的外部中断
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    // UART错误处理
    if (huart->Instance == USART2)
    {
        CammeraReceive_OnError(huart);
    }
}

/////////////////////////////////////////////////////////////

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    Controller::app_ControllerReceive_init();
    CammeraReceive_Init(); // 初始化视觉接收
    Device::app_device_init();
    Chassis::app_chassis_init();
    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    Device::waitAllConnected();
    osDelay(3000);

    Chassis::ctrl_init();

    Chassis::chassis_->enable();
    Chassis::chassis_->startCalibration();

    while (!Chassis::chassis_->isReady())
        osDelay(1);
    osDelay(3000);
    Chassis::chassis_ctrl_->enable();
    osThreadNew(softTIM, NULL, &softTIM_attributes);
    /* 初始化完成后退出线程 */
    osThreadExit();
}
