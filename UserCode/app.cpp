/**
 * @file    app.h
 * @author  rediduck
 * @date    2026-05-1
 */
#include "cmsis_os2.h"
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
    service::Watchdog::EatAll();
    Chassis::update_1kHz();
    Device::update_1kHz();
}

extern "C" void softTIM(void* argument)
{
    while (1)
    {
        Controller::softTIM_controller();
        osDelay(10);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Controller::ControllerReceive_OnRxCplt();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_EXTI_Callback(GPIO_Pin);
}

/////////////////////////////////////////////////////////////

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    flags_create();
    Controller::app_controller_receive_init();
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
