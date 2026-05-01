#include "device.hpp"
#include "can.h"
#include "can_driver.hpp"
#include "cmsis_os2.h"

namespace Device
{

static void can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);

    motors::VESCMotor::CAN_FilterInit(&hcan2, 14);
    CAN_RegisterCallback(&hcan2, motors::VESCMotor::CANBaseReceiveCallback);

    CAN_InitMainCallback(&hcan1);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_InitMainCallback(&hcan2);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

constexpr motors::DJIMotor::Config motor_wheel_dir_config[4] = {
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M2006_C610,
            .id1            = 1,
            .reverse        = true,
            .reduction_rate = 2.0f, // 舵轮底盘2006电机外接减速比为2:1
    },
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M2006_C610,
            .id1            = 2,
            .reverse        = true,
            .reduction_rate = 2.0f,
    },
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M2006_C610,
            .id1            = 3,
            .reverse        = true,
            .reduction_rate = 2.0f,
    },
    {
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M2006_C610,
            .id1            = 4,
            .reverse        = true,
            .reduction_rate = 2.0f,
    },
};

constexpr motors::VESCMotor::Config motor_wheel_speed_config[4] = {
    {
            .hcan           = &hcan2,
            .id             = 0x01,
            .electrodes     = 14,
            .reduction_rate = 1.64f,
            .reverse        = true,

    },
    {
            .hcan           = &hcan2,
            .id             = 0x02,
            .electrodes     = 14,
            .reduction_rate = 1.64f,
    },
    {
            .hcan           = &hcan2,
            .id             = 0x03,
            .electrodes     = 14,
            .reduction_rate = 1.64f,
    },
    {
            .hcan           = &hcan2,
            .id             = 0x04,
            .electrodes     = 14,
            .reduction_rate = 1.64f,
            .reverse        = true,

    },
};

static void motors_init()
{
    for (size_t i = 0; i < 4; ++i)
    {
        motor::motor_wheel_speed[i] = new motors::VESCMotor(motor_wheel_speed_config[i]);
        motor::motor_wheel_dir[i]   = new motors::DJIMotor(motor_wheel_dir_config[i]);
    }
}

void app_device_init()
{
    can_init();
    motors_init();
}

void update_1kHz()
{
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}

bool isAllConnected()
{
    constexpr auto def_and_connected = [](auto a) { return a && a->isConnected(); };

    for (auto& m : motor::motor_wheel_speed)
        if (!def_and_connected(m))
            return false;
    for (auto& m : motor::motor_wheel_dir)
        if (!def_and_connected(m))
            return false;
    return true;
}

void waitAllConnected()
{
    while (!isAllConnected())
        osDelay(1);
}

} // namespace Device