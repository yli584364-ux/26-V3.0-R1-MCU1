#include "chassis.hpp"
#include "device.hpp"
#include "motor_pos_controller.hpp"
#include "pid_motor.hpp"
#include "motor_if.hpp"

namespace Chassis
{

using controllers::ControlMode;

static PIDMotor::Config motor_wheeldir_velpid = {
    .Kp = 500.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 8000
};

static PIDMotor::Config motor_wheeldir_pospid = {
    .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 400
};

MotorVelController* motor_wheelspeed_velctrl[4] = { nullptr };
MotorPosController* motor_wheeldir_posctrl[4]   = { nullptr };
MotorVelController* motor_wheeldir_velctrl[4]   = { nullptr };
// 初始化一个底盘
static void motion_init()
{
    for (size_t i = 0; i < 4; i++)
    {
        motor_wheeldir_posctrl[i] =
                new MotorPosController(Device::motor::motor_wheel_dir[i],
                                       {
                                               .position_pid       = motor_wheeldir_pospid,
                                               .velocity_pid       = motor_wheeldir_velpid,
                                               .pos_vel_freq_ratio = 10,
                                       });
        motor_wheeldir_velctrl[i]   = new MotorVelController(Device::motor::motor_wheel_dir[i],
                                                             { .pid = motor_wheeldir_velpid });
        motor_wheelspeed_velctrl[i] = new MotorVelController(
                Device::motor::motor_wheel_speed[i],
                { .ctrl_mode = ControlMode::InternalVel, .internal_set_ratio = 50 });
    }
    chassis_ = new Steering4(Steering4::Config{
            .enable_calibration = true,
            .radius             = 45.0f,
            .distance_x         = 619.8f,
            .distance_y         = 580.0f,
            .wheel_front_right =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[1],
                                            .steer_motor  = motor_wheeldir_posctrl[0],
                                            .steer_offset = 135.0f,

                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor            = motor_wheeldir_velctrl[0],
                                            .photogate              = GPIO_FRONT,
                                            .photogate_active_state = GPIO_PIN_SET,
                                    },

                    },
            .wheel_front_left =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[2],
                                            .steer_motor  = motor_wheeldir_posctrl[1],
                                            .steer_offset = -45.0f,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor            = motor_wheeldir_velctrl[1],
                                            .photogate              = GPIO_LEFT,
                                            .photogate_active_state = GPIO_PIN_SET,
                                    },
                    },
            .wheel_rear_left =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[0],
                                            .steer_motor  = motor_wheeldir_posctrl[2],
                                            .steer_offset = 135.0f,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor            = motor_wheeldir_velctrl[2],
                                            .photogate              = GPIO_REAR,
                                            .photogate_active_state = GPIO_PIN_SET,

                                    },
                    },
            .wheel_rear_right =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[3],
                                            .steer_motor  = motor_wheeldir_posctrl[3],
                                            .steer_offset = 135.0f,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor            = motor_wheeldir_velctrl[3],
                                            .photogate              = GPIO_RIGHT,
                                            .photogate_active_state = GPIO_PIN_SET,
                                    },
                    },

    });
}

static void loc_init()
{
    chassis_loc_ = new JustEncoder(*chassis_);
}

static void controller_init()
{
    chassis_ctrl_ =
            new Master(*chassis_,
                       *chassis_loc_,
                       {.posture_error_pd_cfg =
                                {
                                        .vx = {.Kp = 5.0f, .Kd = 3.0f, .abs_output_max = 0.1f},
                                        .vy = {.Kp = 5.0f, .Kd = 3.0f, .abs_output_max = 0.1f},
                                        .wz = {.Kp = 30.0f, .Kd = 4.0f, .abs_output_max = 25.0f},
                                },
                        .limit = {.x   = {.max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f},
                                  .y   = {.max_spd = 1.0f, .max_acc = 1.2f, .max_jerk = 20.0f},
                                  .yaw = {.max_spd = 90.0f, .max_acc = 45.0f, .max_jerk = 90.0f}}});
}

void app_chassis_init()
{
    motion_init(); // 底盘启动
}
void ctrl_init()
{
    loc_init();        // 定位启动
    controller_init(); // 底盘控制器启动
}
void update_1kHz()
{
    if (chassis_loc_)
        chassis_loc_->update(0.001f);
    if (chassis_ctrl_)
        chassis_ctrl_->controllerUpdate();

    chassis_->update();
}

} // namespace Chassis
