#include "chassis.hpp"
#include "device.hpp"
#include "motor_pos_controller.hpp"
#include "pid_motor.hpp"
#include "motor_if.hpp"

namespace Chassis
{

using controllers::ControlMode;
namespace ProjectChassisConfig = AppConfig::Chassis;

MotorVelController* motor_wheelspeed_velctrl[4] = { nullptr };
MotorPosController* motor_wheeldir_posctrl[4]   = { nullptr };
MotorVelController* motor_wheeldir_velctrl[4]   = { nullptr };

static void motion_init()
{
    for (size_t i = 0; i < 4; i++)
    {
        motor_wheeldir_posctrl[i] =
                new MotorPosController(Device::motor::motor_wheel_dir[i],
                                       {
                                               .position_pid =
                                                       ProjectChassisConfig::WheelDirPositionPid,
                                               .velocity_pid =
                                                       ProjectChassisConfig::WheelDirVelocityPid,
                                               .pos_vel_freq_ratio = 10,
                                       });
        motor_wheeldir_velctrl[i]   = new MotorVelController(
                Device::motor::motor_wheel_dir[i],
                { .pid = ProjectChassisConfig::WheelDirVelocityPid });
        motor_wheelspeed_velctrl[i] = new MotorVelController(
                Device::motor::motor_wheel_speed[i],
                { .ctrl_mode = ControlMode::InternalVel, .internal_set_ratio = 50 });
    }

    chassis_ = new Steering4(Steering4::Config{
            .enable_calibration = true,
            .radius             = ProjectChassisConfig::Radius,
            .distance_x         = ProjectChassisConfig::DistanceX,
            .distance_y         = ProjectChassisConfig::DistanceY,
            .wheel_front_right =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[1],
                                            .steer_motor  = motor_wheeldir_posctrl[0],
                                            .steer_offset = ProjectChassisConfig::FrontRightSteerAngle,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor = motor_wheeldir_velctrl[0],
                                            .photogate   = ProjectChassisConfig::FrontPhotogate,
                                            .photogate_active_state =
                                                    ProjectChassisConfig::PhotogateActiveState,
                                    },
                    },
            .wheel_front_left =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[2],
                                            .steer_motor  = motor_wheeldir_posctrl[1],
                                            .steer_offset = ProjectChassisConfig::FrontLeftSteerAngle,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor = motor_wheeldir_velctrl[1],
                                            .photogate   = ProjectChassisConfig::LeftPhotogate,
                                            .photogate_active_state =
                                                    ProjectChassisConfig::PhotogateActiveState,
                                    },
                    },
            .wheel_rear_left =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[0],
                                            .steer_motor  = motor_wheeldir_posctrl[2],
                                            .steer_offset = ProjectChassisConfig::RearLeftSteerAngle,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor = motor_wheeldir_velctrl[2],
                                            .photogate   = ProjectChassisConfig::RearPhotogate,
                                            .photogate_active_state =
                                                    ProjectChassisConfig::PhotogateActiveState,
                                    },
                    },
            .wheel_rear_right =
                    {
                            .cfg =
                                    {
                                            .drive_motor  = motor_wheelspeed_velctrl[3],
                                            .steer_motor  = motor_wheeldir_posctrl[3],
                                            .steer_offset = ProjectChassisConfig::RearRightSteerAngle,
                                    },
                            .calib_cfg =
                                    {
                                            .steer_motor = motor_wheeldir_velctrl[3],
                                            .photogate   = ProjectChassisConfig::RightPhotogate,
                                            .photogate_active_state =
                                                    ProjectChassisConfig::PhotogateActiveState,
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
    chassis_ctrl_ = new Master(*chassis_, *chassis_loc_, ProjectChassisConfig::ControllerCfg);
}

void app_chassis_init()
{
    motion_init();
}

void ctrl_init()
{
    loc_init();
    controller_init();
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
