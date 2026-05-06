#include "vision_auto_align.hpp"

#include "vision_receive.hpp"

#include <cmath>

namespace VisionAutoAlignConfig = AppConfig::VisionAutoAlign;

static uint32_t g_vision_last_update_seq = 0U;
static uint32_t g_vision_stale_cycles    = 0U;

static bool     g_auto_align_pos_executed_once = false;
static uint32_t g_auto_align_window_count      = 0U;
static uint32_t g_auto_align_last_sample_seq   = 0U;
static float    g_auto_align_window_x          = 0.0f;
static float    g_auto_align_window_y          = 0.0f;
static float    g_auto_align_window_yaw        = 0.0f;

static bool  g_step_cmd_active       = false;
static bool  g_emergency_hold_active = false;
static bool  g_vision_filter_inited  = false;
static float g_target_x_filtered     = 0.0f;
static float g_target_y_filtered     = 0.0f;

static inline float ClampFloat(float value,
                               float min_value,
                               float max_value) // 窗口滤波时使用，限制每周期最大调整量
{
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

static void ApplyVisionTargetFilter(float raw_x, float raw_y, float* out_x, float* out_y)
{
    if (!out_x || !out_y)
    {
        return;
    }

    if (!g_vision_filter_inited)
    {
        g_target_x_filtered    = raw_x;
        g_target_y_filtered    = raw_y;
        g_vision_filter_inited = true;
    }

    const float dx = ClampFloat(raw_x - g_target_x_filtered,
                                -VisionAutoAlignConfig::MaxStepPerCycleM,
                                VisionAutoAlignConfig::MaxStepPerCycleM);
    const float dy = ClampFloat(raw_y - g_target_y_filtered,
                                -VisionAutoAlignConfig::MaxStepPerCycleM,
                                VisionAutoAlignConfig::MaxStepPerCycleM);

    g_target_x_filtered += VisionAutoAlignConfig::PositionLpfAlpha * dx;
    g_target_y_filtered += VisionAutoAlignConfig::PositionLpfAlpha * dy;

    if (fabsf(g_target_x_filtered) < VisionAutoAlignConfig::PositionDeadbandM)
    {
        g_target_x_filtered = 0.0f;
    }
    if (fabsf(g_target_y_filtered) < VisionAutoAlignConfig::PositionDeadbandM)
    {
        g_target_y_filtered = 0.0f;
    }

    *out_x = g_target_x_filtered;
    *out_y = g_target_y_filtered;
}

static void ApplyYawTargetFilter(float raw_yaw, float* yaw)
{
    if (!yaw)
    {
        return;
    }

    const float dyaw = ClampFloat(raw_yaw - *yaw,
                                  -VisionAutoAlignConfig::MaxStepPerCycleDeg,
                                  VisionAutoAlignConfig::MaxStepPerCycleDeg);
    float yaw_output = *yaw + VisionAutoAlignConfig::YawLpfAlpha * dyaw;

    if (fabsf(yaw_output) < VisionAutoAlignConfig::YawDeadbandDeg)
    {
        yaw_output = 0.0f;
    }

    *yaw = yaw_output;
}

static void AbortAutoAlignAndStop(float*              target_x,
                                  float*              target_y,
                                  float*              target_yaw,
                                  Control_Mode*       chassis_control_mode,
                                  Chassis_Velocity_t* chassis_v,
                                  uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return;
    }

    VisionAutoAlign_ResetState();
    g_step_cmd_active     = false;
    *auto_mode            = 0;
    *target_x             = 0.0f;
    *target_y             = 0.0f;
    *target_yaw           = 0.0f;
    *chassis_control_mode = VEL_Control;
    chassis_v->vx         = 0.0f;
    chassis_v->vy         = 0.0f;
    chassis_v->wz         = 0.0f;
}

void VisionAutoAlign_ResetState(void)
{
    g_vision_last_update_seq       = 0U;
    g_vision_stale_cycles          = 0U;
    g_auto_align_pos_executed_once = false;
    g_auto_align_window_count      = 0U;
    g_auto_align_last_sample_seq   = 0U;
    g_auto_align_window_x          = 0.0f;
    g_auto_align_window_y          = 0.0f;
    g_auto_align_window_yaw        = 0.0f;
    g_step_cmd_active              = false;
    g_emergency_hold_active        = false;
    g_vision_filter_inited         = false;
    g_target_x_filtered            = 0.0f;
    g_target_y_filtered            = 0.0f;
}

void VisionAutoAlign_OnModeEnter(void)
{
    VisionAutoAlign_ResetState();
}

static bool VisionAutoAlign_Apply(float*              target_x,
                                  float*              target_y,
                                  float*              target_yaw,
                                  Control_Mode*       chassis_control_mode,
                                  Chassis_Velocity_t* chassis_v,
                                  uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return false;
    }

    chassis_v->vx = 0.0f;
    chassis_v->vy = 0.0f;
    chassis_v->wz = 0.0f;

    if (g_auto_align_pos_executed_once)
    {
        return true;
    }

    const uint32_t apriltag_seq = lr_apriltag_update_seq;
    const uint32_t detect_seq   = lr_detect_update_seq;
    const uint32_t latest_seq   = apriltag_seq >= detect_seq ? apriltag_seq : detect_seq;
    const bool     has_apriltag = apriltag_seq > 0U;
    const bool     has_detect   = detect_seq > 0U;

    if (!has_apriltag && !has_detect)
    {
        *auto_mode = 0;
        return false;
    }

    if (g_vision_last_update_seq == 0U || latest_seq != g_vision_last_update_seq)
    {
        g_vision_last_update_seq = latest_seq;
        g_vision_stale_cycles    = 0U;
    }
    else
    {
        if (g_vision_stale_cycles < 0xFFFFFFFFU)
        {
            g_vision_stale_cycles++;
        }
        if (g_vision_stale_cycles > VisionAutoAlignConfig::LostCycleThreshold)
        {
            *auto_mode = 0;
            return false;
        }
    }

    const bool use_apriltag = has_apriltag && (!has_detect || (apriltag_seq >= detect_seq));
    *auto_mode              = use_apriltag ? 2U : 1U;
    g_step_cmd_active       = false;

    if (latest_seq == g_auto_align_last_sample_seq)
    {
        return true;
    }
    g_auto_align_last_sample_seq = latest_seq;

    LR_DataPacket src = { 0 };
    if (use_apriltag)
    {
        const int latest_idx = (lr_apriltag_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
        src                  = lr_apriltag_buffer[latest_idx];
    }
    else
    {
        const int latest_idx = (lr_detect_write_idx + LR_DATA_MAX_NUM - 1) % LR_DATA_MAX_NUM;
        src                  = lr_detect_buffer[latest_idx];
    }

    float sample_target_x   = 0.0f;
    float sample_target_y   = 0.0f;
    float sample_target_yaw = 0.0f;
    LR_Compute_Target(
            src.x, src.y, src.z, src.yaw, &sample_target_x, &sample_target_y, &sample_target_yaw);

    ApplyVisionTargetFilter(sample_target_x, sample_target_y, &sample_target_x, &sample_target_y);
    ApplyYawTargetFilter(sample_target_yaw, &sample_target_yaw);

    if (g_auto_align_window_count > 0U)
    {
        const float window_avg_x   = g_auto_align_window_x / (float)g_auto_align_window_count;
        const float window_avg_y   = g_auto_align_window_y / (float)g_auto_align_window_count;
        const float window_avg_yaw = g_auto_align_window_yaw / (float)g_auto_align_window_count;

        const float delta_x   = fabsf(sample_target_x - window_avg_x);
        const float delta_y   = fabsf(sample_target_y - window_avg_y);
        const float delta_yaw = fabsf(sample_target_yaw - window_avg_yaw);
        const float delta_pos = sqrtf(delta_x * delta_x + delta_y * delta_y);

        if (delta_pos > VisionAutoAlignConfig::OutlierThresholdM ||
            delta_yaw > VisionAutoAlignConfig::OutlierThresholdDeg)
        {
            return true;
        }
    }

    g_auto_align_window_x += sample_target_x;
    g_auto_align_window_y += sample_target_y;
    g_auto_align_window_yaw += sample_target_yaw;
    g_auto_align_window_count++;

    if (g_auto_align_window_count < VisionAutoAlignConfig::WindowCapacity)
    {
        return true;
    }

    const float inv_count = 1.0f / (float)g_auto_align_window_count;
    *target_x             = g_auto_align_window_x * inv_count;
    *target_y             = g_auto_align_window_y * inv_count;
    *target_yaw           = g_auto_align_window_yaw * inv_count;
    *chassis_control_mode = POS_Control;
    g_auto_align_pos_executed_once = true;

    return true;
}

void VisionAutoAlign_RunMode(uint32_t            button_status,
                             bool                button_pressed,
                             float*              target_x,
                             float*              target_y,
                             float*              target_yaw,
                             Control_Mode*       chassis_control_mode,
                             Chassis_Velocity_t* chassis_v,
                             uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !chassis_v || !auto_mode)
    {
        return;
    }

    if (button_pressed || ((button_status & (1U << 8)) != 0U))
    {
        g_emergency_hold_active = true;
    }

    if (g_emergency_hold_active)
    {
        AbortAutoAlignAndStop(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v, auto_mode);
        if (!button_pressed)
        {
            g_emergency_hold_active = false;
        }
    }
    else
    {
        VisionAutoAlign_Apply(
                target_x, target_y, target_yaw, chassis_control_mode, chassis_v, auto_mode);
    }
}
