#include "vision_auto_align.hpp"

#include "vision_receive.hpp"
#include <cmath>

constexpr uint32_t kVisionLostCycleThreshold     = 10U;   // controller_task 10ms周期，约100ms
constexpr uint32_t kAutoAlignWindowCapacity      = 20U;   // 滤波窗口容量
constexpr float    kAutoAlignOutlierThresholdM   = 0.05f; // 位置离群值阈值，单位米
constexpr float    kAutoAlignOutlierThresholdDeg = 2.0f;  // 朝向离群值阈值，单位度

// 滤波系数和限幅值，自动对齐时每周期（10ms）允许的最大调整量，过大可能导致震荡，过小可能导致响应迟钝
constexpr float kVisionLpfAlpha           = 0.85f;
constexpr float kVisionMaxStepPerCycleM   = 0.03f;
constexpr float kVisionMaxStepPerCycleDeg = 12.0f;
constexpr float kVisionPosDeadbandM       = 0.03f;
constexpr float kVisionYawLpfAlpha        = 0.60f;
constexpr float kVisionYawDeadbandDeg     = 0.8f;

static uint32_t g_vision_stale_cycles        = 0U;
static uint32_t g_auto_align_window_count    = 0U; // 窗口中当前有效样本数
static uint32_t g_auto_align_last_sample_seq = 0U;
static float    g_auto_align_window_x        = 0.0f; // 窗口内X坐标累加和
static float    g_auto_align_window_y        = 0.0f; // 窗口内Y坐标累加和
static float    g_auto_align_window_yaw      = 0.0f; // 窗口内朝向累加和
static float g_target_x_filtered = 0.0f;
static float g_target_y_filtered = 0.0f;


void VisionAutoAlign_ResetState(void)
{
    g_vision_stale_cycles        = 0U;
    g_auto_align_window_count    = 0U;
    g_auto_align_last_sample_seq = 0U;
    g_auto_align_window_x        = 0.0f;
    g_auto_align_window_y        = 0.0f;
    g_auto_align_window_yaw      = 0.0f;
    g_target_x_filtered          = 0.0f;
    g_target_y_filtered          = 0.0f;
}

void VisionAutoAlign_OnModeEnter(void)
{
    VisionAutoAlign_ResetState();
}

// 应用视觉对齐坐标
static bool VisionAutoAlign_Apply(float*              target_x,
                                  float*              target_y,
                                  float*              target_yaw,
                                  Control_Mode*       chassis_control_mode,
                                  uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !auto_mode)
    {
        return false;
    }

    // 如果窗口已满，继续使用当前平均值输出，不再更新，当前解算值可以输出，true
    if (g_auto_align_window_count >= kAutoAlignWindowCapacity)
    {
        return true;
    }

    const uint32_t apriltag_seq = lr_apriltag_update_seq;
    const uint32_t detect_seq   = lr_detect_update_seq;
    const uint32_t latest_seq   = (apriltag_seq >= detect_seq) ? apriltag_seq : detect_seq;
    const int      has_apriltag = (apriltag_seq > 0U);
    const int      has_detect   = (detect_seq > 0U);
    if (!has_apriltag && !has_detect)
    {
        *auto_mode = 0;
        return false;
    }

    const bool has_new_sample = (latest_seq != g_auto_align_last_sample_seq);
    if (has_new_sample)
    {
        g_vision_stale_cycles = 0U;
    }
    else
    {
        if (g_vision_stale_cycles < 0xFFFFFFFFU)
        {
            g_vision_stale_cycles++;
        }
        if (g_vision_stale_cycles > kVisionLostCycleThreshold)
        {
            *auto_mode = 0;
            return false;
        }
    }

    const int use_apriltag = has_apriltag && (!has_detect || (apriltag_seq >= detect_seq));
    *auto_mode             = use_apriltag ? 2 : 1; // 2=apriltag, 1=detect

    if (!has_new_sample)
    {
        return false;
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

    // 窗口滤波：检查新样本是否为离群值
    if (g_auto_align_window_count > 0U)
    {
        const float window_avg_x   = g_auto_align_window_x / (float)g_auto_align_window_count;
        const float window_avg_y   = g_auto_align_window_y / (float)g_auto_align_window_count;
        const float window_avg_yaw = g_auto_align_window_yaw / (float)g_auto_align_window_count;

        const float delta_x   = fabsf(sample_target_x - window_avg_x);
        const float delta_y   = fabsf(sample_target_y - window_avg_y);
        const float delta_yaw = fabsf(sample_target_yaw - window_avg_yaw);
        const float delta_pos = sqrtf(delta_x * delta_x + delta_y * delta_y);

        // 如果位置或朝向偏差过大则舍弃该样本
        if (delta_pos > kAutoAlignOutlierThresholdM || delta_yaw > kAutoAlignOutlierThresholdDeg)
        {
            return false; // 舍弃离群值，继续等待下一个样本
        }
    }

    // 样本有效，加入窗口
    g_auto_align_window_x += sample_target_x;
    g_auto_align_window_y += sample_target_y;
    g_auto_align_window_yaw += sample_target_yaw;
    g_auto_align_window_count++;

    // 如果窗口未满，继续等待更多样本
    if (g_auto_align_window_count < kAutoAlignWindowCapacity)
    {
        return false;
    }

    // 窗口满，计算平均值并应用
    const float inv_count = 1.0f / (float)g_auto_align_window_count;
    *target_x             = g_auto_align_window_x * inv_count;
    *target_y             = g_auto_align_window_y * inv_count;
    *target_yaw           = g_auto_align_window_yaw * inv_count;
    *chassis_control_mode = POS_Control;

    return true;
}

// 自动对齐遥控模式下的总逻辑流程，后续可以对应到按键进行更改
bool VisionAutoAlign_RunMode(float*              target_x,
                             float*              target_y,
                             float*              target_yaw,
                             Control_Mode*       chassis_control_mode,
                             uint8_t*            auto_mode)
{
    if (!target_x || !target_y || !target_yaw || !chassis_control_mode || !auto_mode)
    {
        return false;
    }

    else
    {
        return VisionAutoAlign_Apply(
                target_x, target_y, target_yaw, chassis_control_mode, auto_mode);
    }
}
