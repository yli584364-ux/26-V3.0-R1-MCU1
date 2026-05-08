/**
 * @file lower_receive.h
 * @author Mburn
 * @date 2026-03-10
 * @brief 视觉串口收发与解析模块头文件
 *
 * 提供视觉串口固定二进制帧收发接口与目标缓存。
 * 帧格式：`0xAA + float32(x) + float32(y) + float32(yaw) + uint8(status) + uint8(crc8)`。
 * `float` 按小端序发送，CRC8 覆盖 `x/y/yaw/status`，即帧头之后的 13 字节。
 *
 * 用法：
 * 1. 在串口接收中断中调用 `LR_Parse_And_Store(byte)` 完成自动分帧与解析。
 * 2. 可通过 `LR_Set_DataType_Callback` 注册回调，区分数据类型。
 * 3. 通过全局缓存访问解析后的数据，始终保留最新的 `LR_DATA_MAX_NUM` 条。
 */
#ifndef __LOWER_RECEIVE_H__
#define __LOWER_RECEIVE_H__

#include "config.hpp"

// ======================== 配置参数 ========================
// 串口接收缓冲区长度，仅用于诊断字符串缓存。
inline constexpr uint8_t LR_RX_BUFFER_SIZE = AppConfig::Vision::RxBufferSize;
// 可缓存的数据包最大数量，即环形缓冲区大小。
inline constexpr int LR_DATA_MAX_NUM = AppConfig::Vision::DataMaxNum;
// 摄像头是否倒置，影响坐标系转换时的符号处理。
inline constexpr bool LR_CAMERA_REVERSED = AppConfig::Vision::CameraReversed;
// 固定二进制帧帧头。
inline constexpr uint8_t LR_VISION_FRAME_HEADER = AppConfig::Vision::FrameHeader;
// 固定二进制帧总长度。
inline constexpr uint8_t LR_VISION_FRAME_SIZE = AppConfig::Vision::FrameSize;

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os2.h"
#include "gpio.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    // 1: 含 roll/pitch/yaw, 0: 仅使用 yaw。
    int has_rpy;
} LR_DataPacket;

typedef struct
{
    float x;
    float y;
    float z;
} LR_Vector3;

// ======================== 全局变量 ========================
// detect 格式环形缓存。
extern LR_DataPacket lr_detect_buffer[LR_DATA_MAX_NUM];
extern int           lr_detect_count;
extern int           lr_detect_write_idx;

// apriltag 格式环形缓存。
extern LR_DataPacket lr_apriltag_buffer[LR_DATA_MAX_NUM];
extern int           lr_apriltag_count;
extern int           lr_apriltag_write_idx;

// 数据更新序号。对应类型成功解析一次后自增。
extern volatile uint32_t lr_detect_update_seq;
extern volatile uint32_t lr_apriltag_update_seq;

// 解析诊断信息，可直接在调试器 Watch 中观察。
extern volatile uint32_t lr_diag_parse_ok_count;
extern volatile uint32_t lr_diag_parse_fail_count;
extern volatile float    lr_diag_last_x;
extern volatile float    lr_diag_last_y;
extern volatile float    lr_diag_last_z;
extern volatile float    lr_diag_last_yaw;
extern volatile uint8_t  lr_diag_last_type;
extern volatile uint8_t  lr_diag_last_fail_stage;
extern volatile uint32_t lr_diag_last_raw_len;
extern volatile char     lr_diag_last_raw_frame[LR_RX_BUFFER_SIZE];
extern volatile uint8_t  lr_diag_last_status;

// USART2 视觉串口接收最小诊断计数器。
extern volatile uint32_t vision_rx_irq_cnt;
extern volatile uint32_t vision_rx_byte_cnt;
extern volatile uint32_t vision_fail_cnt;
extern volatile uint32_t vision_err_cnt;
extern volatile uint32_t vision_last_err_code;

// ======================== 接口函数 ========================
void CammeraReceive_Init(void);
bool CammeraReceive_OnRxCplt(UART_HandleTypeDef* huart);
bool CammeraReceive_OnError(UART_HandleTypeDef* huart);

/**
 * @brief 串口接收中断中调用，按固定二进制帧协议逐字节解析。
 * @param byte 新接收到的字节
 */
void LR_Parse_And_Store(uint8_t byte);

/**
 * @brief 按固定二进制帧协议发送一帧视觉数据。
 * @param x 目标 x
 * @param y 目标 y
 * @param yaw 目标 yaw
 * @param status 8 位状态位
 * @return true 发送调用成功；false 发送失败
 */
bool LR_Send_Frame(float x, float y, float yaw, uint8_t status);

/**
 * @brief 设置 1Hz 请求线程发送的相机 ID 字节。
 */
void LR_Set_RequestCameraId(uint8_t camera_id);

/**
 * @brief 清空所有已接收并解析的数据包缓存。
 */
void LR_Clear_Data_Buffer(void);

/**
 * @brief 设置数据类型回调函数。
 * @param type 0 表示 detect
 */
typedef void (*LR_DataTypeCallback)(int type);
void LR_Set_DataType_Callback(LR_DataTypeCallback cb);

/**
 * @brief 设置相机中心相对车体中心的位置偏移。
 */
void LR_Set_Camera_To_Body_Offset(float x, float y, float z);
/**
 * @brief 设置机械臂基座相对车体中心的位置偏移。
 */
void LR_Set_Arm_To_Body_Offset(float x, float y, float z);
/**
 * @brief 读取当前相机到车体的偏移参数。
 */
LR_Vector3 LR_Get_Camera_To_Body_Offset(void);
/**
 * @brief 读取当前机械臂到车体的偏移参数。
 */
LR_Vector3 LR_Get_Arm_To_Body_Offset(void);

/**
 * @brief 将相机坐标系下的点转换为车体坐标系下的点。
 */
void LR_Convert_CameraPoint_To_Body(
        float cam_x, float cam_y, float cam_z, float* body_x, float* body_y, float* body_z);
/**
 * @brief 将相机坐标系下的点转换为机械臂坐标系下的点。
 */
void LR_Convert_CameraPoint_To_Arm(
        float cam_x, float cam_y, float cam_z, float* arm_x, float* arm_y, float* arm_z);
/**
 * @brief 将相机坐标系下的 yaw 角转换为车体坐标系下的 yaw 角。
 */
void LR_Convert_Camerayaw_To_Body(float cam_yaw_deg, float* body_yaw_deg);
/**
 * @brief 将相机坐标系下的 yaw 角转换为机械臂坐标系下的 yaw 角。
 */
void LR_Convert_Camerayaw_To_Arm(float cam_yaw_deg, float* arm_yaw_deg);

/**
 * @brief 根据视觉返回值解算底盘目标位置。
 */
void LR_Compute_Target(
        float x, float y, float z, float yaw, float* target_x, float* target_y, float* target_yaw);

/**
 * @brief 将数据包中的位置从相机基准转换为车体基准，姿态字段保持不变。
 */
LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt);
/**
 * @brief 将数据包中的位置从相机基准转换为机械臂基准，姿态字段保持不变。
 */
LR_DataPacket LR_Convert_Packet_CameraToArm(const LR_DataPacket* cam_pkt);

#ifdef __cplusplus
}
#endif

#endif
