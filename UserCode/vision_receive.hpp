/**
 * @file lower_receive.h
 * @author Mburn
 * @date 2026-03-10
 * @brief 视觉串口收发与解析模块
 */
#ifndef __LOWER_RECEIVE_H__
#define __LOWER_RECEIVE_H__

#include "config.hpp"

inline constexpr uint8_t LR_RX_BUFFER_SIZE      = AppConfig::Vision::RxBufferSize;
inline constexpr int     LR_DATA_MAX_NUM        = AppConfig::Vision::DataMaxNum;
inline constexpr bool    LR_CAMERA_REVERSED     = AppConfig::Vision::CameraReversed;
inline constexpr uint8_t LR_VISION_FRAME_HEADER = AppConfig::Vision::FrameHeader;
inline constexpr uint8_t LR_VISION_FRAME_SIZE   = AppConfig::Vision::FrameSize;

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
    int   has_rpy;
} LR_DataPacket;

typedef struct
{
    float x;
    float y;
    float z;
} LR_Vector3;

extern LR_DataPacket lr_detect_buffer[LR_DATA_MAX_NUM];
extern int           lr_detect_count;
extern int           lr_detect_write_idx;

extern LR_DataPacket lr_apriltag_buffer[LR_DATA_MAX_NUM];
extern int           lr_apriltag_count;
extern int           lr_apriltag_write_idx;

extern volatile uint32_t lr_detect_update_seq;
extern volatile uint32_t lr_apriltag_update_seq;

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

extern volatile uint32_t vision_rx_irq_cnt;
extern volatile uint32_t vision_rx_byte_cnt;
extern volatile uint32_t vision_fail_cnt;
extern volatile uint32_t vision_err_cnt;
extern volatile uint32_t vision_last_err_code;

void CammeraReceive_Init(void);
bool CammeraReceive_OnRxCplt(UART_HandleTypeDef* huart);
bool CammeraReceive_OnError(UART_HandleTypeDef* huart);

void LR_Parse_And_Store(uint8_t byte);
bool LR_Send_Frame(float x, float y, float yaw, uint8_t status);
void LR_Set_RequestCameraId(uint8_t camera_id);
void LR_Clear_Data_Buffer(void);

typedef void (*LR_DataTypeCallback)(int type);
void LR_Set_DataType_Callback(LR_DataTypeCallback cb);

void       LR_Set_Camera_To_Body_Offset(float x, float y, float z);
void       LR_Set_Arm_To_Body_Offset(float x, float y, float z);
LR_Vector3 LR_Get_Camera_To_Body_Offset(void);
LR_Vector3 LR_Get_Arm_To_Body_Offset(void);

void LR_Convert_CameraPoint_To_Body(
        float cam_x, float cam_y, float cam_z, float* body_x, float* body_y, float* body_z);
void LR_Convert_CameraPoint_To_Arm(
        float cam_x, float cam_y, float cam_z, float* arm_x, float* arm_y, float* arm_z);
void LR_Convert_Camerayaw_To_Body(float cam_yaw_deg, float* body_yaw_deg);
void LR_Convert_Camerayaw_To_Arm(float cam_yaw_deg, float* arm_yaw_deg);

void LR_Compute_Target(
        float x, float y, float z, float yaw, float* target_x, float* target_y, float* target_yaw);

LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt);
LR_DataPacket LR_Convert_Packet_CameraToArm(const LR_DataPacket* cam_pkt);

#ifdef __cplusplus
}
#endif

#endif
