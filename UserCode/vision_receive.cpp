#include "vision_receive.hpp"

#include <cmath>
#include <string.h>

namespace ProjectVisionConfig = AppConfig::Vision;

static uint8_t             g_lr_uart2_rx_byte = 0;
static LR_DataTypeCallback g_datatype_cb      = NULL;

static osThreadId_t         vision_parse_thread_handle   = NULL;
static osThreadId_t         vision_request_thread_handle = NULL;
static const osThreadAttr_t vision_parse_thread_attr     = {
    .name       = "vision_parse",
    .stack_size = 256 * 8,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
static const osThreadAttr_t vision_request_thread_attr = {
    .name       = "vision_req",
    .stack_size = 128 * 8,
    .priority   = (osPriority_t)osPriorityNormal,
};

static constexpr uint16_t LR_RX_RING_SIZE = ProjectVisionConfig::RxRingSize;
static uint8_t           g_rx_ring[LR_RX_RING_SIZE] = { 0 };
static volatile uint16_t g_rx_ring_head             = 0U;
static volatile uint16_t g_rx_ring_tail             = 0U;
static volatile uint32_t g_rx_ring_overflow_cnt     = 0U;

static volatile uint8_t g_request_camera_id = 0x01U;

volatile uint32_t vision_rx_irq_cnt    = 0;
volatile uint32_t vision_rx_byte_cnt   = 0;
volatile uint32_t vision_fail_cnt      = 0;
volatile uint32_t vision_err_cnt       = 0;
volatile uint32_t vision_last_err_code = 0;

volatile uint32_t lr_diag_parse_ok_count                    = 0;
volatile uint32_t lr_diag_parse_fail_count                  = 0;
volatile float    lr_diag_last_x                            = 0.0f;
volatile float    lr_diag_last_y                            = 0.0f;
volatile float    lr_diag_last_z                            = 0.0f;
volatile float    lr_diag_last_yaw                          = 0.0f;
volatile uint8_t  lr_diag_last_type                         = 0;
volatile uint8_t  lr_diag_last_fail_stage                   = 0;
volatile uint32_t lr_diag_last_raw_len                      = 0;
volatile char     lr_diag_last_raw_frame[LR_RX_BUFFER_SIZE] = { 0 };
volatile uint8_t  lr_diag_last_status                       = 0;

static LR_Vector3 g_camera_to_body_offset = { ProjectVisionConfig::CameraToBodyOffsetX,
                                              ProjectVisionConfig::CameraToBodyOffsetY,
                                              ProjectVisionConfig::CameraToBodyOffsetZ };
static LR_Vector3 g_arm_to_body_offset = { ProjectVisionConfig::ArmToBodyOffsetX,
                                           ProjectVisionConfig::ArmToBodyOffsetY,
                                           ProjectVisionConfig::ArmToBodyOffsetZ };
static int        yaw_camera_to_body_deg = ProjectVisionConfig::YawCameraToBodyDeg;
static int        yaw_arm_to_body_deg    = ProjectVisionConfig::YawArmToBodyDeg;

constexpr float PI = 3.14159265358979323846f;

LR_DataPacket     lr_detect_buffer[LR_DATA_MAX_NUM];
int               lr_detect_count      = 0;
int               lr_detect_write_idx  = 0;
volatile uint32_t lr_detect_update_seq = 0;

LR_DataPacket     lr_apriltag_buffer[LR_DATA_MAX_NUM];
int               lr_apriltag_count      = 0;
int               lr_apriltag_write_idx  = 0;
volatile uint32_t lr_apriltag_update_seq = 0;

static uint8_t g_frame_buf[LR_VISION_FRAME_SIZE] = { 0 };
static uint8_t g_frame_pos                       = 0U;

static bool LR_RingPushByte(uint8_t byte)
{
    const uint16_t head      = g_rx_ring_head;
    const uint16_t next_head = (uint16_t)((head + 1U) % LR_RX_RING_SIZE);
    if (next_head == g_rx_ring_tail)
    {
        return false;
    }

    g_rx_ring[head] = byte;
    g_rx_ring_head  = next_head;
    return true;
}

static bool LR_RingPopByte(uint8_t* out)
{
    if (!out)
    {
        return false;
    }

    const uint16_t tail = g_rx_ring_tail;
    if (tail == g_rx_ring_head)
    {
        return false;
    }

    *out           = g_rx_ring[tail];
    g_rx_ring_tail = (uint16_t)((tail + 1U) % LR_RX_RING_SIZE);
    return true;
}

static void VisionParseTask(void* argument)
{
    (void)argument;
    for (;;)
    {
        uint8_t byte = 0U;
        if (LR_RingPopByte(&byte))
        {
            LR_Parse_And_Store(byte);
            continue;
        }

        osDelay(1);
    }
}

static void VisionRequestTask(void* argument)
{
    (void)argument;
    for (;;)
    {
        const uint8_t req = g_request_camera_id;
        if (HAL_UART_Transmit(&huart2, (uint8_t*)&req, 1U, 10U) != HAL_OK)
        {
            vision_fail_cnt++;
        }
        osDelay(1000);
    }
}

static uint8_t LR_CRC8_Payload(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

static void LR_PushDetectPacket(const LR_DataPacket* pkt)
{
    if (!pkt)
    {
        return;
    }

    lr_detect_buffer[lr_detect_write_idx] = *pkt;
    lr_detect_write_idx                   = (lr_detect_write_idx + 1) % LR_DATA_MAX_NUM;
    if (lr_detect_count < LR_DATA_MAX_NUM)
    {
        lr_detect_count++;
    }
    lr_detect_update_seq++;

    if (g_datatype_cb)
    {
        g_datatype_cb(0);
    }
}

static void RotateXY(float in_x, float in_y, float yaw_deg, float* out_x, float* out_y)
{
    const float yaw_rad = yaw_deg * PI / 180.0f;
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);

    if (out_x)
    {
        *out_x = in_x * cos_yaw - in_y * sin_yaw;
    }
    if (out_y)
    {
        *out_y = in_x * sin_yaw + in_y * cos_yaw;
    }
}

void CammeraReceive_Init(void)
{
    g_rx_ring_head         = 0U;
    g_rx_ring_tail         = 0U;
    g_rx_ring_overflow_cnt = 0U;

    if (!vision_parse_thread_handle)
    {
        vision_parse_thread_handle = osThreadNew(VisionParseTask, NULL, &vision_parse_thread_attr);
    }

    if (!vision_request_thread_handle)
    {
        vision_request_thread_handle =
                osThreadNew(VisionRequestTask, NULL, &vision_request_thread_attr);
    }

    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1U) != HAL_OK)
    {
        vision_fail_cnt++;
    }
}

bool CammeraReceive_OnRxCplt(UART_HandleTypeDef* huart)
{
    if (huart->Instance != USART2)
    {
        return false;
    }

    const uint8_t rx_byte = g_lr_uart2_rx_byte;
    vision_rx_irq_cnt++;
    vision_rx_byte_cnt++;

    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1U) != HAL_OK)
    {
        vision_fail_cnt++;
        return true;
    }

    if (!LR_RingPushByte(rx_byte))
    {
        g_rx_ring_overflow_cnt++;
        vision_fail_cnt++;
    }

    return true;
}

bool CammeraReceive_OnError(UART_HandleTypeDef* huart)
{
    if (huart->Instance != USART2)
    {
        return false;
    }

    vision_err_cnt++;
    vision_last_err_code = huart->ErrorCode;

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (HAL_UART_Receive_IT(&huart2, &g_lr_uart2_rx_byte, 1U) != HAL_OK)
    {
        vision_fail_cnt++;
    }

    return true;
}

void LR_Set_RequestCameraId(uint8_t camera_id)
{
    g_request_camera_id = camera_id;
}

static void LR_EncodeFloatLE(float value, uint8_t* out4)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    out4[0] = (uint8_t)(bits & 0xFFU);
    out4[1] = (uint8_t)((bits >> 8) & 0xFFU);
    out4[2] = (uint8_t)((bits >> 16) & 0xFFU);
    out4[3] = (uint8_t)((bits >> 24) & 0xFFU);
}

static float LR_DecodeFloatLE(const uint8_t* in4)
{
    const uint32_t bits = ((uint32_t)in4[0]) | ((uint32_t)in4[1] << 8) |
                          ((uint32_t)in4[2] << 16) | ((uint32_t)in4[3] << 24);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void LR_Set_DataType_Callback(LR_DataTypeCallback cb)
{
    g_datatype_cb = cb;
}

void LR_Set_Camera_To_Body_Offset(float x, float y, float z)
{
    g_camera_to_body_offset.x = x;
    g_camera_to_body_offset.y = y;
    g_camera_to_body_offset.z = z;
}

void LR_Set_Arm_To_Body_Offset(float x, float y, float z)
{
    g_arm_to_body_offset.x = x;
    g_arm_to_body_offset.y = y;
    g_arm_to_body_offset.z = z;
}

LR_Vector3 LR_Get_Camera_To_Body_Offset(void)
{
    return g_camera_to_body_offset;
}

LR_Vector3 LR_Get_Arm_To_Body_Offset(void)
{
    return g_arm_to_body_offset;
}

void LR_Convert_Camerayaw_To_Body(float cam_yaw_deg, float* body_yaw_deg)
{
    if (!body_yaw_deg)
    {
        return;
    }

    const float camera_yaw = LR_CAMERA_REVERSED ? -cam_yaw_deg : cam_yaw_deg;
    *body_yaw_deg          = camera_yaw + (float)yaw_camera_to_body_deg;
}

void LR_Convert_Camerayaw_To_Arm(float cam_yaw_deg, float* arm_yaw_deg)
{
    if (!arm_yaw_deg)
    {
        return;
    }

    float body_yaw = 0.0f;
    LR_Convert_Camerayaw_To_Body(cam_yaw_deg, &body_yaw);
    *arm_yaw_deg = body_yaw - (float)yaw_arm_to_body_deg;
}

void LR_Convert_CameraPoint_To_Body(
        float cam_x, float cam_y, float cam_z, float* body_x, float* body_y, float* body_z)
{
    const float camera_y = LR_CAMERA_REVERSED ? -cam_y : cam_y;

    float rotated_x = cam_x;
    float rotated_y = camera_y;
    RotateXY(cam_x, camera_y, (float)yaw_camera_to_body_deg, &rotated_x, &rotated_y);

    if (body_x)
    {
        *body_x = rotated_x + g_camera_to_body_offset.x;
    }
    if (body_y)
    {
        *body_y = rotated_y + g_camera_to_body_offset.y;
    }
    if (body_z)
    {
        *body_z = cam_z + g_camera_to_body_offset.z;
    }
}

void LR_Convert_CameraPoint_To_Arm(
        float cam_x, float cam_y, float cam_z, float* arm_x, float* arm_y, float* arm_z)
{
    float body_x = 0.0f;
    float body_y = 0.0f;
    float body_z = 0.0f;
    LR_Convert_CameraPoint_To_Body(cam_x, cam_y, cam_z, &body_x, &body_y, &body_z);

    const float rel_x = body_x - g_arm_to_body_offset.x;
    const float rel_y = body_y - g_arm_to_body_offset.y;
    RotateXY(rel_x, rel_y, -(float)yaw_arm_to_body_deg, arm_x, arm_y);

    if (arm_z)
    {
        *arm_z = body_z - g_arm_to_body_offset.z;
    }
}

void LR_Compute_Target(
        float x, float y, float z, float yaw, float* target_x, float* target_y, float* target_yaw)
{
    float body_x     = 0.0f;
    float body_y     = 0.0f;
    float camera_yaw = 0.0f;

    LR_Convert_CameraPoint_To_Body(x, y, z, &body_x, &body_y, nullptr);
    LR_Convert_Camerayaw_To_Body(yaw, &camera_yaw);

    const float target_in_body_x =
            body_x - g_arm_to_body_offset.x * cosf(PI * camera_yaw / 180.0f) +
            g_arm_to_body_offset.y * sinf(PI * camera_yaw / 180.0f);
    const float target_in_body_y =
            body_y - g_arm_to_body_offset.x * sinf(PI * camera_yaw / 180.0f) -
            g_arm_to_body_offset.y * cosf(PI * camera_yaw / 180.0f);

    *target_x   = target_in_body_x;
    *target_y   = target_in_body_y;
    *target_yaw = camera_yaw;
}

LR_DataPacket LR_Convert_Packet_CameraToBody(const LR_DataPacket* cam_pkt)
{
    LR_DataPacket out = { 0 };
    if (!cam_pkt)
    {
        return out;
    }

    out = *cam_pkt;
    LR_Convert_CameraPoint_To_Body(cam_pkt->x, cam_pkt->y, cam_pkt->z, &out.x, &out.y, &out.z);
    LR_Convert_Camerayaw_To_Body(cam_pkt->yaw, &out.yaw);
    if (cam_pkt->has_rpy)
    {
        LR_Convert_Camerayaw_To_Body(cam_pkt->roll, &out.roll);
    }
    return out;
}

LR_DataPacket LR_Convert_Packet_CameraToArm(const LR_DataPacket* cam_pkt)
{
    LR_DataPacket out = { 0 };
    if (!cam_pkt)
    {
        return out;
    }

    out = *cam_pkt;
    LR_Convert_CameraPoint_To_Arm(cam_pkt->x, cam_pkt->y, cam_pkt->z, &out.x, &out.y, &out.z);
    LR_Convert_Camerayaw_To_Arm(cam_pkt->yaw, &out.yaw);
    if (cam_pkt->has_rpy)
    {
        LR_Convert_Camerayaw_To_Arm(cam_pkt->roll, &out.roll);
    }
    return out;
}

void LR_Clear_Data_Buffer(void)
{
    lr_detect_count      = 0;
    lr_detect_write_idx  = 0;
    lr_detect_update_seq = 0;
    memset(lr_detect_buffer, 0, sizeof(lr_detect_buffer));

    lr_apriltag_count      = 0;
    lr_apriltag_write_idx  = 0;
    lr_apriltag_update_seq = 0;
    memset(lr_apriltag_buffer, 0, sizeof(lr_apriltag_buffer));

    g_frame_pos = 0U;
    memset(g_frame_buf, 0, sizeof(g_frame_buf));

    g_rx_ring_head         = 0U;
    g_rx_ring_tail         = 0U;
    g_rx_ring_overflow_cnt = 0U;
    memset(g_rx_ring, 0, sizeof(g_rx_ring));
}

bool LR_Send_Frame(float x, float y, float yaw, uint8_t status)
{
    uint8_t frame[LR_VISION_FRAME_SIZE] = { 0 };

    frame[0] = LR_VISION_FRAME_HEADER;
    LR_EncodeFloatLE(x, &frame[1]);
    LR_EncodeFloatLE(y, &frame[5]);
    LR_EncodeFloatLE(yaw, &frame[9]);
    frame[13] = status;
    frame[14] = LR_CRC8_Payload(&frame[1], 13U);

    return HAL_UART_Transmit(&huart2, frame, LR_VISION_FRAME_SIZE, 10U) == HAL_OK;
}

void LR_Parse_And_Store(uint8_t byte)
{
    if (g_frame_pos == 0U)
    {
        if (byte == LR_VISION_FRAME_HEADER)
        {
            g_frame_buf[0] = byte;
            g_frame_pos    = 1U;
        }
        return;
    }

    g_frame_buf[g_frame_pos++] = byte;
    if (g_frame_pos < LR_VISION_FRAME_SIZE)
    {
        return;
    }

    const uint8_t expected_crc = LR_CRC8_Payload(&g_frame_buf[1], 13U);
    if (expected_crc != g_frame_buf[14])
    {
        lr_diag_parse_fail_count++;
        lr_diag_last_fail_stage = 4U;
        vision_fail_cnt++;

        if (byte == LR_VISION_FRAME_HEADER)
        {
            g_frame_buf[0] = byte;
            g_frame_pos    = 1U;
        }
        else
        {
            g_frame_pos = 0U;
        }
        return;
    }

    LR_DataPacket pkt = { 0 };
    pkt.x             = LR_DecodeFloatLE(&g_frame_buf[1]);
    pkt.y             = LR_DecodeFloatLE(&g_frame_buf[5]);
    pkt.z             = 0.0f;
    pkt.yaw           = LR_DecodeFloatLE(&g_frame_buf[9]);
    pkt.roll          = pkt.yaw;
    pkt.pitch         = 0.0f;
    pkt.has_rpy       = 0;

    lr_diag_parse_ok_count++;
    lr_diag_last_x       = pkt.x;
    lr_diag_last_y       = pkt.y;
    lr_diag_last_z       = pkt.z;
    lr_diag_last_yaw     = pkt.yaw;
    lr_diag_last_status  = g_frame_buf[13];
    lr_diag_last_type    = 0U;
    lr_diag_last_raw_len = LR_VISION_FRAME_SIZE;

    LR_PushDetectPacket(&pkt);
    g_frame_pos = 0U;
}
