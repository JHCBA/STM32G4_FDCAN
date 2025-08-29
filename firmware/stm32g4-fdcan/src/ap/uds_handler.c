#include "uds_handler.h"
#include "can_manager.h"
#include "comm_handler.h"
#include "can_tx.h"

// DEBUG 출력 매크로
#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) all_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// UDS 핸들러 인스턴스
static uds_handler_t uds_handler;

// UDS 응답 메시지 데이터 정의
static const uint8_t uds_first_frame[] = {0x10, 0x17, 0x62, 0x01, 0x01, 0xFF, 0xF0, 0x00};
static const uint8_t uds_consecutive_frame1[] = {0x21, 0x00, 0x8A, 0x89, 0xF1, 0x00, 0xE6, 0xFF};
static const uint8_t uds_consecutive_frame2[] = {0x22, 0x55, 0xE5, 0x00, 0x00, 0x00, 0x10, 0x10};
static const uint8_t uds_consecutive_frame3[] = {0x23, 0x25, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};

// UDS 초기화
bool uds_init(void)
{
    DEBUG_PRINT("UDS Handler initialized\r\n");
    uds_reset_state();
    return true;
}

// UDS 상태 리셋
void uds_reset_state(void)
{
    uds_handler.state = UDS_STATE_IDLE;
    uds_handler.timeout_start = 0;
    uds_handler.sequence_number = 1;
    uds_handler.active = false;
}

// UDS 메인 처리
void uds_process(void)
{
    uint32_t current_time = millis();
    
    // 상태별 처리
    switch (uds_handler.state) {
        case UDS_STATE_WAITING_FLOW_CONTROL:
            // Flow Control 대기 중 타임아웃 체크
            if (current_time - uds_handler.timeout_start >= UDS_TIMEOUT_MS) {
                DEBUG_PRINT("[UDS] Flow Control timeout!\r\n");
                uds_handler.state = UDS_STATE_ERROR;
                uds_reset_state();
            }
            break;
            
        case UDS_STATE_SENDING_CONSECUTIVE:
            // Consecutive Frame 전송
            if (uds_send_consecutive_frames()) {
                DEBUG_PRINT("[UDS] All consecutive frames sent successfully\r\n");
                uds_reset_state();
            } else {
                DEBUG_PRINT("[UDS] Failed to send consecutive frames\r\n");
                uds_handler.state = UDS_STATE_ERROR;
                uds_reset_state();
            }
            break;
            
        case UDS_STATE_ERROR:
            DEBUG_PRINT("[UDS] Error state, resetting...\r\n");
            uds_reset_state();
            break;
            
        case UDS_STATE_IDLE:
        default:
            // IDLE 상태에서는 아무것도 하지 않음
            break;
    }
}

// UDS 요청 처리
bool uds_handle_request(const can_msg_t *msg)
{
    // CAN ID 확인
    if (msg->id != UDS_REQUEST_ID) {
        return false;
    }
    
    // 메시지 길이 확인
    if (msg->length < 8) {
        return false;
    }
    
    // 특정 UDS 요청 메시지 확인: 03 22 01 01 55 55 55 55
    if (msg->data[0] == 0x03 && 
        msg->data[1] == 0x22 && 
        msg->data[2] == 0x01 && 
        msg->data[3] == 0x01) {
        
        DEBUG_PRINT("[UDS] Valid request received!\r\n");
        uds_print_message("[UDS] Request", msg);
        
        // First Frame 전송
        if (uds_send_first_frame()) {
            uds_handler.state = UDS_STATE_WAITING_FLOW_CONTROL;
            uds_handler.timeout_start = millis();
            uds_handler.active = true;
            return true;
        } else {
            DEBUG_PRINT("[UDS] Failed to send first frame\r\n");
            return false;
        }
    }
    
    return false;
}

// Flow Control 확인
bool uds_check_flow_control(const can_msg_t *msg)
{
    // UDS_STATE_WAITING_FLOW_CONTROL 상태가 아니면 무시
    if (uds_handler.state != UDS_STATE_WAITING_FLOW_CONTROL) {
        return false;
    }
    
    // CAN ID 확인
    if (msg->id != UDS_REQUEST_ID) {
        return false;
    }
    
    // 메시지 길이 확인
    if (msg->length < 8) {
        return false;
    }
    
    // Flow Control 메시지 확인: 30 08 02 55 55 55 55 55
    if (msg->data[0] == 0x30) {
        
        DEBUG_PRINT("[UDS] Valid Flow Control received!\r\n");
        uds_print_message("[UDS] Flow Control", msg);
        
        // Consecutive Frame 전송 상태로 전환
        uds_handler.state = UDS_STATE_SENDING_CONSECUTIVE;
        return true;
    }
    
    return false;
}

// First Frame 전송
bool uds_send_first_frame(void)
{
    can_msg_t tx_msg;
    
    // CAN 메시지 초기화
    canMsgInit(&tx_msg, CAN_FD_BRS, CAN_STD, CAN_DLC_8);
    tx_msg.id = UDS_RESPONSE_ID;
    
    // First Frame 데이터 복사
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = uds_first_frame[i];
    }
    
    // 메시지 전송
    if (canMsgWrite(_DEF_CAN1, &tx_msg, 10)) {
        DEBUG_PRINT("[UDS] First Frame sent successfully\r\n");
        uds_print_message("[UDS] Response FF", &tx_msg);
        ledToggle(HW_LED_CH_TX);
        return true;
    } else {
        DEBUG_PRINT("[UDS] Failed to send First Frame\r\n");
        return false;
    }
}

// Consecutive Frame들 전송
bool uds_send_consecutive_frames(void)
{
    can_msg_t tx_msg;
    bool success = true;
    
    // CAN 메시지 초기화
    canMsgInit(&tx_msg, CAN_FD_BRS, CAN_STD, CAN_DLC_8);
    tx_msg.id = UDS_RESPONSE_ID;
    
    // Consecutive Frame 1 전송
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = uds_consecutive_frame1[i];
    }
    
    if (canMsgWrite(_DEF_CAN1, &tx_msg, 10)) {
        DEBUG_PRINT("[UDS] Consecutive Frame 1 sent\r\n");
        uds_print_message("[UDS] Response CF1", &tx_msg);
        ledToggle(HW_LED_CH_TX);
    } else {
        DEBUG_PRINT("[UDS] Failed to send Consecutive Frame 1\r\n");
        success = false;
    }
    
    // 짧은 딜레이 (연속 전송을 위해)
    delay(1);
    
    // Consecutive Frame 2 전송
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = uds_consecutive_frame2[i];
    }
    
    if (canMsgWrite(_DEF_CAN1, &tx_msg, 10)) {
        DEBUG_PRINT("[UDS] Consecutive Frame 2 sent\r\n");
        uds_print_message("[UDS] Response CF2", &tx_msg);
        ledToggle(HW_LED_CH_TX);
    } else {
        DEBUG_PRINT("[UDS] Failed to send Consecutive Frame 2\r\n");
        success = false;
    }
    
    // 짧은 딜레이
    delay(1);
    
    // Consecutive Frame 3 전송
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = uds_consecutive_frame3[i];
    }
    
    if (canMsgWrite(_DEF_CAN1, &tx_msg, 10)) {
        DEBUG_PRINT("[UDS] Consecutive Frame 3 sent\r\n");
        uds_print_message("[UDS] Response CF3", &tx_msg);
        ledToggle(HW_LED_CH_TX);
    } else {
        DEBUG_PRINT("[UDS] Failed to send Consecutive Frame 3\r\n");
        success = false;
    }
    
    return success;
}

// UDS 메시지 디버그 출력
void uds_print_message(const char* prefix, const can_msg_t *msg)
{
    DEBUG_PRINT("%s ID: 0x%lX | Length: %d | Data: ", prefix, msg->id, msg->length);
    for (int i = 0; i < msg->length && i < 8; i++) {
        DEBUG_PRINT("%02X ", msg->data[i]);
    }
    DEBUG_PRINT("\r\n");
}

// UDS 활성화 상태 확인
bool uds_is_active(void)
{
    return uds_handler.active;
}
