#include "can_manager.h"
#include "cli.h"
#include "ap.h"

// UDS 관련 상수 정의
#define UDS_FUNCTIONAL_ID    0x7DF  // UDS 기능적 요청 ID
#define UDS_PHYSICAL_ID      0x7E0  // UDS 물리적 요청 ID (예시)
#define UDS_RESPONSE_ID      0x7E8  // UDS 응답 ID (예시)

// 정적 변수들
static uint32_t error_check_time = 0;
static uint32_t last_tx_err_count = 0;
static uint32_t last_rx_err_count = 0;
static bool can_initialized = false;
static bool is_tx_mode = false;

// UDS 메시지 처리를 위한 변수들
static uint32_t last_uds_message_time = 0;
static uint32_t uds_message_count = 0;

// 외부에서 사용할 UDS 디버깅 함수 선언
extern void uds_debug_output(uint32_t can_id, uint8_t *data, uint8_t length);

// CLI 함수 제거됨 - TALK 모드에서 직접 입력 처리


// CAN 매니저 초기화
bool can_manager_init(bool tx_mode)
{
    is_tx_mode = tx_mode;
    
    canInit();
    
    CanMode_t can_mode = CAN_NORMAL;  // UDS는 일반 CAN 모드 사용
    
    if (canOpen(_DEF_CAN1, can_mode, CAN_CLASSIC, CAN_500K, CAN_2M) == false)
    {
        all_printf("UDS CAN init failed\r\n");
        return false;
    }
    
    all_printf("UDS CAN initialized: 500K/2M, CAN-FD BRS\r\n");
    all_printf("UDS Mode: %s\r\n", tx_mode ? "RX/TX (TX Enabled)" : "RX Only");
    all_printf("Monitoring All CAN IDs (including OEM specific IDs)\r\n");
    all_printf("CAN Filter: Accept ALL messages for UDS debugging\r\n");
    
// CLI 명령어는 TALK 모드에서 직접 입력 방식으로 대체됨
    
    can_initialized = true;
    return true;
}

// CAN 매니저 메인 처리
void can_manager_process(void)
{
    if (!can_initialized) return;
    
    // 에러 체크
    can_error_check();
    
    // UDS 메시지 수신 처리
    uds_rx_process();
}

// UDS 메시지 수신 처리
void uds_rx_process(void)
{
    // CAN 메시지 수신 확인
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            uds_message_count++;
            last_uds_message_time = millis();
            
            // UDS 디버깅 정보 출력 (요구사항에 맞는 형식)
            uds_debug_output(rx_msg.id, rx_msg.data, rx_msg.length);
            
            // ISO-TP First Frame 자동 Flow Control 처리
            uds_auto_flow_control(&rx_msg);
            
            // LED 토글로 수신 표시
            ledToggle(HW_LED_CH_RX);
            
            // 추가 디버깅: 수신 카운터 표시
            if (uds_message_count % 10 == 1) {  // 매 10번째 메시지마다
                all_printf("[DEBUG] Total RX: %lu messages\r\n", uds_message_count);
            }
        }
    }
    
    // 주기적 상태 출력 (10초마다)
    static uint32_t last_status_time = 0;
    if (millis() - last_status_time >= 10000)
    {
        last_status_time = millis();
        // all_printf("UDS Status: %lu messages received, last: %lu ms ago\r\n", 
        //            uds_message_count, 
        //            last_uds_message_time > 0 ? (millis() - last_uds_message_time) : 0);
    }
}

// UDS 메시지 정보 상세 출력
void uds_print_message_info(uint32_t can_id, uint8_t *data, uint8_t length)
{
    if (length < 1) return;
    
    const char* msg_type = "Unknown";
    uint8_t pci = data[0];  // Protocol Control Information
    
    // UDS 메시지 타입 분류
    if (can_id == 0x7DF)
    {
        msg_type = "Functional Request";
    }
    else if (can_id >= 0x7E0 && can_id <= 0x7E7)
    {
        msg_type = "Physical Request";
    }
    else if (can_id >= 0x7E8 && can_id <= 0x7EF)
    {
        msg_type = "Response";
    }
    
    all_printf("[UDS INFO] Type: %s | ID: 0x%lX | Length: %d\r\n", msg_type, can_id, length);
    
    // ISO-TP 프레임 타입 분석
    uint8_t frame_type = (pci >> 4) & 0x0F;
    
    switch (frame_type)
    {
        case 0x0:  // Single Frame (0-7 bytes)
        {
            uint8_t sf_length = pci & 0x0F;
            if (length > 1 && sf_length > 0)
            {
                uint8_t service_id = data[1];
                const char* service_name = uds_get_service_name(service_id);
                all_printf("[ISO-TP] Single Frame | Length: %d | Service: 0x%02X (%s)\r\n", 
                          sf_length, service_id, service_name);
                
                // DID 정보 표시 (Read DID 서비스인 경우)
                if (service_id == 0x22 && length >= 4)
                {
                    uint16_t did = (data[2] << 8) | data[3];
                    all_printf("[UDS DID] Read Data Identifier: 0x%04X\r\n", did);
                }
            }
                        break;
                    }
        case 0x1:  // First Frame (8+ bytes)
        {
            uint16_t ff_length = ((pci & 0x0F) << 8) | data[1];
            if (length > 2)
            {
                uint8_t service_id = data[2];
                const char* service_name = uds_get_service_name(service_id);
                all_printf("[ISO-TP] First Frame | Total Length: %d bytes | Service: 0x%02X (%s)\r\n", 
                          ff_length, service_id, service_name);
                
                // 응답 서비스인 경우 (0x40 이상)
                if (service_id >= 0x40 && service_id <= 0x7F)
                {
                    uint8_t original_service = service_id - 0x40;
                    all_printf("[UDS RESPONSE] Positive Response to Service: 0x%02X\r\n", original_service);
                    
                    // DID 응답인 경우
                    if (original_service == 0x22 && length >= 5)
                    {
                        uint16_t did = (data[3] << 8) | data[4];
                        all_printf("[UDS DID] Response for DID: 0x%04X | Remaining: %d bytes\r\n", 
                                  did, ff_length - 3);
                    }
                }
            }
            break;
        }
        case 0x2:  // Consecutive Frame
        {
            uint8_t sequence_number = pci & 0x0F;
            all_printf("[ISO-TP] Consecutive Frame | Sequence: %d | Data: 7 bytes\r\n", sequence_number);
            break;
        }
        case 0x3:  // Flow Control Frame
        {
            uint8_t flow_status = pci & 0x0F;
            const char* flow_type = "Unknown";
            switch (flow_status)
            {
                case 0: flow_type = "Continue To Send (CTS)"; break;
                case 1: flow_type = "Wait (WT)"; break;
                case 2: flow_type = "Overflow (OVFLW)"; break;
            }
            all_printf("[ISO-TP] Flow Control | Status: %s\r\n", flow_type);
            break;
        }
        default:
            all_printf("[ISO-TP] Unknown Frame Type: 0x%X\r\n", frame_type);
                break;
            }
        }
        
// UDS 서비스 이름 반환
const char* uds_get_service_name(uint8_t service_id)
{
    switch (service_id)
    {
        case 0x10: return "Diagnostic Session Control";
        case 0x11: return "ECU Reset";
        case 0x14: return "Clear Diagnostic Information";
        case 0x19: return "Read DTC Information";
        case 0x22: return "Read Data By Identifier";
        case 0x23: return "Read Memory By Address";
        case 0x24: return "Read Scaling Data By Identifier";
        case 0x27: return "Security Access";
        case 0x28: return "Communication Control";
        case 0x2A: return "Read Data By Periodic Identifier";
        case 0x2C: return "Dynamically Define Data Identifier";
        case 0x2E: return "Write Data By Identifier";
        case 0x2F: return "Input Output Control By Identifier";
        case 0x31: return "Routine Control";
        case 0x34: return "Request Download";
        case 0x35: return "Request Upload";
        case 0x36: return "Transfer Data";
        case 0x37: return "Request Transfer Exit";
        case 0x3D: return "Write Memory By Address";
        case 0x3E: return "Tester Present";
        case 0x50: return "Diagnostic Session Control (Positive Response)";
        case 0x51: return "ECU Reset (Positive Response)";
        case 0x54: return "Clear Diagnostic Information (Positive Response)";
        case 0x59: return "Read DTC Information (Positive Response)";
        case 0x62: return "Read Data By Identifier (Positive Response)";
        case 0x67: return "Security Access (Positive Response)";
        case 0x6E: return "Write Data By Identifier (Positive Response)";
        case 0x71: return "Routine Control (Positive Response)";
        case 0x74: return "Request Download (Positive Response)";
        case 0x76: return "Transfer Data (Positive Response)";
        case 0x77: return "Request Transfer Exit (Positive Response)";
        case 0x7E: return "Tester Present (Positive Response)";
        case 0x7F: return "Negative Response";
        default: return "Unknown/Reserved";
    }
}

// CAN 에러 체크
void can_error_check(void)
{
    // 500ms마다 에러 상태 체크
    if (millis() - error_check_time >= 500)
    {
        error_check_time = millis();
        
        // 에러 상태 업데이트
        canUpdate();
        
        uint32_t can_error = canGetError(_DEF_CAN1);
        uint16_t tx_err_count = canGetTxErrCount(_DEF_CAN1);
        uint16_t rx_err_count = canGetRxErrCount(_DEF_CAN1);
        
        // 에러 카운터 증가 감지
        if (tx_err_count != last_tx_err_count || rx_err_count != last_rx_err_count)
        {
            all_printf("UDS CAN Error: TX=%d, RX=%d, Code=0x%08lX\r\n", 
                      tx_err_count, rx_err_count, can_error);
            last_tx_err_count = tx_err_count;
            last_rx_err_count = rx_err_count;
        }
        
        // Bus-Off 또는 Error Passive 상태 감지 시 복구 시도
        if (can_error & (CAN_ERR_BUS_OFF | CAN_ERR_PASSIVE))
        {
            all_printf("UDS CAN Error detected! Attempting recovery...\r\n");
                can_error_recovery();
        }
    }
}

// CAN 에러 복구
void can_error_recovery(void)
{
    all_printf("Performing UDS CAN recovery...\r\n");
    
    canRecovery(_DEF_CAN1);
    
    // 복구 후 잠시 대기
    delay(10);
    
    all_printf("UDS CAN recovery completed\r\n");
}

// ISO-TP 자동 Flow Control 처리 함수
void uds_auto_flow_control(can_msg_t *rx_msg)
{
    // TALK 모드에서만 자동 Flow Control 동작
    if (get_current_mode() != UDS_MODE_TALK) {
        return;
    }
    
    if (rx_msg->length < 1) return;
    
    uint8_t pci = rx_msg->data[0];
    uint8_t frame_type = (pci >> 4) & 0x0F;
    
    // First Frame (0x1)인지 확인
    if (frame_type == 0x1) {
        // 전체 데이터 길이 확인
        uint16_t total_length = ((pci & 0x0F) << 8) | rx_msg->data[1];
        
        all_printf("[AUTO FC] First Frame detected: Total %d bytes\r\n", total_length);
        
        // 모든 UDS 응답에 대해 Flow Control 전송 (길이가 8바이트를 초과하는 경우)
        if (rx_msg->length >= 3 && total_length > 7) {
            // Flow Control 메시지 구성
            can_msg_t flow_control_msg;
            
            // 응답 ID 계산 (일반적으로 수신 ID - 8)
            // 0x7DC(수신) -> 0x7D4(송신)
            uint32_t tx_id = rx_msg->id - 8;
            
            // CAN 메시지 초기화
            canMsgInit(&flow_control_msg, CAN_CLASSIC, CAN_STD, canGetDlc(8));
            flow_control_msg.id = tx_id;
            flow_control_msg.length = 8;
            
            // Flow Control 데이터 구성
            flow_control_msg.data[0] = 0x30;  // Flow Control (0x3) + CTS (0x0)
            flow_control_msg.data[1] = 0x00;  // Block Size (0 = 무제한)
            flow_control_msg.data[2] = 0x00;  // Separation Time (0ms)
            flow_control_msg.data[3] = 0x00;  // 패딩
            flow_control_msg.data[4] = 0x00;  // 패딩
            flow_control_msg.data[5] = 0x00;  // 패딩
            flow_control_msg.data[6] = 0x00;  // 패딩
            flow_control_msg.data[7] = 0x00;  // 패딩
            
            // Flow Control 전송
            all_printf("[AUTO FC] Sending Flow Control: 30 00 00 (CTS)\r\n");
            if (canMsgWrite(_DEF_CAN1, &flow_control_msg, 100)) {
                all_printf("[AUTO FC] Flow Control sent to ID: 0x%lX\r\n", tx_id);
            } else {
                all_printf("[AUTO FC] Flow Control send failed!\r\n");
            }
        }
    }
}

// CLI 함수 제거됨 - TALK 모드에서 직접 터미널 입력 처리로 대체
