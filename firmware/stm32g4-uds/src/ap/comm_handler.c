#include "comm_handler.h"
#include "ap.h"
#include "diag_db.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// DEBUG 출력 매크로
#define DEBUG_CAN_PROTOCOL  0  // 1: 활성화, 0: 비활성화

#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) cdc_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// CAN ID to Protocol ID 매핑 테이블
// 필터링된 CAN ID들을 순서대로 프로토콜 Data ID로 매핑
static const can_to_protocol_map_t can_protocol_map[] = {
    {DATA_TYPE_SPEED,       PROTOCOL_ID_VEHICLE_SPEED},   // Vehicle Speed
    {0xFF,                  PROTOCOL_ID_BPS},             // APS  
    {0xFF,                  PROTOCOL_ID_APS},             // BPS
    {DATA_TYPE_STEERING,    PROTOCOL_ID_STEERING_ANGLE},  // Steering Angle
    {0xFF,                  PROTOCOL_ID_EPS_ERR},         // EPS Error
    {0xFF,                  PROTOCOL_ID_GEAR},            // Gear Status
    {0xFF,                  PROTOCOL_ID_TURN_SIGNAL},     // Turn Signal
    {0xFF,                  PROTOCOL_ID_DOOR_OPEN},       // Door Open
    {0xFF,                  PROTOCOL_ID_SEAT_BELT},       // Seat Belt
    {0xFF,                  PROTOCOL_ID_RADAR},           // Radar
    {DATA_TYPE_BATT_TEMP_1, PROTOCOL_ID_BMS},             // BMS
};

// 간단한 UART 출력 함수
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uartPrintf(HW_UART_CH_EXT, buf);
}

// USB CDC 출력 함수
void cdc_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (cdcIsConnect()) {
        cdcWrite((uint8_t*)buf, strlen(buf));
    }
}

// 모든 채널로 출력하는 함수 (HW_UART_CH_EXT 제외 - 프로토콜 전용)
void all_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 물리적 UART로 출력 (EXT 제외 - 프로토콜 전용으로 사용)
    uartPrintf(HW_UART_CH_DEBUG, buf);    
    // uartPrintf(HW_UART_CH_EXT, buf);  // 제거: 프로토콜 전용 채널

    // USB CDC로도 출력
    if (cdcIsConnect()) {
        cdcWrite((uint8_t*)buf, strlen(buf));
    }
}

// UDS 디버그 출력 함수
void uds_debug_output(uint32_t can_id, uint8_t *data, uint8_t length)
{
    // 현재 모드 확인
    uds_mode_t mode = get_current_mode();
    
    if (mode == UDS_MODE_TALK) {
        // TALK 모드: 간단한 RX 메시지 출력
        cdc_printf("[RX] ID: 0x%lX | Length: %d | Data: ", can_id, length);
        for (int i = 0; i < length; i++) {
            cdc_printf("%02X ", data[i]);
        }
        cdc_printf("\r\n");
    }
    
    // UDS_PATH 모드에서는 활성화된 데이터 타입들 처리
    if (mode == UDS_MODE_UDS_PATH) {
        // DIAG_DB를 사용하여 등록된 응답 CAN ID 확인
        data_type_t data_type;
        
        // 디버그: 수신된 CAN ID와 예상 응답 ID 확인
        static uint32_t last_debug_time = 0;
        if (millis() - last_debug_time > 1000) {  // 1초마다만 출력
            last_debug_time = millis();
        }
        
        if (diag_db_is_response_id(can_id, &data_type)) {
            steering_data_handler(can_id, data, length);
        } else {
            DEBUG_PRINT("[DEBUG] No matching response ID found\r\n");
        }
    }
}
// 완성된 멀티프레임 데이터에서 모든 해당 데이터 파싱 (범용)
void parse_multiple_data(uint8_t *complete_data, uint8_t data_length, uint32_t can_id)
{
    // 해당 CAN ID와 매칭되는 모든 데이터 타입들을 찾아서 처리
    bool any_data_extracted = false;
    char group_output[256] = {0};
    char vehicle_name[32] = {0};
    int group_count = 0;
    
    // 차량 이름 복사
    strncpy(vehicle_name, diag_db_get_vehicle_name(), sizeof(vehicle_name) - 1);
    

    
    for (int i = 0; i < DATA_TYPE_MAX; i++) {
        data_config_t* cfg = &g_vehicle_settings.data_configs[i];
        
        // 활성화되고 응답 CAN ID가 매칭되는 데이터 타입 확인
        if (!cfg->enabled || diag_db_get_response_id(cfg->request_id) != can_id) {
            continue;
        }
        
        // UDS_PATH 모드에서만 체크 로그 출력
        if (get_current_mode() == UDS_MODE_UDS_PATH) {
            DEBUG_PRINT("[COMM] Checking data type %d (%s): enabled=%d, response_id=0x%03X, can_id=0x%03X\r\n", 
                       i, cfg->name, cfg->enabled, diag_db_get_response_id(cfg->request_id), can_id);
        }
        
        // 데이터 추출 시도
        float value = 0.0f;
        bool extract_result = diag_db_extract_data_value((data_type_t)i, complete_data, data_length, &value);
        if (extract_result) {
            DEBUG_PRINT("[COMM] Data type %d (%s) extraction SUCCESS, value=%.1f\r\n", i, cfg->name, value);
            any_data_extracted = true;
            
            // 프로토콜 매핑 테이블에서 해당 데이터 타입의 프로토콜 ID 찾기
            for (int j = 0; j < sizeof(can_protocol_map) / sizeof(can_protocol_map[0]); j++) {
                if (can_protocol_map[j].data_type == i) {
                    // 매핑된 프로토콜 ID로 패킷 전송
                    send_protocol_packet(can_protocol_map[j].data_id, value);
                    break;
                }
            }
            
            // Speed 데이터도 출력
            if (i == DATA_TYPE_SPEED) {
                // Speed 값도 확인용으로 출력 (디버그용)
                DEBUG_PRINT("[COMM] Speed extracted: %.1f km/h\r\n", value);
            }
            
            DEBUG_PRINT("[COMM] About to prepare group output for %s\r\n", cfg->name);
            
            // UDS_PATH 모드에서만 그룹화된 출력 준비
            if (get_current_mode() == UDS_MODE_UDS_PATH) {
                DEBUG_PRINT("[COMM] UDS_PATH mode confirmed, group_count=%d\r\n", group_count);
                if (group_count == 0) {                    
                    const char* safe_vehicle_name = "UNKNOWN";
                    if (vehicle_name) {
                        DEBUG_PRINT("[COMM] vehicle_name is not NULL\r\n");
                        safe_vehicle_name = vehicle_name;
                    } else {
                        DEBUG_PRINT("[COMM] vehicle_name is NULL\r\n");
                    }

                    DEBUG_PRINT("[COMM] Getting data type name for type %d\r\n", i);
                    const char* safe_data_type_name = diag_db_get_data_type_name((data_type_t)i);
                    DEBUG_PRINT("[COMM] Got data type name: %s\r\n", safe_data_type_name ? "NOT_NULL" : "NULL");
                    
                    const char* safe_unit = "";
                    if (cfg->unit) {
                        DEBUG_PRINT("[COMM] cfg->unit is not NULL\r\n");
                        safe_unit = cfg->unit;
                    } else {
                        DEBUG_PRINT("[COMM] cfg->unit is NULL\r\n");
                    }
                    DEBUG_PRINT("[COMM] value received: %.6f\r\n", value);
                    
                    // Float 문제 해결을 위해 integer로 변환
                    int value_int = (int)(value * 10);  // 95.7 -> 957
                    int value_whole = value_int / 10;   // 957 -> 95
                    int value_decimal = value_int % 10; // 957 -> 7
                    if (value_decimal < 0) value_decimal = -value_decimal;
                    
                    DEBUG_PRINT("[COMM] Converted to int: whole=%d, decimal=%d\r\n", value_whole, value_decimal);
                    
                    if (safe_data_type_name) {
                        DEBUG_PRINT("[COMM] Using data_type_name path\r\n");
                        int result = snprintf(group_output, sizeof(group_output), "[%s] %s: %d.%d %s", 
                                safe_vehicle_name ? safe_vehicle_name : "NULL",
                                safe_data_type_name,
                                value_whole,
                                value_decimal,
                                safe_unit ? safe_unit : "");
                        DEBUG_PRINT("[COMM] snprintf returned: %d\r\n", result);
                        DEBUG_PRINT("[COMM] First group data prepared: %s\r\n", group_output);
                    } else {
                        DEBUG_PRINT("[COMM] ERROR: data_type_name is NULL, using fallback!\r\n");
                        int result = snprintf(group_output, sizeof(group_output), "[%s] DataType%d: %d.%d %s", 
                                safe_vehicle_name ? safe_vehicle_name : "NULL", i, value_whole, value_decimal, safe_unit ? safe_unit : "");
                        DEBUG_PRINT("[COMM] fallback snprintf returned: %d\r\n", result);
                    }
                    
                    DEBUG_PRINT("[COMM] snprintf completed successfully\r\n");
                } else {
                    // 추가 데이터들은 같은 줄에 추가
                    DEBUG_PRINT("[COMM] Preparing additional group data\r\n");
                    
                    // 추가 데이터도 integer로 변환
                    int add_value_int = (int)(value * 10);
                    int add_value_whole = add_value_int / 10;
                    int add_value_decimal = add_value_int % 10;
                    if (add_value_decimal < 0) add_value_decimal = -add_value_decimal;
                    
                    char temp[64];
                    snprintf(temp, sizeof(temp), ", %s: %d.%d %s", 
                            diag_db_get_data_type_name((data_type_t)i),
                            add_value_whole,
                            add_value_decimal,
                            cfg->unit);
                    strncat(group_output, temp, sizeof(group_output) - strlen(group_output) - 1);
                    DEBUG_PRINT("[COMM] Additional group data prepared: %s\r\n", temp);
                }
                group_count++;
                DEBUG_PRINT("[COMM] Group count now: %d\r\n", group_count);
            } else {
                DEBUG_PRINT("[COMM] Not UDS_PATH mode, skipping group output\r\n");
            }
            
            DEBUG_PRINT("[COMM] Group output preparation completed for %s\r\n", cfg->name);
        } else {
            DEBUG_PRINT("[COMM] Data type %d (%s) extraction FAILED (returned false)\r\n", i, cfg->name);
        }
        
        DEBUG_PRINT("[COMM] === Completed processing data type %d (%s) ===\r\n", i, cfg->name);
    }
    
    DEBUG_PRINT("[COMM] === Loop completed, processed %d data types ===\r\n", DATA_TYPE_MAX);
    
    // 그룹화된 출력 표시
    if (any_data_extracted && get_current_mode() == UDS_MODE_UDS_PATH && group_count > 0) {
        cdc_printf("%s\r\n", group_output);
    }
    
    if (!any_data_extracted) {
        cdc_printf("[PARSE] No valid data extracted from CAN ID 0x%lX\r\n", can_id);
    }
}
// 프로토콜 관련 함수들

// 체크섬 계산 함수 (ID부터 Data까지의 Sum 연산 하위 1바이트)
uint8_t calculate_checksum(uint8_t data_id, uint8_t length, uint8_t *data)
{
    uint32_t sum = data_id + length;
    
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    
    return (uint8_t)(sum & 0xFF);  // 하위 1바이트만 반환
}

// 프로토콜 패킷 생성 및 UART 전송 함수
void send_protocol_packet(protocol_data_id_t data_id, float value)
{
    protocol_packet_t packet;
    uint8_t uart_buffer[16];  // 최대 패킷 크기
    uint8_t packet_size = 0;
    
    // 패킷 구성
    packet.stx = PROTOCOL_STX;
    packet.data_id = (uint8_t)data_id;
    
    // 데이터 타입에 따른 데이터 변환
    if (data_id == PROTOCOL_ID_STEERING_ANGLE) {
        // 스티어링: float을 2바이트 signed int로 변환 (0.1도 단위)
        int16_t steering_int = (int16_t)(value * 10);
        packet.length = 2;
        packet.data[0] = (uint8_t)(steering_int >> 8);     // High byte
        packet.data[1] = (uint8_t)(steering_int & 0xFF);   // Low byte
    }
    else if (data_id == PROTOCOL_ID_VEHICLE_SPEED) {
        // 속도: float을 2바이트 unsigned int로 변환 (0.1km/h 단위)
        uint16_t speed_int = (uint16_t)(value * 10);
        packet.length = 2;
        packet.data[0] = (uint8_t)(speed_int >> 8);        // High byte
        packet.data[1] = (uint8_t)(speed_int & 0xFF);      // Low byte
    }
    else {
        // 기타 데이터: 4바이트 float로 전송
        packet.length = 4;
        uint32_t *float_ptr = (uint32_t*)&value;
        packet.data[0] = (uint8_t)((*float_ptr) >> 24);
        packet.data[1] = (uint8_t)((*float_ptr) >> 16);
        packet.data[2] = (uint8_t)((*float_ptr) >> 8);
        packet.data[3] = (uint8_t)((*float_ptr) & 0xFF);
    }
    
    // 체크섬 계산
    packet.checksum = calculate_checksum(packet.data_id, packet.length, packet.data);
    packet.etx = PROTOCOL_ETX;
    
    // UART 버퍼에 패킷 데이터 복사
    uart_buffer[packet_size++] = packet.stx;
    uart_buffer[packet_size++] = packet.data_id;
    uart_buffer[packet_size++] = packet.length;
    
    for (int i = 0; i < packet.length; i++) {
        uart_buffer[packet_size++] = packet.data[i];
    }
    
    uart_buffer[packet_size++] = packet.checksum;
    uart_buffer[packet_size++] = packet.etx;
    
    // UART로 패킷 전송
    uint32_t bytes_sent = uartWrite(HW_UART_CH_EXT, uart_buffer, packet_size);
    
    // 디버그 출력 (CDC) - 전송된 전체 패킷 표시
    cdc_printf("[PROTOCOL TX] ID:0x%02X Len:%d Data:", packet.data_id, packet.length);
    for (int i = 0; i < packet.length; i++) {
        cdc_printf(" %02X", packet.data[i]);
    }
    cdc_printf(" Checksum:0x%02X\r\n", packet.checksum);
    
    // 실제 전송된 바이트 표시
    cdc_printf("[UART RAW TX] Sent %lu bytes: ", bytes_sent);
    for (int i = 0; i < packet_size; i++) {
        cdc_printf("%02X ", uart_buffer[i]);
    }
    cdc_printf("\r\n");
}

// 스티어링 데이터 처리 함수 (개선됨: 완전한 데이터 수집 후 파싱)
void steering_data_handler(uint32_t can_id, uint8_t *data, uint8_t length)
{
    // UDS Read Data By Identifier (0x22) 서비스 응답 확인
    // ID: 0x7DC, DID: 0x0101에 대한 multi-frame 응답 처리
    
    static uint8_t multi_frame_buffer[64]; // 전체 데이터 수집용 버퍼
    static uint8_t expected_sequence = 1;  // 다음에 올 시퀀스 번호
    static bool is_collecting = false;     // multi-frame 수집 중인지 확인
    static uint8_t total_data_length = 0;  // 전체 데이터 길이
    static uint8_t collected_length = 0;   // 현재까지 수집된 데이터 길이
    
    // 디버그: 수신된 데이터 상세 정보
    // cdc_printf("[HANDLER] CAN ID: 0x%lX, Length: %d, Data: ", can_id, length);
    // for (int i = 0; i < length; i++) {
    //     cdc_printf("%02X ", data[i]);
    // }
    // cdc_printf("\r\n");
    
    if (length < 1) {
        DEBUG_PRINT("[HANDLER] Invalid length\r\n");
        return;
    }
    
    uint8_t pci = data[0]; // Protocol Control Information
    uint8_t frame_type = (pci >> 4) & 0x0F;    
    
    switch (frame_type) {
        case 0x0: // Single Frame (완전한 데이터가 한 프레임에 포함)
        {
            uint8_t sf_length = pci & 0x0F;
            DEBUG_PRINT("[HANDLER] Single Frame detected, length: %d\r\n", sf_length);
            
            if (sf_length > 0 && (sf_length + 1) <= length) {
                // 단일 프레임 데이터를 바로 파싱
                // data[0] = PCI, data[1]부터 실제 UDS 데이터
                uint8_t *uds_data = &data[1];
                uint8_t uds_length = sf_length;
                
                DEBUG_PRINT("[HANDLER] Parsing single frame data...\r\n");
                parse_multiple_data(uds_data, uds_length, can_id);
            } else {
                DEBUG_PRINT("[HANDLER] Invalid single frame length\r\n");
            }
            break;
        }
        
        case 0x1: // First Frame (Multi-frame 시작)
        {
            if (length < 3) {
                DEBUG_PRINT("[HANDLER] First frame too short\r\n");
                break;
            }
            
            // 전체 데이터 길이 추출
            total_data_length = ((pci & 0x0F) << 8) | data[1];
            
            // 서비스 ID가 0x62 (Read Data By Identifier 응답)인지 확인
            if (data[2] == UDS_SERVICE_READ_DATA_BY_ID_RESP && length >= 5) {
                // DID 확인 - 활성화된 데이터 타입들 중에서 매칭 확인
                uint16_t did = (data[3] << 8) | data[4];
                
                // 현재 CAN ID에 대응하는 데이터 타입 찾기
                data_type_t current_data_type;
                bool found_data_type = false;
                for (int dt = 0; dt < DATA_TYPE_MAX; dt++) {
                    data_config_t* cfg = &g_vehicle_settings.data_configs[dt];
                    if (cfg->enabled &&
                        diag_db_get_response_id(cfg->request_id) == can_id &&
                        cfg->did == did) {
                        current_data_type = (data_type_t)dt;
                        found_data_type = true;
                        break;
                    }
                }
                
                if (found_data_type) {
                    // Multi-frame 수집 시작
                    is_collecting = true;
                    expected_sequence = 1;
                    collected_length = 0;
                    
                    // 버퍼 초기화
                    memset(multi_frame_buffer, 0, sizeof(multi_frame_buffer));
                    
                    // First frame의 전체 UDS 데이터 저장 (인덱스 2부터: Service ID + DID + 실제 Data)
                    for (int i = 2; i < length && collected_length < sizeof(multi_frame_buffer); i++) {
                        multi_frame_buffer[collected_length++] = data[i];
                    }
                }
            }
            break;
        }
        
        case 0x2: // Consecutive Frame
        {
            if (!is_collecting) break;
            
            uint8_t sequence_number = pci & 0x0F;
            
            if (sequence_number == expected_sequence) {
                // 올바른 시퀀스 번호인 경우 데이터 저장
                for (int i = 1; i < length && collected_length < sizeof(multi_frame_buffer); i++) {
                    multi_frame_buffer[collected_length++] = data[i];
                }
                
                expected_sequence++;
                if (expected_sequence > 15) expected_sequence = 0; // 시퀀스 번호는 0-15 순환
                
                // 예상된 데이터 길이에 도달했는지 확인
                if (collected_length >= total_data_length) {  // total_data_length는 순수 UDS 데이터 길이
                    //cdc_printf("[HANDLER] Multi-frame complete, collected %d bytes\r\n", collected_length);
                    // 모든 데이터 수집 완료 - 해당 CAN ID의 모든 데이터 파싱 실행
                    parse_multiple_data(multi_frame_buffer, collected_length, can_id);
                    
                    // 수집 완료, 플래그 리셋
                    is_collecting = false;
                    collected_length = 0;
                } else {
                }
            } else {
                // 시퀀스 오류 - 수집 중단
                DEBUG_PRINT("[MULTI-FRAME] Sequence error: expected %d, got %d\r\n", expected_sequence, sequence_number);
                DEBUG_PRINT("[MULTI-FRAME] Possible causes: CAN frame lost, timing issue, or buffer overflow\r\n");
                DEBUG_PRINT("[MULTI-FRAME] Resetting collection state\r\n");
                is_collecting = false;
                collected_length = 0;
            }
            break;
        }
        
        default:
            DEBUG_PRINT("[HANDLER] Unknown frame type: 0x%X\r\n", frame_type);
            break;
    }
}


