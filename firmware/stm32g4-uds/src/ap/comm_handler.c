#include "comm_handler.h"
#include "ap.h"
#include "diag_db.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 간단한 UART 출력 함수
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uartPrintf(HW_UART_CH_DEBUG, buf);
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

// 모든 채널로 출력하는 함수
void all_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 물리적 UART로 출력
    uartPrintf(HW_UART_CH_DEBUG, buf);    
    uartPrintf(HW_UART_CH_EXT, buf);

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
        all_printf("[RX] ID: 0x%lX | Length: %d | Data: ", can_id, length);
        for (int i = 0; i < length; i++) {
            all_printf("%02X ", data[i]);
        }
        all_printf("\r\n");
    }
    
    // UDS_PATH 모드에서는 활성화된 데이터 타입들 처리
    if (mode == UDS_MODE_UDS_PATH) {
        // DIAG_DB를 사용하여 등록된 응답 CAN ID 확인
        data_type_t data_type;
        if (diag_db_is_response_id(can_id, &data_type)) {
            steering_data_handler(can_id, data, length);
        }
    }
}
// 완성된 멀티프레임 데이터에서 모든 해당 데이터 파싱 (범용)
void parse_multiple_data(uint8_t *complete_data, uint8_t data_length, uint32_t can_id)
{
    // 해당 CAN ID와 매칭되는 모든 데이터 타입들을 찾아서 처리
    bool any_data_extracted = false;
    
    for (int i = 0; i < DATA_TYPE_MAX; i++) {
        data_config_t* cfg = &g_vehicle_settings.data_configs[i];
        
        // 활성화되고 응답 CAN ID가 매칭되는 데이터 타입 확인
        if (!cfg->enabled || diag_db_get_response_id(cfg->request_id) != can_id) {
            continue;
        }
        
        // 데이터 추출 시도
        float value = 0.0f;
        if (diag_db_extract_data_value((data_type_t)i, complete_data, data_length, &value)) {
            any_data_extracted = true;
            
            // 스티어링 데이터는 UART로 스티어링 컬럼에 전송
            if (i == DATA_TYPE_STEERING) {
                char steering_msg[32];
                snprintf(steering_msg, sizeof(steering_msg), "%.1f\r\n", value);
                uartPrintf(HW_UART_CH_EXT, steering_msg);
            }
            
            // UDS_PATH 모드에서만 상세 확인 메시지
            if (get_current_mode() == UDS_MODE_UDS_PATH) {
                all_printf("[%s] %s: %.2f %s\r\n", 
                          diag_db_get_vehicle_name(),
                          diag_db_get_data_type_name((data_type_t)i),
                          value,
                          cfg->unit);
            }
        }
    }
    
    if (!any_data_extracted) {
        all_printf("[PARSE] No valid data extracted from CAN ID 0x%lX\r\n", can_id);
    }
}

// 단일 데이터 파싱 (기존 호환성)
void parse_data(uint8_t *complete_data, uint8_t data_length, data_type_t data_type)
{
    // DIAG_DB를 사용하여 데이터 추출
    float value = 0.0f;
    
    if (!diag_db_extract_data_value(data_type, complete_data, data_length, &value)) {
        all_printf("[%s] Failed to extract data\r\n", diag_db_get_data_type_name(data_type));
        return;
    }
    
    // 스티어링 데이터는 UART로 스티어링 컬럼에 전송
    if (data_type == DATA_TYPE_STEERING) {
        char steering_msg[32];
        snprintf(steering_msg, sizeof(steering_msg), "%.1f\r\n", value);
        uartPrintf(HW_UART_CH_EXT, steering_msg);
    }
    
    // UDS_PATH 모드에서만 상세 확인 메시지
    if (get_current_mode() == UDS_MODE_UDS_PATH) {
        // 데이터 값 표시 (정수.소수 형태)
        int int_part = (int)value;
        int dec_part = (int)((value - int_part) * 10);
        if (dec_part < 0) dec_part = -dec_part;
        
        all_printf("[%s] %s: %.2f %s\r\n", 
                  diag_db_get_vehicle_name(),
                  diag_db_get_data_type_name(data_type),
                  value,
                  g_vehicle_settings.data_configs[data_type].unit);
    }
}

// 기존 호환성을 위한 함수
void parse_steering_data(uint8_t *complete_data, uint8_t data_length)
{
    parse_data(complete_data, data_length, DATA_TYPE_STEERING);
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
    
    
    if (length < 1) {
        return;
    }
    
    uint8_t pci = data[0]; // Protocol Control Information
    uint8_t frame_type = (pci >> 4) & 0x0F;
    
    switch (frame_type) {
        case 0x1: // First Frame (Multi-frame 시작)
        {
            if (length < 3) break;
            
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
                    
                    // First frame의 데이터 부분 저장 (인덱스 5부터: Service ID + DID2 Byte 뒤 실제 Data부터 저장)
                    for (int i = 5; i < length && collected_length < sizeof(multi_frame_buffer); i++) {
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
                if (collected_length >= (total_data_length - 3)) {  // -3: PCI(1) + Length(1) + Service까지 제외
                    // 모든 데이터 수집 완료 - 해당 CAN ID의 모든 데이터 파싱 실행
                    parse_multiple_data(multi_frame_buffer, collected_length, can_id);
                    
                    // 수집 완료, 플래그 리셋
                    is_collecting = false;
                    collected_length = 0;
                }
            } else {
                // 시퀀스 오류 - 수집 중단
                all_printf("[MULTI-FRAME] Sequence error: expected %d, got %d\r\n", expected_sequence, sequence_number);
                is_collecting = false;
                collected_length = 0;
            }
            break;
        }
        
        default:
            break;
    }
}


