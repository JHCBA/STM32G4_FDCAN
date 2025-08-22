#include "comm_handler.h"
#include "ap.h"
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
    
    if (mode == UDS_MODE_NORMAL) {
        // NORMAL 모드: 기존 상세 UDS 메시지 출력
        uds_normal_output(can_id, data, length);
    }
    
    // 두 모드 모두에서 스티어링 데이터 처리 (STEERING 모드에서는 이 부분만 동작)
    steering_data_handler(can_id, data, length);
}

// NORMAL 모드에서의 상세 UDS 출력
void uds_normal_output(uint32_t can_id, uint8_t *data, uint8_t length)
{
    static uint16_t current_did = 0x0000;
    static bool in_multiframe = false;
    
    if (length < 1) return;
    
    uint8_t pci = data[0];
    uint8_t frame_type = (pci >> 4) & 0x0F;
    
    // 프레임 타입별 처리
    switch (frame_type)
    {
        case 0x0:  // Single Frame
        {
            if (length > 1)
            {
                uint8_t service_id = data[1];
                
                // 기본 UDS 메시지 출력
                all_printf("[UDS] ID: 0x%lX | Length: %d | Data: ", can_id, length);
                for (int i = 0; i < length; i++) {
                    all_printf("%02X ", data[i]);
                }
                all_printf("\r\n");
                
                // 서비스 상세 정보
                const char* service_name = uds_get_service_name(service_id);
                all_printf("Service: 0x%02X (%s)\r\n", service_id, service_name);
                
                // DID 표시 (Read DID 서비스인 경우)
                if (service_id == 0x22 && length >= 4)
                {
                    uint16_t did = (data[2] << 8) | data[3];
                    current_did = did;
                    all_printf("Data Identifier: 0x%04X\r\n", did);
                }
                
                in_multiframe = false;
            }
            break;
        }
        case 0x1:  // First Frame
        {
            if (length > 2)
            {
                uint8_t service_id = data[2];
                
                // 응답 서비스인 경우 DID 추출
                if (service_id >= 0x40 && service_id <= 0x7F && length >= 5)
                {
                    uint16_t did = (data[3] << 8) | data[4];
                    current_did = did;
                }
                
                // 간소화된 멀티프레임 표시
                all_printf("[UDS] ID: 0x%lX [0x%04X] ", can_id, current_did);
                for (int i = 0; i < length; i++) {
                    all_printf("%02X ", data[i]);
                }
                all_printf("(MULTI-CHECK)\r\n");
                
                in_multiframe = true;
            }
            break;
        }
        case 0x2:  // Consecutive Frame
        case 0x3:  // Flow Control
        default:
        {
            // 간소화된 형식으로 출력
            all_printf("[UDS] ID: 0x%lX [0x%04X] ", can_id, current_did);
            for (int i = 0; i < length; i++) {
                all_printf("%02X ", data[i]);
            }
            all_printf("\r\n");
            break;
        }
    }
}

// 스티어링 데이터 처리 함수
void steering_data_handler(uint32_t can_id, uint8_t *data, uint8_t length)
{
    // UDS Read Data By Identifier (0x22) 서비스 응답 확인
    // ID: 0x7DC, DID: 0x0101에 대한 multi-frame 응답 처리
    
    static uint8_t multi_frame_buffer[64]; // 전체 데이터 수집용 버퍼
    static uint8_t expected_sequence = 1;  // 다음에 올 시퀀스 번호
    static bool is_collecting = false;     // multi-frame 수집 중인지 확인
    static uint8_t total_data_length = 0;  // 전체 데이터 길이
    static uint8_t collected_length = 0;   // 현재까지 수집된 데이터 길이
    
    // 0x7DC (응답 ID)에서 오는 메시지만 처리
    if (can_id != 0x7DC) {
        return;
    }
    
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
            if (data[2] == 0x62 && length >= 5) {
                // DID 확인 (0x0101)
                uint16_t did = (data[3] << 8) | data[4];
                if (did == 0x0101) {
                    // Multi-frame 수집 시작
                    is_collecting = true;
                    expected_sequence = 1;
                    collected_length = 0;
                    
                    // First frame의 데이터 부분 저장 (인덱스 5부터)
                    for (int i = 5; i < length && collected_length < sizeof(multi_frame_buffer); i++) {
                        multi_frame_buffer[collected_length++] = data[i];
                    }
                    
                    // NORMAL 모드에서만 UDS 출력
                    if (get_current_mode() == UDS_MODE_NORMAL) {
                        all_printf("[UDS] ID: 0x%lX [0x0101] %02X %02X %02X %02X %02X %02X %02X %02X (MULTI-CHECK)\r\n",
                                  can_id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
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
                
                // NORMAL 모드에서만 UDS 출력
                if (get_current_mode() == UDS_MODE_NORMAL) {
                    all_printf("[UDS] ID: 0x%lX [0x0101] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                              can_id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                }
                
                // 스티어링 데이터 저장용 static 변수
                static uint8_t steering_byte7 = 0;
                static uint8_t steering_byte8 = 0;
                static bool steering_valid = false;
                
                // Sequence 1: 스티어링 각도 추출 (21 프레임)
                if (sequence_number == 1 && length >= 7) {
                    steering_byte7 = data[5]; // BYTE 7
                    steering_byte8 = data[6]; // BYTE 8
                    steering_valid = true;
                }
                
                // Sequence 2: 차량 속도 추출 및 전체 데이터 출력 (22 프레임)
                if (sequence_number == 2 && length >= 8 && steering_valid) {
                    uint8_t speed_raw = data[7]; // BYTE 12 (22 프레임의 8번째 바이트)
                    
                    // 스티어링 각도 계산
                    uint16_t steering_raw = (steering_byte7 << 8) | steering_byte8;
                    int16_t steering_signed = (int16_t)steering_raw;
                    float steering_angle = steering_signed * 0.1f;
                    
                    // 차량 속도 계산
                    float vehicle_speed = speed_raw * 2.0f;
                    
                    // UART로 스티어링 컬럼에 전송 (스티어링 각도만)
                    char steering_msg[32];
                    snprintf(steering_msg, sizeof(steering_msg), "%.1f\r\n", steering_angle);
                    uartPrintf(HW_UART_CH_EXT, steering_msg);
                    
                    // STEERING 모드에서만 상세 확인 메시지
                    if (get_current_mode() == UDS_MODE_STEERING) {
                        // 스티어링 각도 표시
                        int steer_int = (int)steering_angle;
                        int steer_dec = (int)((steering_angle - steer_int) * 10);
                        if (steer_dec < 0) steer_dec = -steer_dec;
                        
                        // 차량 속도 표시
                        int speed_int = (int)vehicle_speed;
                        
                        all_printf("STEERING: %02X %02X -> %d.%d Degree | SPEED: %02X -> %d km/h\r\n", 
                                  steering_byte7, steering_byte8, steer_int, steer_dec, 
                                  speed_raw, speed_int);
                    }
                    
                    // 데이터 처리 완료, 플래그 리셋
                    steering_valid = false;
                }
            }
            break;
        }
        
        default:
            break;
    }
} 