#ifndef COMM_HANDLER_H_
#define COMM_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "diag_db.h"

// 프로토콜 상수 정의
#define PROTOCOL_STX    0x02    // Start of Transmission
#define PROTOCOL_ETX    0x03    // End of Transmission
#define PROTOCOL_MAX_DATA_SIZE  10  // 최대 데이터 크기

// data_type_t는 diag_db.h에서 정의됨
// 새로운 데이터 타입들:
// DATA_TYPE_STEERING_MDPS, DATA_TYPE_SPEED_MDPS, DATA_TYPE_APS_ECU, DATA_TYPE_BPS_ECU,
// DATA_TYPE_APS_VCU, DATA_TYPE_BPS_ABS, DATA_TYPE_GEAR_VCU, DATA_TYPE_TURN_SIG_MFSW,
// DATA_TYPE_DOOR_OPEN, DATA_TYPE_SEAT_BELT, DATA_TYPE_RADAR, DATA_TYPE_BATT_TEMP_BMS

// 프로토콜 데이터 ID 정의
typedef enum {
    PROTOCOL_ID_VEHICLE_SPEED   = 0x01,  // 차량 속도 (2바이트, km/h)
    PROTOCOL_ID_APS            = 0x02,  // 엑셀 페달 (2바이트, 0.1% 단위)
    PROTOCOL_ID_BPS            = 0x03,  // 브레이크 페달 (2바이트, 0.1% 단위)
    PROTOCOL_ID_STEERING_ANGLE = 0x04,  // 스티어링 앵글 (2바이트, 도 단위)
    PROTOCOL_ID_EPS_ERR        = 0x05,  // EPS 에러 (1바이트, 상태)
    PROTOCOL_ID_GEAR           = 0x06,  // 기어 상태 (1바이트, P/R/N/D)
    PROTOCOL_ID_TURN_SIGNAL    = 0x07,  // 방향지시등 (1바이트, 비트마스크)
    PROTOCOL_ID_DOOR_OPEN      = 0x08,  // 차량 문 열림 (1바이트, 비트마스크)
    PROTOCOL_ID_SEAT_BELT      = 0x09,  // 안전벨트 (1바이트, 비트마스크)
    PROTOCOL_ID_RADAR          = 0x0A,  // 장애물 감지 (2바이트, cm 단위)
    PROTOCOL_ID_BMS            = 0x0B,  // BMS 데이터 (2바이트, 0.1도 단위)
} protocol_data_id_t;

// 프로토콜 패킷 구조체
typedef struct {
    uint8_t stx;        // STX (0x02)
    uint8_t data_id;    // Data ID
    uint8_t length;     // Data Length
    uint8_t data[PROTOCOL_MAX_DATA_SIZE];  // Data (1~10 bytes)
    uint8_t checksum;   // Checksum
    uint8_t etx;        // ETX (0x03)
} protocol_packet_t;

// CAN ID to Protocol ID 매핑 구조체
typedef struct {
    uint8_t data_type;
    protocol_data_id_t data_id; // Protocol Data ID (0x01~0x0A)
} can_to_protocol_map_t;

// 통신 출력 함수들
void uart_printf(const char *fmt, ...);
void cdc_printf(const char *fmt, ...);
void all_printf(const char *fmt, ...);

// 프로토콜 관련 함수들
uint8_t calculate_checksum(uint8_t data_id, uint8_t length, uint8_t *data);
void send_protocol_packet(protocol_data_id_t data_id, float value);

// UDS 디버그 및 데이터 처리 함수들
void uds_debug_output(uint32_t can_id, uint8_t *data, uint8_t length);
void steering_data_handler(uint32_t can_id, uint8_t *data, uint8_t length);
void parse_multiple_data(uint8_t *complete_data, uint8_t data_length, uint32_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* COMM_HANDLER_H_ */ 