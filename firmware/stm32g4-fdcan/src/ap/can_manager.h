#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "comm_handler.h"
#include "can_db.h"

// DEBUG 플래그 - 디버깅 메시지 출력 제어
#ifndef DEBUG_CAN_PROTOCOL
#define DEBUG_CAN_PROTOCOL  0  // 0: 디버깅 비활성화, 1: 디버깅 활성화
#endif

// CAN 모드 정의
typedef enum {
    CAN_ALL_LISTEN = 0,      // 모든 CAN ID 수신 (기본 주기: 1000ms)
    CAN_FILTER_LISTEN = 1,   // 필터링된 CAN ID만 수신
    CAN_RELEASE = 2,         // 필터링된 CAN ID를 프로토콜로 변환하여 송신
    CAN_DEBUG_MODE = 3,      // 디버깅 모드 (CAN과 무관하게 UART만 출력)
    CAN_SCAN_MODE = 4        // SCAN 모드 (CAN ID만 확인)
} can_mode_t;

// 모든 CAN ID 수신을 위한 구조체 (메모리 절약)
typedef struct {
    uint32_t id;             // CAN ID
    can_msg_t msg;           // CAN 메시지
    bool has_data;           // 데이터 유무
    uint16_t rx_count;       // 수신 카운터 (16비트로 축소)
    uint32_t last_output_time; // 마지막 출력 시간
    uint32_t last_rx_time;   // 마지막 수신 시간
} all_can_tracker_t;

// SCAN 모드를 위한 구조체 (메모리 최소화)
typedef struct {
    uint32_t id;             // CAN ID
    uint8_t dlc;             // Data Length Code
    bool has_data;           // 데이터 유무
    uint16_t rx_count;       // 수신 카운터
    uint32_t last_rx_time;   // 마지막 수신 시간
} scan_can_tracker_t;

// 프로토콜 정의
#define PROTOCOL_STX        0x02  // Start of Transmission
#define PROTOCOL_ETX        0x03  // End of Transmission
#define PROTOCOL_MAX_DATA   10    // 최대 데이터 길이

// 프로토콜 데이터 ID 정의
typedef enum {
    PROTOCOL_ID_VEHICLE_SPEED   = 0x01,  // 차량 속도
    PROTOCOL_ID_APS            = 0x02,  // 엑셀 페달
    PROTOCOL_ID_BPS            = 0x03,  // 브레이크 페달
    PROTOCOL_ID_STEERING_ANGLE = 0x04,  // 스티어링 앵글
    PROTOCOL_ID_EPS_ERR        = 0x05,  // EPS 에러
    PROTOCOL_ID_GEAR           = 0x06,  // 기어 상태
    PROTOCOL_ID_TURN_SIGNAL    = 0x07,  // 방향지시등
    PROTOCOL_ID_DOOR_OPEN      = 0x08,  // 차량 문 열림
    PROTOCOL_ID_SEAT_BELT      = 0x09,  // 안전벨트
    PROTOCOL_ID_RADAR          = 0x0A,  // 장애물 감지
} protocol_data_id_t;

// 프로토콜 프레임 구조체
typedef struct {
    uint8_t stx;        // 0x02
    uint8_t data_id;    // Data ID
    uint8_t length;     // Data Length (1~10)
    uint8_t data[PROTOCOL_MAX_DATA]; // Data
    uint8_t checksum;   // Checksum
    uint8_t etx;        // 0x03
} protocol_frame_t;

// CAN ID to Protocol ID 매핑 구조체
typedef struct {
    uint32_t can_id;           // CAN ID (예: 0x384, 0x125 등)
    protocol_data_id_t data_id; // Protocol Data ID (0x01~0x0A)
    uint32_t period_ms;        // 전송 주기 (ms)
} can_to_protocol_map_t;

// CAN 설정
extern bool can_test_tx_mode;
extern can_mode_t current_can_mode;

// CAN 초기화 및 관리 함수들
bool can_manager_init(bool tx_mode);
void can_manager_process(void);

// CAN TX 관련 함수들
bool can_tx_init(void);
void can_tx_process(void);

// CAN RX 관련 함수들  
bool can_rx_init(void);
void can_rx_process(void);

// CAN 에러 처리 함수들
void can_error_check(void);
void can_error_recovery(void);

// 프로토콜 관련 함수들
bool protocol_init(void);
bool protocol_map_can_to_data_id(uint32_t can_id, protocol_data_id_t *data_id);
bool protocol_convert_can_data(uint32_t can_id, const uint8_t *can_data, uint8_t can_length, 
                              uint8_t *protocol_data, uint8_t *protocol_length);
uint8_t protocol_calculate_checksum(uint8_t data_id, uint8_t length, const uint8_t *data);
bool protocol_send_frame(protocol_data_id_t data_id, const uint8_t *data, uint8_t length);

// 새로운 CAN 모드 관련 함수들
bool can_all_listen_init(void);
void can_all_listen_process(void);
bool can_filter_listen_init(void);
void can_filter_listen_process(void);
bool can_debug_mode_init(void);
void can_debug_mode_process(void);
bool can_scan_mode_init(void);
void can_scan_mode_process(void);
void can_output_raw_message(uint32_t can_id, const can_msg_t *msg);
void can_mode_switch(void);
void can_print_mode_info(void);
void can_set_all_listen_timeout(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MANAGER_H_ */ 