#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "comm_handler.h"

// DEBUG 플래그 - 디버깅 메시지 출력 제어
#ifndef DEBUG_CAN_PROTOCOL
#define DEBUG_CAN_PROTOCOL  0  // 0: 디버깅 비활성화, 1: 디버깅 활성화
#endif

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
    uint32_t can_id;           // CAN ID
    protocol_data_id_t data_id; // Protocol Data ID
    uint32_t period_ms;        // 전송 주기 (ms)
} can_to_protocol_map_t;

// CAN 설정
extern bool can_test_tx_mode;

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

#ifdef __cplusplus
}
#endif

#endif /* CAN_MANAGER_H_ */ 