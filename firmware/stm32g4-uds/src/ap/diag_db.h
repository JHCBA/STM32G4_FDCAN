#ifndef DIAG_DB_H_
#define DIAG_DB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include <stdbool.h>
#include <stdint.h>

// UDS 서비스 ID 정의
#define UDS_SERVICE_READ_DATA_BY_ID         0x22
#define UDS_SERVICE_READ_DATA_BY_ID_RESP    0x62

// 데이터 타입 정의
typedef enum {
    DATA_TYPE_STEERING = 0,
    DATA_TYPE_SPEED = 1,
    DATA_TYPE_ENGINE_RPM = 2,
    DATA_TYPE_BATT_TEMP_1 = 3,
    DATA_TYPE_BATT_TEMP_2 = 4,
    DATA_TYPE_MAX
} data_type_t;

// 데이터 추출 설정 구조체 (BLE로 업데이트 가능)
typedef struct {
    bool     enabled;           // 데이터 추출 활성화 여부
    uint16_t did;              // Data Identifier  
    uint32_t request_id;       // 요청 CAN ID (응답 ID = request_id + 0x08)
    uint8_t  data_offset;      // 데이터 시작 오프셋 (0-based)
    uint8_t  data_length;      // 데이터 길이 (바이트)
    float    resolution;       // 분해능 (unit/LSB)
    int16_t  offset_value;     // 오프셋 값
    bool     is_signed;        // 부호 있는 데이터 여부
    bool     big_end_msb_first;    // 빅엔디안 여부 (false = 리틀엔디안)
    uint32_t request_period_ms; // 요청 주기 (ms, 0=비활성화)
    char     unit[8];          // 단위 (예: "deg", "km/h", "rpm")
    char     name[16];         // 데이터 이름
} data_config_t;

// DID 그룹 관리 구조체 (같은 DID를 가진 데이터들을 그룹핑)
typedef struct {
    uint16_t did;                           // Data Identifier
    uint32_t request_id;                    // 요청 CAN ID
    uint32_t response_id;                   // 응답 CAN ID
    uint32_t last_request_time;             // 마지막 요청 시간
    uint32_t min_period_ms;                 // 최소 요청 주기 (그룹 내 최소값)
    data_type_t data_types[DATA_TYPE_MAX];  // 이 DID를 사용하는 데이터 타입들
    uint8_t data_type_count;                // 데이터 타입 개수
    bool is_active;                         // 그룹 활성화 여부
} did_group_t;

// 최대 DID 그룹 수
#define MAX_DID_GROUPS  8

// 비휘발성 메모리 저장용 차량 설정 구조체
typedef struct {
    uint32_t magic;                                    // 매직 넘버 (설정 유효성 확인)
    uint16_t version;                                  // 설정 버전
    char     vehicle_name[32];                         // 차량 이름
    data_config_t data_configs[DATA_TYPE_MAX];         // 각 데이터 타입별 설정
    uint32_t crc32;                                    // CRC32 체크섬
} vehicle_settings_t;

// 전역 차량 설정 변수 (런타임에서 BLE로 업데이트 가능)
extern vehicle_settings_t g_vehicle_settings;

// DID 그룹 관리 변수 (런타임에서 동적 생성)
extern did_group_t g_did_groups[MAX_DID_GROUPS];
extern uint8_t g_did_group_count;

// 기본 설정값들
#define DIAG_DB_MAGIC           0x44494147  // "DIAG"
#define DIAG_DB_VERSION         0x0001
#define DEFAULT_VEHICLE_NAME    "SONATA_DN8_HME"
#define EV_VEHICLE_NAME     "IONIC5_NE_HMC"

// DIAG_DB 초기화 및 관리 함수들
void diag_db_init(void);
void diag_db_load_defaults(void);
bool diag_db_save_to_nvram(void);
bool diag_db_load_from_nvram(void);
void diag_db_print_current_settings(void);

// 응답 ID 계산 함수 (inline)
static inline uint32_t diag_db_get_response_id(uint32_t request_id) {
    return request_id + 0x08;
}

// 차량 설정 업데이트 함수들 (BLE 인터페이스용)
bool diag_db_set_vehicle_name(const char* name);
bool diag_db_set_data_config(data_type_t type, const data_config_t* config);
bool diag_db_enable_data_type(data_type_t type, bool enable);
bool diag_db_set_did(data_type_t type, uint16_t did);
bool diag_db_set_request_id(data_type_t type, uint32_t request_id);
bool diag_db_set_data_params(data_type_t type, uint8_t offset, uint8_t length, float resolution, int16_t offset_value);
bool diag_db_set_data_format(data_type_t type, bool is_signed, bool big_end_msb_first);
bool diag_db_set_data_info(data_type_t type, const char* name, const char* unit);
bool diag_db_set_request_period(data_type_t type, uint32_t period_ms);

// DID 그룹 관리 함수들
void diag_db_rebuild_did_groups(void);
void diag_db_process_periodic_requests(void);
bool diag_db_find_did_group(uint16_t did, uint32_t request_id, uint32_t response_id, did_group_t** group);
void diag_db_send_group_request(did_group_t* group);
void diag_db_update_group_timing(did_group_t* group);

// 데이터 추출 함수들
bool diag_db_is_response_id(uint32_t can_id, data_type_t* data_type);
bool diag_db_extract_data_value(data_type_t type, uint8_t *complete_data, uint8_t data_length, float *value);
void diag_db_send_data_request(data_type_t type);
const char* diag_db_get_data_type_name(data_type_t type);

// 편의 함수들 (기존 호환성)
bool diag_db_is_steering_response(uint32_t can_id);
bool diag_db_extract_steering_angle(uint8_t *complete_data, uint8_t data_length, float *steering_angle);
void diag_db_send_steering_request(void);
const char* diag_db_get_vehicle_name(void);

// BLE 명령 처리 함수 (문자열 기반 인터페이스)
typedef enum {
    BLE_CMD_OK = 0,
    BLE_CMD_ERROR_INVALID_PARAM,
    BLE_CMD_ERROR_INVALID_TYPE,
    BLE_CMD_ERROR_INVALID_FORMAT,
    BLE_CMD_ERROR_OUT_OF_RANGE
} ble_cmd_result_t;

ble_cmd_result_t diag_db_ble_set_vehicle_name(const char* cmd);
ble_cmd_result_t diag_db_ble_enable_data_type(const char* cmd);
ble_cmd_result_t diag_db_ble_set_did(const char* cmd);
ble_cmd_result_t diag_db_ble_set_request_id(const char* cmd);
ble_cmd_result_t diag_db_ble_set_data_params(const char* cmd);
ble_cmd_result_t diag_db_ble_set_data_format(const char* cmd);
ble_cmd_result_t diag_db_ble_set_data_info(const char* cmd);
ble_cmd_result_t diag_db_ble_set_request_period(const char* cmd);
const char* diag_db_ble_get_error_string(ble_cmd_result_t result);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_DB_H_ */
