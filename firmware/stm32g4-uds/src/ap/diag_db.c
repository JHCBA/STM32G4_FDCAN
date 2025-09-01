#include "diag_db.h"
#include "ap.h" 
#include "comm_handler.h"
#include <string.h>
#include <stdio.h>

// DEBUG 출력 매크로
#define DEBUG_CAN_PROTOCOL  0  // 1: 활성화, 0: 비활성화

#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) all_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

#define DATA_START_IDX  3
// 전역 차량 설정 변수 (BLE로 업데이트 가능)
vehicle_settings_t g_vehicle_settings;

// DID 그룹 관리 변수
did_group_t g_did_groups[MAX_DID_GROUPS];
uint8_t g_did_group_count = 0;

// CRC32 계산 함수 (간단한 구현)
static uint32_t calculate_crc32(const uint8_t* data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// 기본 설정 로드
void diag_db_load_defaults(void)
{
    memset(&g_vehicle_settings, 0, sizeof(vehicle_settings_t));
    
    // 기본 설정값
    g_vehicle_settings.magic = DIAG_DB_MAGIC;
    g_vehicle_settings.version = DIAG_DB_VERSION;
    strncpy(g_vehicle_settings.vehicle_name, DEFAULT_VEHICLE_NAME, sizeof(g_vehicle_settings.vehicle_name) - 1);
    
    // 스티어링 데이터 기본 설정 (SONATA_DN8_HME)
    data_config_t* steering = &g_vehicle_settings.data_configs[DATA_TYPE_STEERING];
    steering->enabled = true;
    steering->did = 0x0101;
    steering->request_id = 0x7D4;       // 응답 ID는 0x7DC (자동 계산)
    steering->data_offset = 7 + DATA_START_IDX;          // 7,8번째 바이트
    steering->data_length = 2;
    steering->resolution = 0.1f;
    steering->offset_value = 0;
    steering->is_signed = true;
    steering->big_end_msb_first = true;     // MSB first
    steering->request_period_ms = 1000;  // 100ms 주기
    strncpy(steering->unit, "deg", sizeof(steering->unit) - 1);
    strncpy(steering->name, "Steering", sizeof(steering->name) - 1);
    
    // 속도 데이터 기본 설정
    data_config_t* speed = &g_vehicle_settings.data_configs[DATA_TYPE_SPEED];
    speed->enabled = true;              // Steering과 함께 활성화
    speed->did = 0x0101;                // 스티어링과 같은 DID 사용
    speed->request_id = 0x7D4;          // 응답 ID는 0x7DC (자동 계산)
    speed->data_offset = 12 + DATA_START_IDX;          // 12번째 바이트
    speed->data_length = 1;
    speed->resolution = 2.0f;
    speed->offset_value = 0;
    speed->is_signed = false;
    speed->big_end_msb_first = false;
    speed->request_period_ms = 1000;     // Steering과 같은 주기로 변경
    strncpy(speed->unit, "km/h", sizeof(speed->unit) - 1);
    strncpy(speed->name, "Speed", sizeof(speed->name) - 1);
    
    // 엔진 RPM 기본 설정 (예시)
    data_config_t* rpm = &g_vehicle_settings.data_configs[DATA_TYPE_ENGINE_RPM];
    rpm->enabled = false;
    rpm->did = 0x0103;
    rpm->request_id = 0x7D4;            // 응답 ID는 0x7DC (자동 계산)
    rpm->data_offset = 9;
    rpm->data_length = 2;
    rpm->resolution = 0.25f;
    rpm->offset_value = 0;
    rpm->is_signed = false;
    rpm->big_end_msb_first = true;
    rpm->request_period_ms = 1000;       // 500ms 주기
    strncpy(rpm->unit, "rpm", sizeof(rpm->unit) - 1);
    strncpy(rpm->name, "EngineRPM", sizeof(rpm->name) - 1);
    
    // 배터리 온도 1
    data_config_t* batt_temp_1 = &g_vehicle_settings.data_configs[DATA_TYPE_BATT_TEMP_1];
    batt_temp_1->enabled = true;
    batt_temp_1->did = 0x0101;
    batt_temp_1->request_id = 0x7E4;            // 응답 ID는 0x7EC (자동 계산)
    batt_temp_1->data_offset = 16 + DATA_START_IDX;     // 16번째 바이트
    batt_temp_1->data_length = 1;
    batt_temp_1->resolution = 1.0f;
    batt_temp_1->offset_value = 0;
    batt_temp_1->is_signed = true;
    batt_temp_1->big_end_msb_first = true;
    batt_temp_1->request_period_ms = 1000;      // Steering과 동일한 주기
    strncpy(batt_temp_1->unit, "Celsius", sizeof(batt_temp_1->unit) - 1);
    strncpy(batt_temp_1->name, "BattTemp1", sizeof(batt_temp_1->name) - 1);
    
    // 배터리 온도 2
    data_config_t* batt_temp_2 = &g_vehicle_settings.data_configs[DATA_TYPE_BATT_TEMP_2];
    batt_temp_2->enabled = true;
    batt_temp_2->did = 0x0101;
    batt_temp_2->request_id = 0x7E4;            // 응답 ID는 0x7EC (자동 계산)
    batt_temp_2->data_offset = 17 + DATA_START_IDX;     // 17번째 바이트
    batt_temp_2->data_length = 1;
    batt_temp_2->resolution = 1.0f;
    batt_temp_2->offset_value = 0;
    batt_temp_2->is_signed = true;
    batt_temp_2->big_end_msb_first = true;
    batt_temp_2->request_period_ms = 1000;      // Steering과 동일한 주기
    strncpy(batt_temp_2->unit, "Celsius", sizeof(batt_temp_2->unit) - 1);
    strncpy(batt_temp_2->name, "BattTemp2", sizeof(batt_temp_2->name) - 1);

    // CRC32 계산
    g_vehicle_settings.crc32 = calculate_crc32((uint8_t*)&g_vehicle_settings, 
                                               sizeof(vehicle_settings_t) - sizeof(uint32_t));
}

// DIAG_DB 초기화
void diag_db_init(void)
{
    // 비휘발성 메모리에서 설정 로드 시도
    if (!diag_db_load_from_nvram()) {
        DEBUG_PRINT("[DIAG_DB] NVRAM load failed, using defaults\r\n");
        diag_db_load_defaults();
        diag_db_save_to_nvram();  // 기본값을 NVRAM에 저장
    }
    
    DEBUG_PRINT("[DIAG_DB] Initialized\r\n");
    diag_db_print_current_settings();
    
    // DID 그룹 빌드
    diag_db_rebuild_did_groups();
}

// 비휘발성 메모리 저장 (간단한 구현 - 실제로는 EEPROM/Flash 사용)
bool diag_db_save_to_nvram(void)
{
    // CRC32 재계산
    g_vehicle_settings.crc32 = calculate_crc32((uint8_t*)&g_vehicle_settings, 
                                               sizeof(vehicle_settings_t) - sizeof(uint32_t));
    
    // TODO: 실제 NVRAM/EEPROM 저장 구현
    DEBUG_PRINT("[DIAG_DB] Settings saved to NVRAM (CRC32: 0x%08lX)\r\n", g_vehicle_settings.crc32);
    return true;
}

// 비휘발성 메모리 로드 (간단한 구현)
bool diag_db_load_from_nvram(void)
{
    // TODO: 실제 NVRAM/EEPROM 로드 구현
    // 현재는 항상 실패로 처리하여 기본값 사용
    return false;
}

// 현재 설정 출력
void diag_db_print_current_settings(void)
{
    DEBUG_PRINT("[DIAG_DB] Vehicle: %s (v%d)\r\n", 
              g_vehicle_settings.vehicle_name, g_vehicle_settings.version);
    
    for (int i = 0; i < DATA_TYPE_MAX; i++) {
        data_config_t* cfg = &g_vehicle_settings.data_configs[i];
        if (cfg->enabled) {
            DEBUG_PRINT("[DIAG_DB] %s: DID=0x%04X, REQ=0x%lX, RESP=0x%lX, %s\r\n",
                      cfg->name, cfg->did, cfg->request_id, 
                      diag_db_get_response_id(cfg->request_id), cfg->unit);
        }
    }
}

// BLE 인터페이스용 설정 함수들
bool diag_db_set_vehicle_name(const char* name)
{
    if (!name || strlen(name) >= sizeof(g_vehicle_settings.vehicle_name)) {
        return false;
    }
    strncpy(g_vehicle_settings.vehicle_name, name, sizeof(g_vehicle_settings.vehicle_name) - 1);
    g_vehicle_settings.vehicle_name[sizeof(g_vehicle_settings.vehicle_name) - 1] = '\0';
    return true;
}

bool diag_db_set_data_config(data_type_t type, const data_config_t* config)
{
    if (type >= DATA_TYPE_MAX || !config) {
        return false;
    }
    memcpy(&g_vehicle_settings.data_configs[type], config, sizeof(data_config_t));
    return true;
}

bool diag_db_enable_data_type(data_type_t type, bool enable)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    g_vehicle_settings.data_configs[type].enabled = enable;
    return true;
}

bool diag_db_set_did(data_type_t type, uint16_t did)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    g_vehicle_settings.data_configs[type].did = did;
    return true;
}

bool diag_db_set_request_id(data_type_t type, uint32_t request_id)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    g_vehicle_settings.data_configs[type].request_id = request_id;
    
    // DID 그룹 재빌드 (CAN ID가 변경되었으므로)
    diag_db_rebuild_did_groups();
    return true;
}

bool diag_db_set_data_params(data_type_t type, uint8_t offset, uint8_t length, float resolution, int16_t offset_value)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    data_config_t* cfg = &g_vehicle_settings.data_configs[type];
    cfg->data_offset = offset;
    cfg->data_length = length;
    cfg->resolution = resolution;
    cfg->offset_value = offset_value;
    return true;
}

bool diag_db_set_data_format(data_type_t type, bool is_signed, bool big_end_msb_first)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    data_config_t* cfg = &g_vehicle_settings.data_configs[type];
    cfg->is_signed = is_signed;
    cfg->big_end_msb_first = big_end_msb_first;
    return true;
}

bool diag_db_set_data_info(data_type_t type, const char* name, const char* unit)
{
    if (type >= DATA_TYPE_MAX || !name || !unit) {
        return false;
    }
    data_config_t* cfg = &g_vehicle_settings.data_configs[type];
    strncpy(cfg->name, name, sizeof(cfg->name) - 1);
    cfg->name[sizeof(cfg->name) - 1] = '\0';
    strncpy(cfg->unit, unit, sizeof(cfg->unit) - 1);
    cfg->unit[sizeof(cfg->unit) - 1] = '\0';
    return true;
}

bool diag_db_set_request_period(data_type_t type, uint32_t period_ms)
{
    if (type >= DATA_TYPE_MAX) {
        return false;
    }
    g_vehicle_settings.data_configs[type].request_period_ms = period_ms;
    
    // DID 그룹 재빌드 (주기가 변경되었으므로)
    diag_db_rebuild_did_groups();
    return true;
}

// DID 그룹 재빌드
void diag_db_rebuild_did_groups(void)
{
    // 기존 그룹 초기화
    memset(g_did_groups, 0, sizeof(g_did_groups));
    g_did_group_count = 0;
    
    DEBUG_PRINT("[DIAG_DB] Rebuilding DID groups...\r\n");
    
    // 활성화된 데이터 타입들을 스캔하여 그룹 생성
    for (int i = 0; i < DATA_TYPE_MAX; i++) {
        data_config_t* cfg = &g_vehicle_settings.data_configs[i];
        
        // 활성화되고 주기가 설정된 데이터만 처리
        if (!cfg->enabled || cfg->request_period_ms == 0) {
            continue;
        }
        
        // 기존 그룹에서 같은 DID + CAN ID 조합 찾기
        did_group_t* group = NULL;
        uint32_t response_id = diag_db_get_response_id(cfg->request_id);
        if (!diag_db_find_did_group(cfg->did, cfg->request_id, response_id, &group)) {
            // 새 그룹 생성
            if (g_did_group_count >= MAX_DID_GROUPS) {
                DEBUG_PRINT("[DIAG_DB] Warning: Max DID groups reached!\r\n");
                break;
            }
            
            group = &g_did_groups[g_did_group_count++];
            group->did = cfg->did;
            group->request_id = cfg->request_id;
            group->response_id = response_id;
            group->last_request_time = 0;
            group->min_period_ms = cfg->request_period_ms;
            group->data_type_count = 0;
            group->is_active = true;
        }
        
        // 그룹에 데이터 타입 추가
        if (group->data_type_count < DATA_TYPE_MAX) {
            group->data_types[group->data_type_count++] = (data_type_t)i;
            
            // 최소 주기 업데이트 (가장 빠른 주기로 설정)
            if (cfg->request_period_ms < group->min_period_ms) {
                group->min_period_ms = cfg->request_period_ms;
            }
        }
    }
    
    // 생성된 그룹 정보 출력
    for (int i = 0; i < g_did_group_count; i++) {
        did_group_t* group = &g_did_groups[i];
        DEBUG_PRINT("[DIAG_DB] Group %d: DID=0x%04X, REQ=0x%lX, RESP=0x%lX, Period=%lums, Types=%d\r\n",
                  i, group->did, group->request_id, group->response_id, 
                  group->min_period_ms, group->data_type_count);
        
        // 그룹에 속한 데이터 타입들 출력
        for (int j = 0; j < group->data_type_count; j++) {
            DEBUG_PRINT("  - %s\r\n", diag_db_get_data_type_name(group->data_types[j]));
        }
    }
}

// DID 그룹 찾기
bool diag_db_find_did_group(uint16_t did, uint32_t request_id, uint32_t response_id, did_group_t** group)
{
    for (int i = 0; i < g_did_group_count; i++) {
        did_group_t* g = &g_did_groups[i];
        if (g->did == did && g->request_id == request_id && g->response_id == response_id) {
            if (group) *group = g;
            return true;
        }
    }
    return false;
}

// 그룹 요청 전송
void diag_db_send_group_request(did_group_t* group)
{
    if (!group || !group->is_active) {
        return;
    }
    
    // UDS Read Data by Identifier 요청 구성
    uint8_t request_data[] = {
        0x03,                           // PCI: Single Frame, 3바이트 데이터
        UDS_SERVICE_READ_DATA_BY_ID,    // Service: Read Data By Identifier (0x22)
        (group->did >> 8) & 0xFF,       // DID MSB
        group->did & 0xFF,              // DID LSB
        0x55, 0x55, 0x55, 0x55          // 패딩
    };
    
    // 그룹에 포함된 데이터 타입들을 함께 출력
    DEBUG_PRINT("[DIAG_DB] Group request: DID=0x%04X, %d data types [", 
              group->did, group->data_type_count);
    for (int i = 0; i < group->data_type_count; i++) {
        DEBUG_PRINT("%s", diag_db_get_data_type_name(group->data_types[i]));
        if (i < group->data_type_count - 1) {
            DEBUG_PRINT(", ");
        }
    }
    DEBUG_PRINT("]\r\n");
    
    // CAN 메시지 전송
    if (get_current_mode() == UDS_MODE_UDS_PATH) {
        // UDS_PATH 모드에서는 자동으로 주기적 요청
        can_tx_message(group->request_id, request_data, 8);
        group->last_request_time = millis();
    }
}

// 주기적 UDS 요청 처리 (메인 루프에서 호출)
void diag_db_process_periodic_requests(void)
{
    // UDS_PATH 모드에서만 주기적 요청 동작
    if (get_current_mode() != UDS_MODE_UDS_PATH) {
        return;
    }
    
    uint32_t current_time = millis();
    
    for (int i = 0; i < g_did_group_count; i++) {
        did_group_t* group = &g_did_groups[i];
        
        if (!group->is_active) {
            continue;
        }
        
        // 주기 체크
        if (current_time - group->last_request_time >= group->min_period_ms) {
            diag_db_send_group_request(group);
        }
    }
}

// 응답 CAN ID인지 확인 및 데이터 타입 반환
bool diag_db_is_response_id(uint32_t can_id, data_type_t* data_type)
{
    for (int i = 0; i < DATA_TYPE_MAX; i++) {
        data_config_t* cfg = &g_vehicle_settings.data_configs[i];
        if (cfg->enabled && diag_db_get_response_id(cfg->request_id) == can_id) {
            if (data_type) *data_type = (data_type_t)i;
            return true;
        }
    }
    return false;
}

// 범용 데이터 추출 함수
bool diag_db_extract_data_value(data_type_t type, uint8_t *complete_data, uint8_t data_length, float *value)
{
    
    if (type >= DATA_TYPE_MAX || !complete_data || !value) {
        DEBUG_PRINT("[DIAG_DB] === FUNCTION END: %s (invalid params) ===\r\n", diag_db_get_data_type_name(type));
        return false;
    }
    
    data_config_t* cfg = &g_vehicle_settings.data_configs[type];
    
    if (!cfg->enabled) {
        return false;
    }
    
    // UDS_PATH 모드에서만 Complete data 출력 (첫 번째 데이터 타입에서만)
    if (get_current_mode() == UDS_MODE_UDS_PATH && type == DATA_TYPE_STEERING) {
        DEBUG_PRINT("[DIAG_DB] Complete data (%d bytes): ", data_length);
        for (int i = 0; i < data_length; i++) {
            DEBUG_PRINT("%02X ", complete_data[i]);
        }
        DEBUG_PRINT("\r\n");
    }
    
    // 데이터 길이 확인
    if (data_length < (cfg->data_offset + cfg->data_length)) {
        return false;
    }
    
    // DID 확인 (응답 데이터 구조: [Service ID][DID MSB][DID LSB][Data...])
    if (data_length >= 3) {
        uint8_t service_id = complete_data[0];
        uint16_t did = (complete_data[1] << 8) | complete_data[2];
        
        if (service_id != UDS_SERVICE_READ_DATA_BY_ID_RESP || did != cfg->did) {
            return false;
        }
    }
    

    
    // 데이터 추출
    uint8_t *data_ptr = &complete_data[cfg->data_offset];
    uint32_t raw_value = 0;
    

    
    // 바이트 순서에 따른 데이터 읽기
    DEBUG_PRINT("[DIAG_DB] %s: Starting data conversion (big_endian=%d)\r\n", cfg->name, cfg->big_end_msb_first);
    
    if (cfg->big_end_msb_first) {
        // 빅엔디안 (MSB first)
        for (int i = 0; i < cfg->data_length; i++) {
            raw_value = (raw_value << 8) | data_ptr[i];
        }
    } else {
        // 리틀엔디안 (LSB first)
        for (int i = cfg->data_length - 1; i >= 0; i--) {
            raw_value = (raw_value << 8) | data_ptr[i];
        }
    }
    

    
    // 부호 처리
              
    if (cfg->is_signed && cfg->data_length <= 4) {

        int32_t signed_value;
        switch (cfg->data_length) {
            case 1:
                signed_value = (int8_t)raw_value;
                break;
            case 2:
                signed_value = (int16_t)raw_value;
                break;
            case 3:
                // 24-bit signed: MSB가 1이면 음수로 확장
                if (raw_value & 0x800000) {
                    signed_value = (int32_t)(raw_value | 0xFF000000);
                } else {
                    signed_value = (int32_t)raw_value;
                }
                break;
            case 4:
                signed_value = (int32_t)raw_value;
                break;
            default:
                signed_value = (int32_t)raw_value;
                break;
        }

        
        *value = (signed_value + cfg->offset_value) * cfg->resolution;
        // UDS_PATH 모드에서만 상세 출력
        if (get_current_mode() == UDS_MODE_UDS_PATH) {
            int final_int = (int)(*value * 10);
            int whole_part = final_int / 10;
            int decimal_part = final_int % 10;
            if (decimal_part < 0) decimal_part = -decimal_part;
            
            DEBUG_PRINT("[DIAG_DB] %s: Raw=0x%04X, Signed=%d, Resolution=%d, Final=%d.%d %s\r\n", 
                       cfg->name, raw_value, signed_value, (int)cfg->resolution, 
                       whole_part, decimal_part, cfg->unit);
        }
    } else {
        *value = (raw_value + cfg->offset_value) * cfg->resolution;
        
        // UDS_PATH 모드에서만 상세 출력
        if (get_current_mode() == UDS_MODE_UDS_PATH) {
            int final_int = (int)(*value * 10);
            int whole_part = final_int / 10;
            int decimal_part = final_int % 10;
            if (decimal_part < 0) decimal_part = -decimal_part;
            
            DEBUG_PRINT("[DIAG_DB] %s: Raw=0x%04X, Unsigned=%u, Resolution=%d, Final=%d.%d %s\r\n", 
                       cfg->name, raw_value, raw_value, (int)cfg->resolution,
                       whole_part, decimal_part, cfg->unit);
        }
    }    
    return true;
}

// 데이터 요청 전송
void diag_db_send_data_request(data_type_t type)
{
    if (type >= DATA_TYPE_MAX) {
        return;
    }
    
    data_config_t* cfg = &g_vehicle_settings.data_configs[type];
    
    if (!cfg->enabled) {
        DEBUG_PRINT("[DIAG_DB] %s is disabled\r\n", cfg->name);
        return;
    }
    
    // UDS Read Data by Identifier 요청 구성
    uint8_t request_data[] = {
        0x03,                           // PCI: Single Frame, 3바이트 데이터
        UDS_SERVICE_READ_DATA_BY_ID,    // Service: Read Data By Identifier (0x22)
        (cfg->did >> 8) & 0xFF,         // DID MSB
        cfg->did & 0xFF,                // DID LSB
        0x55, 0x55, 0x55, 0x55          // 패딩
    };
    
    DEBUG_PRINT("[DIAG_DB] Sending %s request (DID 0x%04X) to ID 0x%lX\r\n", 
              cfg->name, cfg->did, cfg->request_id);
    
    // CAN 메시지 전송 (TALK 모드에서만 가능)
    if (get_current_mode() == UDS_MODE_TALK) {
        can_tx_message(cfg->request_id, request_data, 8);
    } else {
        DEBUG_PRINT("[DIAG_DB] Request can only be sent in TALK mode\r\n");
    }
}

// 데이터 타입 이름 반환
const char* diag_db_get_data_type_name(data_type_t type)
{
    static const char* type_names[] = {
        "Steering", "Speed", "EngineRPM", "BattTemp1", "BattTemp2"
    };
    
    if (type < DATA_TYPE_MAX) {
        return type_names[type];
    }
    return "Unknown";
}

// 기존 호환성을 위한 편의 함수들
bool diag_db_is_steering_response(uint32_t can_id)
{
    data_type_t type;
    if (diag_db_is_response_id(can_id, &type)) {
        return (type == DATA_TYPE_STEERING);
    }
    return false;
}

bool diag_db_extract_steering_angle(uint8_t *complete_data, uint8_t data_length, float *steering_angle)
{
    return diag_db_extract_data_value(DATA_TYPE_STEERING, complete_data, data_length, steering_angle);
}

void diag_db_send_steering_request(void)
{
    diag_db_send_data_request(DATA_TYPE_STEERING);
}

const char* diag_db_get_vehicle_name(void)
{
    return g_vehicle_settings.vehicle_name;
}

// BLE 명령 처리 함수들 (예시 구현)
// 명령 형식: "SET_VEHICLE_NAME:SONATA_DN8_HME"
ble_cmd_result_t diag_db_ble_set_vehicle_name(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    const char* colon = strchr(cmd, ':');
    if (!colon) return BLE_CMD_ERROR_INVALID_FORMAT;
    
    const char* name = colon + 1;
    if (strlen(name) >= sizeof(g_vehicle_settings.vehicle_name)) {
        return BLE_CMD_ERROR_OUT_OF_RANGE;
    }
    
    if (diag_db_set_vehicle_name(name)) {
        DEBUG_PRINT("[BLE] Vehicle name set to: %s\r\n", name);
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "ENABLE_DATA:0:1" (데이터타입:활성화여부)
ble_cmd_result_t diag_db_ble_enable_data_type(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type, enable;
    if (sscanf(cmd, "ENABLE_DATA:%d:%d", &type, &enable) != 2) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_enable_data_type((data_type_t)type, enable != 0)) {
        DEBUG_PRINT("[BLE] %s %s\r\n", 
                  diag_db_get_data_type_name((data_type_t)type),
                  enable ? "enabled" : "disabled");
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_DID:0:0x0101" (데이터타입:DID)
ble_cmd_result_t diag_db_ble_set_did(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type;
    unsigned int did;
    if (sscanf(cmd, "SET_DID:%d:0x%x", &type, &did) != 2) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_set_did((data_type_t)type, (uint16_t)did)) {
        DEBUG_PRINT("[BLE] %s DID set to 0x%04X\r\n", 
                  diag_db_get_data_type_name((data_type_t)type), did);
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_REQUEST_ID:0:0x7D4" (데이터타입:요청ID)
ble_cmd_result_t diag_db_ble_set_request_id(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type;
    unsigned int req_id;
    if (sscanf(cmd, "SET_REQUEST_ID:%d:0x%x", &type, &req_id) != 2) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_set_request_id((data_type_t)type, req_id)) {
        DEBUG_PRINT("[BLE] %s Request ID set: REQ=0x%lX, RESP=0x%lX\r\n", 
                  diag_db_get_data_type_name((data_type_t)type), 
                  (uint32_t)req_id, diag_db_get_response_id(req_id));
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_DATA_PARAMS:0:7:2:0.1:-100" (타입:오프셋:길이:분해능:오프셋값)
ble_cmd_result_t diag_db_ble_set_data_params(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type, offset, length, offset_value;
    float resolution;
    if (sscanf(cmd, "SET_DATA_PARAMS:%d:%d:%d:%f:%d", 
               &type, &offset, &length, &resolution, &offset_value) != 5) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_set_data_params((data_type_t)type, (uint8_t)offset, (uint8_t)length, 
                               resolution, (int16_t)offset_value)) {
        DEBUG_PRINT("[BLE] %s data params set: offset=%d, len=%d, res=%.3f, off=%d\r\n", 
                  diag_db_get_data_type_name((data_type_t)type), 
                  offset, length, resolution, offset_value);
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_DATA_FORMAT:0:1:1" (타입:부호여부:빅엔디안여부)
ble_cmd_result_t diag_db_ble_set_data_format(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type, is_signed, big_end_msb_first;
    if (sscanf(cmd, "SET_DATA_FORMAT:%d:%d:%d", &type, &is_signed, &big_end_msb_first) != 3) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_set_data_format((data_type_t)type, is_signed != 0, big_end_msb_first != 0)) {
        DEBUG_PRINT("[BLE] %s format set: %s, %s\r\n", 
                  diag_db_get_data_type_name((data_type_t)type),
                  is_signed ? "signed" : "unsigned",
                  big_end_msb_first ? "big-endian" : "little-endian");
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_DATA_INFO:0:Steering:deg" (타입:이름:단위)
ble_cmd_result_t diag_db_ble_set_data_info(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    char type_str[8], name[32], unit[16];
    if (sscanf(cmd, "SET_DATA_INFO:%7[^:]:%31[^:]:%15s", type_str, name, unit) != 3) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    int type = atoi(type_str);
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (diag_db_set_data_info((data_type_t)type, name, unit)) {
        DEBUG_PRINT("[BLE] %s info set: name=%s, unit=%s\r\n", 
                  diag_db_get_data_type_name((data_type_t)type), name, unit);
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// 명령 형식: "SET_PERIOD:0:100" (타입:주기ms)
ble_cmd_result_t diag_db_ble_set_request_period(const char* cmd)
{
    if (!cmd) return BLE_CMD_ERROR_INVALID_PARAM;
    
    int type, period;
    if (sscanf(cmd, "SET_PERIOD:%d:%d", &type, &period) != 2) {
        return BLE_CMD_ERROR_INVALID_FORMAT;
    }
    
    if (type < 0 || type >= DATA_TYPE_MAX) {
        return BLE_CMD_ERROR_INVALID_TYPE;
    }
    
    if (period < 0) {
        return BLE_CMD_ERROR_OUT_OF_RANGE;
    }
    
    if (diag_db_set_request_period((data_type_t)type, (uint32_t)period)) {
        DEBUG_PRINT("[BLE] %s period set to %dms\r\n", 
                  diag_db_get_data_type_name((data_type_t)type), period);
        return BLE_CMD_OK;
    }
    return BLE_CMD_ERROR_INVALID_PARAM;
}

// BLE 명령 에러 문자열 반환
const char* diag_db_ble_get_error_string(ble_cmd_result_t result)
{
    switch (result) {
        case BLE_CMD_OK: return "OK";
        case BLE_CMD_ERROR_INVALID_PARAM: return "Invalid parameter";
        case BLE_CMD_ERROR_INVALID_TYPE: return "Invalid data type";
        case BLE_CMD_ERROR_INVALID_FORMAT: return "Invalid command format";
        case BLE_CMD_ERROR_OUT_OF_RANGE: return "Value out of range";
        default: return "Unknown error";
    }
}
