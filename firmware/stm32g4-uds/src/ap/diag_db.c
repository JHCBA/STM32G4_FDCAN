#include "diag_db.h"
#include "ap.h" 
#include "comm_handler.h"
#include <string.h>
#include <stdio.h>

// DEBUG 출력 매크로
#define DEBUG_CAN_PROTOCOL  0  // 1: 활성화, 0: 비활성화

#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) cdc_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

#define DATA_START_IDX  3
// 전역 차량 설정 변수 (BLE로 업데이트 가능)
vehicle_settings_t g_vehicle_settings;

// DID 그룹 관리 변수
did_group_t g_did_groups[MAX_DID_GROUPS];
uint8_t g_did_group_count = 0;

// UDS 요청 큐 관리 변수
static uds_request_queue_item_t g_uds_request_queue[UDS_REQUEST_QUEUE_SIZE];
static int g_uds_queue_head = 0;
static int g_uds_queue_tail = 0;
static int g_uds_queue_count = 0;
static uds_request_state_t g_uds_current_state = UDS_REQUEST_STATE_IDLE;
static uint32_t g_uds_current_request_time = 0;
static did_group_t* g_uds_current_group = NULL;

// UDS 큐 초기화 함수
static void uds_queue_init(void)
{
    memset(g_uds_request_queue, 0, sizeof(g_uds_request_queue));
    g_uds_queue_head = 0;
    g_uds_queue_tail = 0;
    g_uds_queue_count = 0;
    g_uds_current_state = UDS_REQUEST_STATE_IDLE;
    g_uds_current_request_time = 0;
    g_uds_current_group = NULL;
    DEBUG_PRINT("[UDS_QUEUE] Queue initialized\r\n");
}

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
    
    // 스티어링 데이터 기본 설정
    data_config_t* steering = &g_vehicle_settings.data_configs[DATA_TYPE_STEERING_MDPS];
    steering->enabled = true;
    steering->did = 0x0101;
    steering->request_id = 0x7D4;       // 응답 ID는 0x7DC (자동 계산)
    steering->data_offset = 7 + DATA_START_IDX;          // 7,8번째 바이트
    steering->data_length = 2;
    steering->resolution = 0.1f;
    steering->offset_value = 0;
    steering->is_signed = true;
    steering->big_end_msb_first = true;     // MSB first
    steering->request_period_ms = 500;  // 100ms 주기
    strncpy(steering->unit, "deg", sizeof(steering->unit) - 1);
    strncpy(steering->name, "Steering", sizeof(steering->name) - 1);
    
    // 속도 데이터 기본 설정
    data_config_t* speed = &g_vehicle_settings.data_configs[DATA_TYPE_SPEED_MDPS];
    speed->enabled = true;            // Steering과 함께 활성화
    speed->did = 0x0101;                // 스티어링과 같은 DID 사용
    speed->request_id = 0x7D4;          // 응답 ID는 0x7DC (자동 계산)
    speed->data_offset = 12 + DATA_START_IDX;          // 12번째 바이트
    speed->data_length = 1;
    speed->resolution = 2.0f;
    speed->offset_value = 0;
    speed->is_signed = false;
    speed->big_end_msb_first = false;
    speed->request_period_ms = 500;     // Steering과 같은 주기로 변경
    strncpy(speed->unit, "km/h", sizeof(speed->unit) - 1);
    strncpy(speed->name, "Speed", sizeof(speed->name) - 1);
    
    // APS ECU 데이터 설정 TODO
    data_config_t* aps_ecu = &g_vehicle_settings.data_configs[DATA_TYPE_APS_ECU];
    aps_ecu->enabled = false;  // 기본적으로 비활성화
    aps_ecu->did = 0x0102;
    aps_ecu->request_id = 0x7D4;
    aps_ecu->data_offset = 5 + DATA_START_IDX;
    aps_ecu->data_length = 1;
    aps_ecu->resolution = 0.4f;  // 0~100% -> 0~255
    aps_ecu->offset_value = 0;
    aps_ecu->is_signed = false;
    aps_ecu->big_end_msb_first = false;
    aps_ecu->request_period_ms = 500;
    strncpy(aps_ecu->unit, "%", sizeof(aps_ecu->unit) - 1);
    strncpy(aps_ecu->name, "APS_ECU", sizeof(aps_ecu->name) - 1);
    
    // BPS ECU 데이터 설정 TODO
    data_config_t* bps_ecu = &g_vehicle_settings.data_configs[DATA_TYPE_BPS_ECU];
    bps_ecu->enabled = false;  // 기본적으로 비활성화
    bps_ecu->did = 0x0102;
    bps_ecu->request_id = 0x7D4;
    bps_ecu->data_offset = 6 + DATA_START_IDX;
    bps_ecu->data_length = 1;
    bps_ecu->resolution = 0.4f;  // 0~100% -> 0~255
    bps_ecu->offset_value = 0;
    bps_ecu->is_signed = false;
    bps_ecu->big_end_msb_first = false;
    bps_ecu->request_period_ms = 500;
    strncpy(bps_ecu->unit, "%", sizeof(bps_ecu->unit) - 1);
    strncpy(bps_ecu->name, "BPS_ECU", sizeof(bps_ecu->name) - 1);
    
    // APS VCU 데이터 설정
    data_config_t* aps_vcu = &g_vehicle_settings.data_configs[DATA_TYPE_APS_VCU];
    aps_vcu->enabled = true;  // 기본적으로 비활성화
    aps_vcu->did = 0xE004;
    aps_vcu->request_id = 0x7E2;
    aps_vcu->data_offset = 8 + DATA_START_IDX;
    aps_vcu->data_length = 2;
    aps_vcu->resolution = 0.001953125f;
    aps_vcu->offset_value = 0;
    aps_vcu->is_signed = false;
    aps_vcu->big_end_msb_first = false;
    aps_vcu->request_period_ms = 500;
    strncpy(aps_vcu->unit, "%", sizeof(aps_vcu->unit) - 1);
    strncpy(aps_vcu->name, "APS_VCU", sizeof(aps_vcu->name) - 1);
    
    // BPS VCU 데이터 설정
    data_config_t* bps_vcu = &g_vehicle_settings.data_configs[DATA_TYPE_BPS_ABS];
    bps_vcu->enabled = true;  // 기본적으로 비활성화
    bps_vcu->did = 0x0104;
    bps_vcu->request_id = 0x7D1;
    bps_vcu->data_offset = 38 + DATA_START_IDX;
    bps_vcu->data_length = 2;
    bps_vcu->resolution = 0.01f;
    bps_vcu->offset_value = 0;
    bps_vcu->is_signed = true;
    bps_vcu->big_end_msb_first = true;
    bps_vcu->request_period_ms = 500;
    strncpy(bps_vcu->unit, "mm", sizeof(bps_vcu->unit) - 1);
    strncpy(bps_vcu->name, "BPS_VCU", sizeof(bps_vcu->name) - 1);
    
    // 기어 상태 설정
    data_config_t* gear = &g_vehicle_settings.data_configs[DATA_TYPE_GEAR_VCU];
    gear->enabled = true;  // 기본적으로 비활성화
    gear->did = 0xE004;
    gear->request_id = 0x7E2;
    gear->data_offset = 14 + DATA_START_IDX;
    gear->data_length = 1;
    gear->resolution = 1.0f;  // P=0, R=1, N=2, D=3
    gear->offset_value = 0;
    gear->is_signed = false;
    gear->big_end_msb_first = false;
    gear->request_period_ms = 500;  // 2초 주기
    strncpy(gear->unit, "", sizeof(gear->unit) - 1);
    strncpy(gear->name, "Gear", sizeof(gear->name) - 1);
    
    // 방향지시등 설정
    data_config_t* turn_sig = &g_vehicle_settings.data_configs[DATA_TYPE_TURN_SIG_MFSW];
    turn_sig->enabled = true;  // 기본적으로 비활성화
    turn_sig->did = 0xB303;
    turn_sig->request_id = 0x7A6;
    turn_sig->data_offset = 4 + DATA_START_IDX;
    turn_sig->data_length = 1;
    turn_sig->resolution = 1.0f;  // 비트마스크: Left=1, Right=2, Hazard=3
    turn_sig->offset_value = 0;
    turn_sig->is_signed = false;
    turn_sig->big_end_msb_first = false;
    turn_sig->request_period_ms = 500;  // 500ms 주기
    strncpy(turn_sig->unit, "", sizeof(turn_sig->unit) - 1);
    strncpy(turn_sig->name, "TurnSignal", sizeof(turn_sig->name) - 1);
    
    // 차량 문 열림 설정
    data_config_t* door_open = &g_vehicle_settings.data_configs[DATA_TYPE_DOOR_OPEN];
    door_open->enabled = false;  // 기본적으로 비활성화
    door_open->did = 0x0106;
    door_open->request_id = 0x7D4;
    door_open->data_offset = 13 + DATA_START_IDX;
    door_open->data_length = 1;
    door_open->resolution = 1.0f;  // 비트마스크: FL=1, FR=2, RL=4, RR=8
    door_open->offset_value = 0;
    door_open->is_signed = false;
    door_open->big_end_msb_first = false;
    door_open->request_period_ms = 500;  // 2초 주기
    strncpy(door_open->unit, "", sizeof(door_open->unit) - 1);
    strncpy(door_open->name, "DoorOpen", sizeof(door_open->name) - 1);
    
    // 안전벨트 설정
    data_config_t* seat_belt = &g_vehicle_settings.data_configs[DATA_TYPE_SEAT_BELT];
    seat_belt->enabled = false;  // 기본적으로 비활성화
    seat_belt->did = 0x0107;
    seat_belt->request_id = 0x7D4;
    seat_belt->data_offset = 14 + DATA_START_IDX;
    seat_belt->data_length = 1;
    seat_belt->resolution = 1.0f;  // 비트마스크: Driver=1, Passenger=2
    seat_belt->offset_value = 0;
    seat_belt->is_signed = false;
    seat_belt->big_end_msb_first = false;
    seat_belt->request_period_ms = 500;  // 2초 주기
    strncpy(seat_belt->unit, "", sizeof(seat_belt->unit) - 1);
    strncpy(seat_belt->name, "SeatBelt", sizeof(seat_belt->name) - 1);
    
    // 레이더 설정
    data_config_t* radar = &g_vehicle_settings.data_configs[DATA_TYPE_RADAR];
    radar->enabled = false;  // 기본적으로 비활성화
    radar->did = 0x0108;
    radar->request_id = 0x7D4;
    radar->data_offset = 18 + DATA_START_IDX;
    radar->data_length = 2;
    radar->resolution = 0.1f;  // 0.1m 단위
    radar->offset_value = 0;
    radar->is_signed = false;
    radar->big_end_msb_first = true;
    radar->request_period_ms = 500;  // 200ms 주기 (빠른 업데이트)
    strncpy(radar->unit, "m", sizeof(radar->unit) - 1);
    strncpy(radar->name, "Radar", sizeof(radar->name) - 1);
    
    // 배터리 온도 설정 (기존 BATT_TEMP_1을 BATT_TEMP로 변경)
    data_config_t* batt_temp = &g_vehicle_settings.data_configs[DATA_TYPE_BATT_TEMP_BMS];
    batt_temp->enabled = true;
    batt_temp->did = 0x0101;
    batt_temp->request_id = 0x7E4;            // 응답 ID는 0x7EC (자동 계산)
    batt_temp->data_offset = 16 + DATA_START_IDX;     // 16번째 바이트
    batt_temp->data_length = 1;
    batt_temp->resolution = 1.0f;
    batt_temp->offset_value = 0;
    batt_temp->is_signed = true;
    batt_temp->big_end_msb_first = true;
    batt_temp->request_period_ms = 500;      // Steering과 동일한 주기
    strncpy(batt_temp->unit, "Celsius", sizeof(batt_temp->unit) - 1);
    strncpy(batt_temp->name, "BattTemp", sizeof(batt_temp->name) - 1);

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
    
    // UDS 큐 초기화
    uds_queue_init();
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

// UDS 요청 큐에 추가 (중복 방지)
static bool uds_queue_add_request(did_group_t* group)
{
    // 큐가 가득 찬 경우
    if (g_uds_queue_count >= UDS_REQUEST_QUEUE_SIZE) {
        DEBUG_PRINT("[UDS_QUEUE] Queue full, dropping request for DID 0x%04X\r\n", group->did);
        return false;
    }
    
    // 이미 큐에 있는지 확인 (중복 방지)
    for (int i = 0; i < g_uds_queue_count; i++) {
        int index = (g_uds_queue_head + i) % UDS_REQUEST_QUEUE_SIZE;
        uds_request_queue_item_t* item = &g_uds_request_queue[index];
        if (item->group == group && item->is_pending) {
            DEBUG_PRINT("[UDS_QUEUE] Request for DID 0x%04X already in queue, skipping\r\n", group->did);
            return false;
        }
    }
    
    // 현재 처리 중인 요청과 중복 확인
    if (g_uds_current_group == group && g_uds_current_state == UDS_REQUEST_STATE_WAITING_RESPONSE) {
        DEBUG_PRINT("[UDS_QUEUE] Request for DID 0x%04X already being processed, skipping\r\n", group->did);
        return false;
    }
    
    uds_request_queue_item_t* item = &g_uds_request_queue[g_uds_queue_tail];
    item->group = group;
    item->request_time = millis();
    item->is_pending = true;
    
    g_uds_queue_tail = (g_uds_queue_tail + 1) % UDS_REQUEST_QUEUE_SIZE;
    g_uds_queue_count++;
    
    DEBUG_PRINT("[UDS_QUEUE] Added request for DID 0x%04X (queue size: %d)\r\n", 
               group->did, g_uds_queue_count);
    return true;
}

// UDS 요청 큐에서 다음 요청 가져오기
static uds_request_queue_item_t* uds_queue_get_next_request(void)
{
    if (g_uds_queue_count == 0) {
        return NULL;
    }
    
    uds_request_queue_item_t* item = &g_uds_request_queue[g_uds_queue_head];
    return item;
}

// UDS 요청 큐에서 완료된 요청 제거
static void uds_queue_remove_completed_request(void)
{
    if (g_uds_queue_count == 0) {
        return;
    }
    
    uds_request_queue_item_t* item = &g_uds_request_queue[g_uds_queue_head];
    DEBUG_PRINT("[UDS_QUEUE] Completed request for DID 0x%04X\r\n", item->group->did);
    
    g_uds_queue_head = (g_uds_queue_head + 1) % UDS_REQUEST_QUEUE_SIZE;
    g_uds_queue_count--;
}

// UDS 응답 수신 처리
void diag_db_handle_uds_response(uint32_t can_id)
{
    DEBUG_PRINT("[UDS_QUEUE] Response received: CAN ID 0x%lX, Current state: %d, Current group: %p\r\n", 
               can_id, g_uds_current_state, g_uds_current_group);
    
    if (g_uds_current_state == UDS_REQUEST_STATE_WAITING_RESPONSE && g_uds_current_group) {
        // 현재 대기 중인 요청의 응답인지 확인
        uint32_t expected_response_id = diag_db_get_response_id(g_uds_current_group->request_id);
        DEBUG_PRINT("[UDS_QUEUE] Expected response ID: 0x%lX, Received: 0x%lX\r\n", 
                   expected_response_id, can_id);
        
        if (expected_response_id == can_id) {
            DEBUG_PRINT("[UDS_QUEUE] Response matched for DID 0x%04X (CAN ID: 0x%lX)\r\n", 
                       g_uds_current_group->did, can_id);
            g_uds_current_state = UDS_REQUEST_STATE_COMPLETED;
        } else {
            DEBUG_PRINT("[UDS_QUEUE] Response ID mismatch for DID 0x%04X\r\n", g_uds_current_group->did);
        }
    } else {
        DEBUG_PRINT("[UDS_QUEUE] No active request waiting for response\r\n");
    }
}

// 주기적 UDS 요청 처리 (순차 처리 방식으로 변경)
void diag_db_process_periodic_requests(void)
{
    // UDS_PATH 모드에서만 주기적 요청 동작
    if (get_current_mode() != UDS_MODE_UDS_PATH) {
        return;
    }
    
    uint32_t current_time = millis();
    
    // 현재 UDS 요청 상태 처리
    switch (g_uds_current_state) {
        case UDS_REQUEST_STATE_IDLE:
        {
            // ISO-TP 멀티프레임 수신 중인지 확인
            extern bool can_manager_is_isotp_active(void);
            if (can_manager_is_isotp_active()) {
                DEBUG_PRINT("[UDS_QUEUE] Waiting for ISO-TP multi-frame completion\r\n");
                break;  // 멀티프레임 수신 중이면 대기
            }
            
            // 주기적 요청 체크 및 큐에 추가
            for (int i = 0; i < g_did_group_count; i++) {
                did_group_t* group = &g_did_groups[i];
                
                if (!group->is_active) {
                    continue;
                }
                
                // 주기 체크
                if (current_time - group->last_request_time >= group->min_period_ms) {
                    DEBUG_PRINT("[UDS_QUEUE] Period check: DID 0x%04X (last: %lu, period: %lu, current: %lu)\r\n", 
                               group->did, group->last_request_time, group->min_period_ms, current_time);
                    uds_queue_add_request(group);
                }
            }
            
            // 큐에서 다음 요청 처리
            uds_request_queue_item_t* next_request = uds_queue_get_next_request();
            if (next_request) {
                g_uds_current_group = next_request->group;
                g_uds_current_request_time = current_time;
                g_uds_current_state = UDS_REQUEST_STATE_WAITING_RESPONSE;
                
                DEBUG_PRINT("[UDS_QUEUE] Processing request for DID 0x%04X (queue remaining: %d)\r\n", 
                           g_uds_current_group->did, g_uds_queue_count - 1);
                
                // 실제 UDS 요청 전송
                diag_db_send_group_request(g_uds_current_group);
            }
            break;
        }
        
        case UDS_REQUEST_STATE_WAITING_RESPONSE:
        {
            // 응답 타임아웃 체크
            uint32_t elapsed_time = current_time - g_uds_current_request_time;
            if (elapsed_time >= UDS_RESPONSE_TIMEOUT_MS) {
                DEBUG_PRINT("[UDS_QUEUE] Timeout for DID 0x%04X after %lums (timeout: %dms)\r\n", 
                           g_uds_current_group->did, elapsed_time, UDS_RESPONSE_TIMEOUT_MS);
                g_uds_current_state = UDS_REQUEST_STATE_TIMEOUT;
            } else {
                // 주기적으로 대기 상태 로그 (5초마다)
                static uint32_t last_waiting_log = 0;
                if (current_time - last_waiting_log >= 5000) {
                    DEBUG_PRINT("[UDS_QUEUE] Waiting for response: DID 0x%04X, elapsed: %lums\r\n", 
                               g_uds_current_group->did, elapsed_time);
                    last_waiting_log = current_time;
                }
            }
            break;
        }
        
        case UDS_REQUEST_STATE_COMPLETED:
        {
            DEBUG_PRINT("[UDS_QUEUE] Request completed for DID 0x%04X\r\n", g_uds_current_group->did);
            // 완료된 요청 제거 및 상태 초기화
            uds_queue_remove_completed_request();
            g_uds_current_group = NULL;
            g_uds_current_state = UDS_REQUEST_STATE_IDLE;
            break;
        }
        
        case UDS_REQUEST_STATE_TIMEOUT:
        {
            DEBUG_PRINT("[UDS_QUEUE] Request timeout for DID 0x%04X\r\n", g_uds_current_group->did);
            // 타임아웃된 요청 제거 및 상태 초기화
            uds_queue_remove_completed_request();
            g_uds_current_group = NULL;
            g_uds_current_state = UDS_REQUEST_STATE_IDLE;
            break;
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
    if (get_current_mode() == UDS_MODE_UDS_PATH && type == DATA_TYPE_STEERING_MDPS) {
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
        "Steering",      // DATA_TYPE_STEERING_MDPS = 0
        "Speed",         // DATA_TYPE_SPEED_MDPS = 1
        "APS_ECU",       // DATA_TYPE_APS_ECU = 2
        "BPS_ECU",       // DATA_TYPE_BPS_ECU = 3
        "APS_VCU",       // DATA_TYPE_APS_VCU = 4
        "BPS_VCU",       // DATA_TYPE_BPS_ABS = 5
        "Gear",          // DATA_TYPE_GEAR_VCU = 6
        "TurnSignal",    // DATA_TYPE_TURN_SIG_MFSW = 7
        "DoorOpen",      // DATA_TYPE_DOOR_OPEN = 8
        "SeatBelt",      // DATA_TYPE_SEAT_BELT = 9
        "Radar",         // DATA_TYPE_RADAR = 10
        "BattTemp"       // DATA_TYPE_BATT_TEMP_BMS = 11
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
        return (type == DATA_TYPE_STEERING_MDPS);
    }
    return false;
}

bool diag_db_extract_steering_angle(uint8_t *complete_data, uint8_t data_length, float *steering_angle)
{
    return diag_db_extract_data_value(DATA_TYPE_STEERING_MDPS, complete_data, data_length, steering_angle);
}

void diag_db_send_steering_request(void)
{
    diag_db_send_data_request(DATA_TYPE_STEERING_MDPS);
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
