#include "can_manager.h"
#include "can_db.h"

// DEBUG 출력 매크로
#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) all_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// 디버깅 매크로들
#define DEBUG_CAN 0  // CAN 디버깅 활성화 (1로 설정하면 디버그 목록의 ID만 출력)

// DEBUG_CAN이 1일 때 출력할 CAN ID 목록
static const uint32_t debug_can_ids[] = {
    //0xEA,   // MDPS (Steering Angle)
    //0xA0,   // WHEEL_SPEEDS (Vehicle Speed)
    //0x60,   // BPS
    //0x100,  // ACCELERATOR_BRAKE_ALT (BPS)
    //0x40,   // GEAR_ALT_2 (Gear)
    //0x125,  // STEERING_SENSORS (EPS Error)
    //0x413,  // BLINKERS (Turn Signal)
    //0x411,  // DOORS_SEATBELTS (Door/Seatbelt)
    0x1BA,  // BLINDSPOTS_REAR_CORNERS (Radar)
};

#define DEBUG_CAN_IDS_COUNT (sizeof(debug_can_ids) / sizeof(debug_can_ids[0]))

// CAN ID가 디버그 목록에 있는지 확인하는 함수
static bool is_debug_can_id(uint32_t can_id) {
    for (int i = 0; i < DEBUG_CAN_IDS_COUNT; i++) {
        if (debug_can_ids[i] == can_id) {
            return true;
        }
    }
    return false;
}

// CAN 모드 설정
can_mode_t current_can_mode = CAN_FILTER_LISTEN;  // 기본값: 필터 리스닝 모드

// RX 관련 변수들
#define MAX_RX_IDS 30  // 최대 추적할 수 있는 CAN ID 개수
static uint32_t rx_display_time = 0;
// 모든 CAN ID 수신을 위한 설정
#define MAX_ALL_CAN_IDS MAX_RX_IDS  // 최대 추적할 수 있는 모든 CAN ID 개수 (메모리 절약)
static all_can_tracker_t all_can_trackers[MAX_ALL_CAN_IDS];
static uint8_t all_can_active_count = 0;
static uint32_t all_can_output_period = 1000;  // 기본 출력 주기: 1000ms
static uint32_t all_can_timeout_period = 3000;  // 타임아웃 주기: 3000ms (3초)

// 디버깅 모드를 위한 설정
static uint32_t debug_output_time = 0;
static uint32_t debug_counter = 0;

// SCAN 모드를 위한 설정
#define MAX_SCAN_IDS 500  // 최대 추적할 수 있는 CAN ID 개수
static scan_can_tracker_t scan_trackers[MAX_SCAN_IDS];
static uint8_t scan_active_count = 0;
static uint32_t scan_output_period = 2000;  // 출력 주기: 1000ms
static uint32_t scan_timeout_period = 1000;  // 타임아웃 주기: 1000ms

// 샘플 CAN FD 메시지 정의
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[64];
} sample_can_msg_t;

// 실제 CAN FD 메시지 샘플들
static const sample_can_msg_t sample_messages[] = {
    {
        .id = 0xEA,
        .dlc = CAN_DLC_24,
        .data = {0x7E, 0x41, 0xBB, 0x00, 0x01, 0x41, 0x00, 0x00, 
                 0x01, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 
                 0xAC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },
    {
        .id = 0x125,
        .dlc = CAN_DLC_16,
        .data = {0x3E, 0x9F, 0xB3, 0xBB, 0xFF, 0x00, 0x07, 0x00, 
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },
    {
        .id = 0x60,
        .dlc = CAN_DLC_32,
        .data = {0x2B, 0x59, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 
                 0x02, 0x02, 0x00, 0x02, 0x00, 0xFF, 0x00, 0xFF, 
                 0xAD, 0x00, 0x15, 0x00, 0x00, 0x08, 0x03, 0x01, 
                 0x40, 0x00, 0x00, 0x04, 0xFF, 0xFA, 0x00, 0x00}
    },
    {
        .id = 0x100,
        .dlc = CAN_DLC_32,
        .data = {0xED, 0xA6, 0xF6, 0x00, 0x11, 0xAD, 0x7E, 0x6C, 
                 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x57, 
                 0x23, 0x00, 0x48, 0x00, 0xE5, 0x00, 0x00, 0x00, 
                 0xFE, 0x0F, 0x00, 0x90, 0x93, 0x00, 0x00, 0x00}
    }
};

#define SAMPLE_MSG_COUNT (sizeof(sample_messages) / sizeof(sample_messages[0]))

// 필터링할 CAN ID 목록
#define FILTERED_ID_COUNT 10

// CAN ID to Protocol ID 매핑 테이블
// 필터링된 CAN ID들을 순서대로 프로토콜 Data ID로 매핑
static const can_to_protocol_map_t can_protocol_map[] = {
    {0xA0,  PROTOCOL_ID_VEHICLE_SPEED,   100},  // Vehicle Speed
    {0x60,  PROTOCOL_ID_BPS,            100},  // APS
    {0x100, PROTOCOL_ID_APS,            100},  // BPS
    {0xEA,  PROTOCOL_ID_STEERING_ANGLE, 100},  // Steering Angle
    {0x125, PROTOCOL_ID_EPS_ERR,        100},  // EPS Error
    {0x40, PROTOCOL_ID_GEAR,    100},  // Gear Status
    {0x413, PROTOCOL_ID_TURN_SIGNAL,    100},  // Turn Signal
    {0x411, PROTOCOL_ID_DOOR_OPEN,     1000},  // Door Open
    {0x411, PROTOCOL_ID_SEAT_BELT,      1000},  // Seat Belt
    {0x1BA, PROTOCOL_ID_RADAR,          100},  // Radar
};

#define CAN_PROTOCOL_MAP_SIZE (sizeof(can_protocol_map) / sizeof(can_protocol_map[0]))

// 프로토콜 ID별 설명 문자열
static const char* protocol_descriptions[] = {
    [PROTOCOL_ID_VEHICLE_SPEED] = "Vehicle Speed",
    [PROTOCOL_ID_APS] = "Accelerator Pedal",
    [PROTOCOL_ID_BPS] = "Brake Pedal", 
    [PROTOCOL_ID_STEERING_ANGLE] = "Steering Angle",
    [PROTOCOL_ID_EPS_ERR] = "EPS Error",
    [PROTOCOL_ID_GEAR] = "Gear Status",
    [PROTOCOL_ID_TURN_SIGNAL] = "Turn Signal",
    [PROTOCOL_ID_DOOR_OPEN] = "Door Open",
    [PROTOCOL_ID_SEAT_BELT] = "Seat Belt",
    [PROTOCOL_ID_RADAR] = "Radar Status"
};

// 정적 변수들
static uint32_t tx_time = 0;
static uint32_t tx_counter = 0;
static uint32_t error_check_time = 0;
static uint32_t last_tx_err_count = 0;
static uint32_t last_rx_err_count = 0;
static uint32_t tx_fail_count = 0;
static bool can_initialized = false;
static bool is_tx_mode = false;  // TX 모드 여부를 저장
static bool protocol_initialized = false;


typedef struct {
    uint32_t id;
    can_msg_t msg;
    bool has_data;
    uint16_t rx_count;           // 수신 카운터 (16비트로 축소)
    uint32_t last_protocol_time;  // 마지막 프로토콜 전송 시간
} rx_id_tracker_t;

static rx_id_tracker_t rx_trackers[MAX_RX_IDS];
static uint8_t active_id_count = 0;

// 프로토콜 관련 함수들 구현
bool protocol_init(void)
{
    protocol_initialized = true;
    DEBUG_PRINT("Protocol initialized\r\n");
    return true;
}

bool protocol_map_can_to_data_id(uint32_t can_id, protocol_data_id_t *data_id)
{
    if (data_id == NULL) return false;
    
    for (int i = 0; i < CAN_PROTOCOL_MAP_SIZE; i++) {
        if (can_protocol_map[i].can_id == can_id) {
            *data_id = can_protocol_map[i].data_id;
            return true;
        }
    }
    return false;
}

bool protocol_convert_can_data(uint32_t can_id, const uint8_t *can_data, uint8_t can_length, 
                              uint8_t *protocol_data, uint8_t *protocol_length)
{
    if (can_data == NULL || protocol_data == NULL || protocol_length == NULL) return false;
    
    protocol_data_id_t data_id;
    if (!protocol_map_can_to_data_id(can_id, &data_id)) {
        return false; // 매핑되지 않은 CAN ID
    }
    
    // 데이터 ID별 변환 로직
    switch (data_id) {
        case PROTOCOL_ID_VEHICLE_SPEED:
            // 차량 속도 (WHEEL_SPEEDS 메시지 파싱)
            if (can_length >= 16) {
                wheel_speeds_msg_t wheel_msg;
                if (parse_wheel_speeds_message(can_data, can_length, &wheel_msg)) {
                    // 평균 차량 속도 계산 (kph)
                    float avg_speed = calculate_vehicle_speed(&wheel_msg);
                    // kph를 프로토콜 데이터로 변환 (0-255 범위로 스케일링)
                    uint8_t speed_protocol = (uint8_t)(avg_speed); // 최대 200kph로 정규화
                    if (speed_protocol > 255) speed_protocol = 255;
                    
                    protocol_data[0] = speed_protocol;
                    *protocol_length = 1;
                    
                    // 디버깅 정보 출력
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        vehicle_direction_t direction = get_vehicle_direction(&wheel_msg);
                        all_printf("CAN 0x%lX: Speed=%d kph, Dir=%d, Counter=%d (raw: %d,%d,%d,%d)\r\n", 
                                 can_id, (int)avg_speed, direction, wheel_msg.counter, 
                                 wheel_msg.wheel_speed_1, wheel_msg.wheel_speed_2, 
                                 wheel_msg.wheel_speed_3, wheel_msg.wheel_speed_4);
                        
                        // 상세 디버깅 정보
                        print_wheel_speeds_message(&wheel_msg);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_APS:
            // 엑셀 페달 (ACCELERATOR 메시지 파싱)
            if (can_length >= 8) {
                accelerator_msg_t accel_msg;
                if (parse_accelerator_message(can_data, can_length, &accel_msg)) {
                    // 엑셀 페달 값을 0-100%로 변환
                    float pedal_percent = calculate_accelerator_pedal_percent(&accel_msg);
                    uint8_t pedal_protocol = (uint8_t)pedal_percent;
                    protocol_data[0] = pedal_protocol;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: APS=%d%%, Gear=%d, Counter=%d\r\n", 
                                 can_id, (int)pedal_percent, accel_msg.gear, accel_msg.counter);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_BPS:
            // 브레이크 페달 (BRAKE 메시지 파싱)
            if (can_length >= 8) {
                brake_msg_t brake_msg;
                if (parse_brake_message(can_data, can_length, &brake_msg)) {
                    // 브레이크 위치를 0-100%로 변환
                    float brake_percent = calculate_brake_pedal_percent(&brake_msg);
                    uint8_t brake_protocol = (uint8_t)brake_percent;
                    protocol_data[0] = brake_protocol;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: , bps=%d, BPS=%d%%, Pressed=%d, Counter=%d\r\n", 
                                 can_id, brake_msg.brake_position, (int)brake_percent, brake_msg.brake_pressed, brake_msg.counter);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_STEERING_ANGLE:
            // 스티어링 앵글 (MDPS 메시지 파싱)
            if (can_length >= 18) {
                mdps_msg_t mdps_msg;
                if (parse_mdps_message(can_data, can_length, &mdps_msg)) {
                    // 스티어링 앵글을 -900~900 범위로 변환 (bytes[16-17] 사용)
                    uint16_t raw_angle = (can_data[17] << 8) | can_data[16];
                    float angle_deg = steering_angle_to_deg(raw_angle);
                    int16_t angle_protocol = (int16_t)angle_deg;
                    if (angle_protocol < -900) angle_protocol = -900;
                    if (angle_protocol > 900) angle_protocol = 900;
                    
                    // 각속도 계산 (이전 값과의 차이로 계산, 임시로 0으로 설정)
                    int8_t angular_velocity = 0;  // TODO: 실제 각속도 계산 필요
                    
                    // 스티어링 토크를 -1000~1000 Nm 범위로 변환
                    float torque_nm = ((float)mdps_msg.steering_col_torque * 0.005) - 20.48;
                    int16_t torque_protocol = (int16_t)torque_nm;
                    if (torque_protocol < -1000) torque_protocol = -1000;
                    if (torque_protocol > 1000) torque_protocol = 1000;
                    
                    // 핸들 입력 여부 (토크가 일정 값 이상이면 1, 아니면 0)
                    uint8_t handle_input = (torque_protocol > 100) ? 1 : 0;
                    
                    // 6바이트 프로토콜 데이터 구성 (big-endian 순서)
                    protocol_data[0] = (uint8_t)((angle_protocol >> 8) & 0xFF); // 각도 상위 바이트
                    protocol_data[1] = (uint8_t)(angle_protocol & 0xFF);         // 각도 하위 바이트
                    protocol_data[2] = (uint8_t)angular_velocity;                // 각속도
                    protocol_data[3] = (uint8_t)((torque_protocol >> 8) & 0xFF); // 토크 상위 바이트
                    protocol_data[4] = (uint8_t)(torque_protocol & 0xFF);        // 토크 하위 바이트
                    protocol_data[5] = handle_input;                             // 핸들 입력 여부
                    *protocol_length = 6;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Angle=%d deg, Torque=%d Nm, Handle=%d, Counter=%d (raw_angle=%d, bytes[16]=%02X, bytes[17]=%02X)\r\n", 
                                 can_id, (int)angle_deg, (int)torque_nm, handle_input, mdps_msg.counter, raw_angle, can_data[16], can_data[17]);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_GEAR:
            // 기어 상태 (GEAR 메시지 파싱)
            if (can_length >= 4) {
                gear_msg_t gear_msg;
                if (parse_gear_message(can_data, can_length, &gear_msg)) {
                    // 기어 상태를 프로토콜 형식으로 변환
                    gear_state_t gear_state = get_gear_state(&gear_msg);
                    uint8_t gear_protocol = 0;
                    switch (gear_state) {
                        case GEAR_P: gear_protocol = 0; break;
                        case GEAR_R: gear_protocol = 1; break;
                        case GEAR_N: gear_protocol = 2; break;
                        case GEAR_D: gear_protocol = 3; break;
                        default: gear_protocol = 0; break;
                    }
                    
                    protocol_data[0] = gear_protocol;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Gear=%d (raw=%d), Counter=%d\r\n", 
                                 can_id, gear_protocol, gear_msg.gear, gear_msg.counter);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_EPS_ERR:
            // EPS 에러 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // 0: 정상, 1: 에러
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_TURN_SIGNAL:
            // 방향지시등 상태 (BLINKERS 메시지 파싱)
            if (can_length >= 2) {
                blinkers_msg_t blinkers_msg;
                if (parse_blinkers_message(can_data, can_length, &blinkers_msg)) {
                    // 방향지시등 상태를 비트로 조합
                    uint8_t turn_signal = 0;
                    if (blinkers_msg.left_stalk || blinkers_msg.left_lamp) turn_signal |= 0x01;  // 좌측
                    if (blinkers_msg.right_stalk || blinkers_msg.right_lamp) turn_signal |= 0x02; // 우측
                    
                    protocol_data[0] = turn_signal;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Turn Signal=0x%02X (L=%d, R=%d)\r\n", 
                                 can_id, turn_signal, blinkers_msg.left_stalk, blinkers_msg.right_stalk);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_DOOR_OPEN:
            // 차량 문 열림 상태 (DOORS_SEATBELTS 메시지 파싱)
            if (can_length >= 2) {
                doors_seatbelts_msg_t doors_msg;
                if (parse_doors_seatbelts_message(can_data, can_length, &doors_msg)) {
                    // 문 상태를 비트로 조합
                    uint8_t door_status = 0;
                    if (doors_msg.driver_door) door_status |= 0x01;      // 운전석 문
                    if (doors_msg.passenger_door) door_status |= 0x02;   // 조수석 문
                    if (doors_msg.driver_rear_door) door_status |= 0x04; // 운전석 뒷문
                    if (doors_msg.passenger_rear_door) door_status |= 0x08; // 조수석 뒷문
                    
                    protocol_data[0] = door_status;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Door Status=0x%02X (Driver=%d, Passenger=%d)\r\n", 
                                 can_id, door_status, doors_msg.driver_door, doors_msg.passenger_door);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_SEAT_BELT:
            // 안전벨트 상태 (DOORS_SEATBELTS 메시지 파싱)
            if (can_length >= 2) {
                doors_seatbelts_msg_t seatbelt_msg;
                if (parse_doors_seatbelts_message(can_data, can_length, &seatbelt_msg)) {
                    // 안전벨트 상태를 비트로 조합
                    uint8_t seatbelt_status = 0;
                    if (seatbelt_msg.driver_seatbelt) seatbelt_status |= 0x01;      // 운전석 미착용
                    if (seatbelt_msg.passenger_seatbelt) seatbelt_status |= 0x02;   // 조수석 미착용
                    
                    protocol_data[0] = seatbelt_status;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Seatbelt Status=0x%02X (Driver=%d, Passenger=%d)\r\n", 
                                 can_id, seatbelt_status, seatbelt_msg.driver_seatbelt, seatbelt_msg.passenger_seatbelt);
                    }
                    
                    return true;
                }
            }
            break;
            
        case PROTOCOL_ID_RADAR:
            // 블라인드스팟 상태 (BLINDSPOTS_REAR_CORNERS 메시지 파싱)
            if (can_length >= 24) {
                blindspots_rear_corners_msg_t blindspots_msg;
                if (parse_blindspots_rear_corners_message(can_data, can_length, &blindspots_msg)) {
                    // 블라인드스팟 상태를 비트로 조합
                    uint8_t blindspots_status = 0;
                    if (blindspots_msg.left_blocked) blindspots_status |= 0x01;  // 좌측 차단됨
                    if (blindspots_msg.right_blocked) blindspots_status |= 0x02; // 우측 차단됨
                    if (blindspots_msg.collision_avoidance_active) blindspots_status |= 0x04; // 충돌 회피 활성화
                    if (blindspots_msg.fl_indicator_alt) blindspots_status |= 0x08; // 좌측 전방 표시등
                    if (blindspots_msg.fr_indicator_alt) blindspots_status |= 0x10; // 우측 전방 표시등
                    
                    protocol_data[0] = blindspots_status;
                    *protocol_length = 1;
                    
                    if (DEBUG_CAN && is_debug_can_id(can_id)) {
                        // CAN 메시지 정보 출력
                        all_printf("CAN MSG: ID=0x%lX, DLC=%d, Data: ", can_id, can_length);
                        for (int i = 0; i < can_length && i < 64; i++) {
                            all_printf("%02X ", can_data[i]);
                        }
                        all_printf("\r\n");
                        
                        all_printf("CAN 0x%lX: Blindspots Status=0x%02X (L=%d, R=%d, CA=%d, FL=%d, FR=%d)\r\n", 
                                 can_id, blindspots_status, 
                                 blindspots_msg.left_blocked, 
                                 blindspots_msg.right_blocked,
                                 blindspots_msg.collision_avoidance_active,
                                 blindspots_msg.fl_indicator_alt,
                                 blindspots_msg.fr_indicator_alt);
                    }
                    
                    return true;
                }
            }
            break;
            
        default:
            return false;
    }
    
    return false;
}

uint8_t protocol_calculate_checksum(uint8_t data_id, uint8_t length, const uint8_t *data)
{
    uint16_t sum = data_id + length;
    
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    
    return (uint8_t)(sum & 0xFF);  // 하위 1바이트만 반환
}

bool protocol_send_frame(protocol_data_id_t data_id, const uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0 || length > PROTOCOL_MAX_DATA) {
        return false;
    }
    
    // DEBUG 모드일 때는 프로토콜 프레임 출력하지 않음
    if (current_can_mode == CAN_DEBUG_MODE) {
        return true;  // 성공으로 처리하지만 실제 전송은 하지 않음
    }
    
    protocol_frame_t frame;
    
    // 프레임 구성
    frame.stx = PROTOCOL_STX;
    frame.data_id = (uint8_t)data_id;
    frame.length = length;
    
    // 데이터 복사
    for (int i = 0; i < length; i++) {
        frame.data[i] = data[i];
    }
    
    // 체크섬 계산
    frame.checksum = protocol_calculate_checksum(frame.data_id, frame.length, frame.data);
    frame.etx = PROTOCOL_ETX;
    
    // UART로 프레임 송신
    uint8_t frame_bytes[3 + PROTOCOL_MAX_DATA + 2]; // STX + ID + Length + Data + Checksum + ETX
    int frame_size = 0;
    
    frame_bytes[frame_size++] = frame.stx;
    frame_bytes[frame_size++] = frame.data_id;
    frame_bytes[frame_size++] = frame.length;
    
    for (int i = 0; i < length; i++) {
        frame_bytes[frame_size++] = frame.data[i];
    }
    
    frame_bytes[frame_size++] = frame.checksum;
    frame_bytes[frame_size++] = frame.etx;
    
    // UART 채널을 통해 송신
    uint32_t sent = uartWrite(HW_UART_CH_DEBUG, frame_bytes, frame_size);
    (void)uartWrite(HW_UART_CH_EXT, frame_bytes, frame_size);
    
    if (sent == frame_size) {
        // 디버깅 출력 형태 결정
#if DEBUG_CAN_PROTOCOL
        // 상세한 디버깅 정보 출력
        all_printf("TX: %02X | Data ID: %02X | Length: %02X | Data: ", 
                   frame.stx, frame.data_id, frame.length);
        for (int i = 0; i < length; i++) {
            all_printf("%02X ", frame.data[i]);
        }
        all_printf("| Checksum: %02X | ETX: %02X\r\n", frame.checksum, frame.etx);
#else
        // 순수 바이너리 데이터만 16진수로 출력 (한 줄로 정리)
        all_printf("%02X %02X %02X ", frame.stx, frame.data_id, frame.length);
        for (int i = 0; i < length; i++) {
            all_printf("%02X ", frame.data[i]);
        }
        all_printf("%02X %02X\r\n", frame.checksum, frame.etx);
#endif
        return true;
    } else {
        DEBUG_PRINT("Protocol TX Failed: sent=%lu, expected=%d\r\n", sent, frame_size);
        return false;
    }
}

// CAN 매니저 초기화
bool can_manager_init(bool tx_mode)
{
    is_tx_mode = tx_mode;  // TX 모드 저장
    
    canInit();
    
    CanMode_t can_mode;
    
    // TX 모드는 NORMAL, RX 모드는 MONITOR
    if (is_tx_mode)
    {
        can_mode = CAN_NORMAL;  // 송신 가능 모드
        DEBUG_PRINT("CAN mode: NORMAL (TX capable)\r\n");
    }
    else
    {
        can_mode = CAN_NORMAL;
        //can_mode = CAN_MONITOR; // 수신 전용 모드
        DEBUG_PRINT("CAN mode: MONITOR (RX only)\r\n");
    }
    
    if (canOpen(_DEF_CAN1, can_mode, CAN_FD_BRS, CAN_500K, CAN_2M) == false)
    {
        DEBUG_PRINT("CAN init failed\r\n");
        return false;
    }
    
    DEBUG_PRINT("CAN initialized: 500K/2M, CAN-FD BRS\r\n");
    
    // 프로토콜 초기화
    if (!protocol_init()) {
        DEBUG_PRINT("Protocol init failed\r\n");
        return false;
    }
    
    // 모드별 초기화
    if (is_tx_mode)
    {
        can_tx_init();
    }
    else
    {
        // 새로운 CAN 모드에 따른 초기화
        switch (current_can_mode)
        {
            case CAN_ALL_LISTEN:
                can_all_listen_init();
                break;
            case CAN_FILTER_LISTEN:
                can_filter_listen_init();
                break;
            case CAN_RELEASE:
                can_rx_init();  // 기존 프로토콜 변환 모드
                break;
            case CAN_DEBUG_MODE:
                can_debug_mode_init();
                break;
            case CAN_SCAN_MODE:
                can_scan_mode_init();
                break;
            default:
                can_rx_init();  // 기본값
                break;
        }
    }
    
    can_initialized = true;
    return true;
}

// CAN 매니저 메인 처리
void can_manager_process(void)
{
    if (!can_initialized) return;
    
    // 에러 체크 (공통)
    can_error_check();
    
    
    // TX 처리 (TX 모드에서만)
    if (is_tx_mode)
    {
        can_tx_process();
    }
    else
    {
        // 새로운 CAN 모드에 따른 처리
        switch (current_can_mode)
        {
            case CAN_ALL_LISTEN:
                can_all_listen_process();
                break;
            case CAN_FILTER_LISTEN:
                can_filter_listen_process();
                break;
            case CAN_RELEASE:
                can_rx_process();  // 기존 프로토콜 변환 모드
                break;
            case CAN_DEBUG_MODE:
                can_debug_mode_process();
                break;
            case CAN_SCAN_MODE:
                can_scan_mode_process();
                break;
            default:
                can_rx_process();  // 기본값
                break;
        }
    }
}

// CAN TX 초기화
bool can_tx_init(void)
{
    DEBUG_PRINT("CAN TEST MODE: TX (Transmitter)\r\n");
    DEBUG_PRINT("Will send %d sample CAN FD messages every 10ms\r\n", SAMPLE_MSG_COUNT);
    
    tx_time = millis();
    tx_counter = 0;
    tx_fail_count = 0;
    
    return true;
}

// CAN TX 처리
void can_tx_process(void)
{
    if (millis() - tx_time >= 10)  // 10ms마다 송신
    {
        tx_time = millis();
        
        // 샘플 메시지 중 하나를 순차적으로 선택
        uint32_t msg_index = tx_counter % SAMPLE_MSG_COUNT;
        const sample_can_msg_t *sample = &sample_messages[msg_index];
        
        can_msg_t tx_msg;
        canMsgInit(&tx_msg, CAN_FD_BRS, CAN_STD, sample->dlc);
        tx_msg.id = sample->id;
        
        // 샘플 데이터 복사 (카운터 추가로 변화 표시)
        uint8_t data_length = canGetLen(sample->dlc);
        for (int i = 0; i < data_length; i++)
        {
            if (i < 4)  // 처음 4바이트에 변화하는 데이터 추가
            {
                tx_msg.data[i] = sample->data[i] ^ (tx_counter & 0xFF);
            }
            else
            {
                tx_msg.data[i] = sample->data[i];
            }
        }
        
        if (canMsgWrite(_DEF_CAN1, &tx_msg, 10))
        {
            DEBUG_PRINT("CAN TX: ID=0x%03lX, DLC=%d, Data=", tx_msg.id, data_length);
            for (int i = 0; i < data_length; i++)
            {
                DEBUG_PRINT("%02X ", tx_msg.data[i]);
            }
            DEBUG_PRINT("(Seq=%lu)\r\n", tx_counter);
            ledToggle(HW_LED_CH_TX);
            tx_fail_count = 0; // 송신 성공 시 실패 카운터 리셋
        }
        else
        {
            tx_fail_count++;
            DEBUG_PRINT("CAN TX Failed! ID=0x%03lX (Fail count: %lu)\r\n", sample->id, tx_fail_count);
            
            // 연속 실패 시 복구 시도
            if (tx_fail_count >= 3)
            {
                DEBUG_PRINT("Multiple TX failures detected. Attempting CAN recovery...\r\n");
                can_error_recovery();
                tx_fail_count = 0;
            }
        }
        
        tx_counter++;
    }
}

// CAN RX 초기화
bool can_rx_init(void)
{
    all_printf("=== CAN RELEASE MODE INITIALIZED ===\r\n");
    all_printf("CAN TEST MODE: RX (Receiver)\r\n");
    all_printf("Protocol output enabled via UART\r\n");
    all_printf("==================================\r\n");
    
    rx_display_time = millis();
    // Initialize trackers
    for (int i = 0; i < MAX_RX_IDS; i++) {
        rx_trackers[i].id = 0;
        rx_trackers[i].has_data = false;
        rx_trackers[i].rx_count = 0;
        rx_trackers[i].last_protocol_time = 0;
    }
    active_id_count = 0;
    
    return true;
}

// CAN RX 처리
void can_rx_process(void)
{
    // 새로운 메시지 수신 시 ID별로 저장
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            // 프로토콜 매핑 확인
            protocol_data_id_t data_id;
            if (protocol_map_can_to_data_id(rx_msg.id, &data_id))
            {
                // 0xA0 (WHEEL_SPEEDS) 메시지 특별 처리 (프로토콜 모드에서는 출력 안함)
                //if (rx_msg.id == CAN_ID_WHEEL_SPEEDS && rx_msg.length >= 16) 
                {
                    wheel_speeds_msg_t wheel_msg;
                    if (parse_wheel_speeds_message(rx_msg.data, rx_msg.length, &wheel_msg)) {
                        // 프로토콜 모드에서는 디버깅 출력 안함
                        // float avg_speed = calculate_vehicle_speed(&wheel_msg);
                        // vehicle_direction_t direction = get_vehicle_direction(&wheel_msg);
                        // all_printf("WHEEL_SPEEDS: Speed=%.1f kph, Dir=%d, Counter=%d\n", 
                        //          avg_speed, direction, wheel_msg.counter);
                        
                        // 상세 정보 출력 (선택적)
                        if (DEBUG_CAN && is_debug_can_id(rx_msg.id)) {
                            print_wheel_speeds_message(&wheel_msg);
                        }
                    }
                }
                
                // 해당 ID의 추적기 찾기
                int tracker_index = -1;
                for (int i = 0; i < MAX_RX_IDS; i++) {
                    if (rx_trackers[i].has_data && rx_trackers[i].id == rx_msg.id) {
                        tracker_index = i;
                        break;
                    }
                }
                
                // 새로운 ID라면 추적기 할당
                if (tracker_index == -1) {
                    for (int i = 0; i < MAX_RX_IDS; i++) {
                        if (!rx_trackers[i].has_data) {
                            tracker_index = i;
                            rx_trackers[i].id = rx_msg.id;
                            rx_trackers[i].has_data = true;
                            rx_trackers[i].rx_count = 0;
                            rx_trackers[i].last_protocol_time = 0;
                            break;
                        }
                    }
                }
                
                // 추적기에 데이터 저장
                if (tracker_index != -1) {
                    rx_trackers[tracker_index].msg = rx_msg;
                    rx_trackers[tracker_index].rx_count++;
                }
            }
            // 매핑되지 않은 CAN ID는 무시 (프로토콜 출력하지 않음)
            
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // 주기별 프로토콜 프레임 전송 체크
    uint32_t current_time = millis();
    for (int i = 0; i < MAX_RX_IDS; i++) {
        if (rx_trackers[i].has_data) {
            protocol_data_id_t data_id;
            if (protocol_map_can_to_data_id(rx_trackers[i].id, &data_id)) {
                // 해당 ID의 전송 주기 확인
                uint32_t period_ms = 0;
                for (int j = 0; j < CAN_PROTOCOL_MAP_SIZE; j++) {
                    if (can_protocol_map[j].can_id == rx_trackers[i].id) {
                        period_ms = can_protocol_map[j].period_ms;
                        break;
                    }
                }
                
                // 주기 체크 및 프로토콜 전송
                if (current_time - rx_trackers[i].last_protocol_time >= period_ms) {
                    // DEBUG_CAN이 활성화된 경우 디버그 목록의 ID만 처리
#if DEBUG_CAN
                    if (!is_debug_can_id(rx_trackers[i].id)) {
                        rx_trackers[i].last_protocol_time = current_time;
                        continue;  // 디버그 목록에 없는 ID는 건너뛰기
                    }
#endif
                    
                    // 프로토콜 데이터 변환
                    uint8_t protocol_data[PROTOCOL_MAX_DATA];
                    uint8_t protocol_length;
                    
                    if (protocol_convert_can_data(rx_trackers[i].id, rx_trackers[i].msg.data, 
                                                rx_trackers[i].msg.length, protocol_data, &protocol_length)) {
                        // 프로토콜 프레임 전송
                        if (protocol_send_frame(data_id, protocol_data, protocol_length)) {
                            rx_trackers[i].last_protocol_time = current_time;
                        }
                    }
                }
            }
        }
    }
    
    // 디버깅 출력 (DEBUG 플래그가 설정된 경우에만)
#if DEBUG_CAN_PROTOCOL
    // 1초마다 모든 ID의 최신 메시지 출력 (디버깅용)
    if (millis() - rx_display_time >= 1000)
    {
        rx_display_time = millis();
        
        bool has_any_data = false;
        for (int i = 0; i < MAX_RX_IDS; i++) {
            if (rx_trackers[i].has_data) {
                has_any_data = true;
                break;
            }
        }
        
        if (has_any_data) {
            DEBUG_PRINT("=== CAN FD RX Summary (1sec) ===\r\n");
            for (int i = 0; i < MAX_RX_IDS; i++) {
                if (rx_trackers[i].has_data) {
                    protocol_data_id_t data_id;
                    if (protocol_map_can_to_data_id(rx_trackers[i].id, &data_id)) {
                        DEBUG_PRINT("ID=0x%03lX->0x%02X, Count=%u, DLC=%d\r\n", 
                                   rx_trackers[i].id, (uint8_t)data_id, 
                                   rx_trackers[i].rx_count, rx_trackers[i].msg.length);
                    } else {
                        DEBUG_PRINT("ID=0x%03lX (unmapped), Count=%u, DLC=%d\r\n", 
                                   rx_trackers[i].id, rx_trackers[i].rx_count, rx_trackers[i].msg.length);
                    }
                }
            }
            DEBUG_PRINT("================================\r\n");
            
            // 카운터만 리셋하고 데이터는 유지 (다음 주기를 위해)
            for (int i = 0; i < MAX_RX_IDS; i++) {
                rx_trackers[i].rx_count = 0;
            }
        }
    }
#endif
}

// CAN 에러 체크
void can_error_check(void)
{
    // 500ms마다 에러 상태 체크
    if (millis() - error_check_time >= 500)
    {
        error_check_time = millis();
        
        // 에러 상태 업데이트 (Bus-Off 상태 자동 복구 포함)
        canUpdate();
        
        uint32_t can_error = canGetError(_DEF_CAN1);
        uint16_t tx_err_count = canGetTxErrCount(_DEF_CAN1);
        uint16_t rx_err_count = canGetRxErrCount(_DEF_CAN1);
        
        // 에러 카운터 증가 감지
        if (tx_err_count != last_tx_err_count || rx_err_count != last_rx_err_count)
        {
            DEBUG_PRINT("CAN Err: TX=%d, RX=%d, Code=0x%08lX\r\n", 
                      tx_err_count, rx_err_count, can_error);
            last_tx_err_count = tx_err_count;
            last_rx_err_count = rx_err_count;
        }
        
        // Bus-Off 또는 Error Passive 상태 감지 시 복구 시도
        if (can_error & (CAN_ERR_BUS_OFF | CAN_ERR_PASSIVE))
        {
            DEBUG_PRINT("CAN Error detected! Attempting recovery...\r\n");
            can_error_recovery();
        }
        
        // 에러 카운터가 너무 높으면 경고
        if (tx_err_count > 100 || rx_err_count > 100)
        {
            DEBUG_PRINT("WARNING: High error count detected!\r\n");
            
            // 에러 카운터가 매우 높으면 강제 복구 시도
            if (tx_err_count > 200 || rx_err_count > 200)
            {
                DEBUG_PRINT("CRITICAL: Error count too high, forcing recovery!\r\n");
                can_error_recovery();
            }
        }
        
        // Error Warning 상태가 지속되면 주기적 복구
        if (can_error & CAN_ERR_WARNING)
        {
            static uint32_t warning_count = 0;
            warning_count++;
            
            if (warning_count >= 5)  // 5번 연속 Warning이면 복구 시도
            {
                DEBUG_PRINT("Persistent warning detected, attempting recovery...\r\n");
                can_error_recovery();
                warning_count = 0;
            }
        }
    }
}

// CAN 에러 복구
void can_error_recovery(void)
{
    DEBUG_PRINT("Performing CAN recovery...\r\n");
    
    canRecovery(_DEF_CAN1);
    
    // 복구 후 잠시 대기
    delay(10);
    
    DEBUG_PRINT("CAN recovery completed\r\n");
}

// ===============================================
// 새로운 CAN 모드 관련 함수들
// ===============================================

// 모든 CAN ID 수신 초기화
bool can_all_listen_init(void)
{
    DEBUG_PRINT("CAN ALL LISTEN MODE: 모든 CAN ID 수신\r\n");
    DEBUG_PRINT("출력 주기: %lu ms\r\n", all_can_output_period);
    DEBUG_PRINT("타임아웃 주기: %lu ms\r\n", all_can_timeout_period);
    
    // 모든 CAN ID 추적기 초기화
    for (int i = 0; i < MAX_ALL_CAN_IDS; i++) {
        all_can_trackers[i].id = 0;
        all_can_trackers[i].has_data = false;
        all_can_trackers[i].rx_count = 0;
        all_can_trackers[i].last_output_time = 0;
        all_can_trackers[i].last_rx_time = 0;
    }
    all_can_active_count = 0;
    
    return true;
}

// 모든 CAN ID 수신 처리
void can_all_listen_process(void)
{
    uint32_t current_time = millis();
    
    // 새로운 메시지 수신 시 모든 ID 저장
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            // 해당 ID의 추적기 찾기
            int tracker_index = -1;
            for (int i = 0; i < MAX_ALL_CAN_IDS; i++) {
                if (all_can_trackers[i].has_data && all_can_trackers[i].id == rx_msg.id) {
                    tracker_index = i;
                    break;
                }
            }
            
            // 새로운 ID라면 추적기 할당
            if (tracker_index == -1) {
                for (int i = 0; i < MAX_ALL_CAN_IDS; i++) {
                    if (!all_can_trackers[i].has_data) {
                        tracker_index = i;
                        all_can_trackers[i].id = rx_msg.id;
                        all_can_trackers[i].has_data = true;
                        all_can_trackers[i].rx_count = 0;
                        all_can_trackers[i].last_output_time = 0;
                        all_can_trackers[i].last_rx_time = current_time;
                        all_can_active_count++;
                        DEBUG_PRINT("NEW CAN ID: 0x%lX (Total: %d)\r\n", rx_msg.id, all_can_active_count);
                        break;
                    }
                }
            }
            
            // 추적기에 데이터 저장
            if (tracker_index != -1) {
                all_can_trackers[tracker_index].msg = rx_msg;
                all_can_trackers[tracker_index].rx_count++;
                all_can_trackers[tracker_index].last_rx_time = current_time;
            }
            
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // 타임아웃 체크 및 삭제
    for (int i = 0; i < MAX_ALL_CAN_IDS; i++) {
        if (all_can_trackers[i].has_data) {
            if (current_time - all_can_trackers[i].last_rx_time >= all_can_timeout_period) {
                DEBUG_PRINT("TIMEOUT CAN ID: 0x%lX (Count: %u)\r\n", 
                           all_can_trackers[i].id, all_can_trackers[i].rx_count);
                // 타임아웃된 ID 삭제
                all_can_trackers[i].id = 0;
                all_can_trackers[i].has_data = false;
                all_can_trackers[i].rx_count = 0;
                all_can_trackers[i].last_output_time = 0;
                all_can_trackers[i].last_rx_time = 0;
                all_can_active_count--;
            }
        }
    }
    
    // 주기별 출력 체크
    for (int i = 0; i < MAX_ALL_CAN_IDS; i++) {
        if (all_can_trackers[i].has_data) {
            if (current_time - all_can_trackers[i].last_output_time >= all_can_output_period) {
                can_output_raw_message(all_can_trackers[i].id, &all_can_trackers[i].msg);
                all_can_trackers[i].last_output_time = current_time;
            }
        }
    }
}

// 필터링된 CAN ID 수신 초기화
bool can_filter_listen_init(void)
{
    all_printf("=== CAN FILTER LISTEN MODE INITIALIZED ===\r\n");
    all_printf("모든 CAN ID 수신 (매핑된 ID는 설정 주기, 나머지는 1000ms)\r\n");
    all_printf("매핑된 ID는 프로토콜 설명과 함께 STX/ETX 형식으로 출력됩니다.\r\n");
    all_printf("==========================================\r\n");
    
    // 기존 RX 초기화와 동일하지만 프로토콜 변환 없이 출력
    rx_display_time = millis();
    for (int i = 0; i < MAX_RX_IDS; i++) {
        rx_trackers[i].id = 0;
        rx_trackers[i].has_data = false;
        rx_trackers[i].rx_count = 0;
        rx_trackers[i].last_protocol_time = 0;
    }
    active_id_count = 0;
    
    return true;
}

// 필터링된 CAN ID 수신 처리
void can_filter_listen_process(void)
{
    // 새로운 메시지 수신 시 모든 ID 저장 (매핑 여부와 관계없이)
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            // 해당 ID의 추적기 찾기
            int tracker_index = -1;
            for (int i = 0; i < MAX_RX_IDS; i++) {
                if (rx_trackers[i].has_data && rx_trackers[i].id == rx_msg.id) {
                    tracker_index = i;
                    break;
                }
            }
            
            // 새로운 ID라면 추적기 할당
            if (tracker_index == -1) {
                for (int i = 0; i < MAX_RX_IDS; i++) {
                    if (!rx_trackers[i].has_data) {
                        tracker_index = i;
                        rx_trackers[i].id = rx_msg.id;
                        rx_trackers[i].has_data = true;
                        rx_trackers[i].rx_count = 0;
                        rx_trackers[i].last_protocol_time = 0;
                        break;
                    }
                }
            }
            
            // 추적기에 데이터 저장
            if (tracker_index != -1) {
                rx_trackers[tracker_index].msg = rx_msg;
                rx_trackers[tracker_index].rx_count++;
            }
            
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // 주기별 출력 체크
    uint32_t current_time = millis();
    for (int i = 0; i < MAX_RX_IDS; i++) {
        if (rx_trackers[i].has_data) {
            protocol_data_id_t data_id;
            bool is_mapped = protocol_map_can_to_data_id(rx_trackers[i].id, &data_id);
            
            if (is_mapped) {
                // 매핑된 ID: 설정된 주기로 출력
                uint32_t period_ms = 0;
                for (int j = 0; j < CAN_PROTOCOL_MAP_SIZE; j++) {
                    if (can_protocol_map[j].can_id == rx_trackers[i].id) {
                        period_ms = can_protocol_map[j].period_ms;
                        break;
                    }
                }
                
                if (current_time - rx_trackers[i].last_protocol_time >= period_ms) {
                    // 프로토콜 데이터 변환 및 설명과 함께 출력
                    uint8_t protocol_data[PROTOCOL_MAX_DATA];
                    uint8_t protocol_length;
                    
                    if (protocol_convert_can_data(rx_trackers[i].id, rx_trackers[i].msg.data, 
                                                rx_trackers[i].msg.length, protocol_data, &protocol_length)) {
                        // CAN 메시지 먼저 출력
                        can_output_raw_message(rx_trackers[i].id, &rx_trackers[i].msg);
                        // 그 다음 프로토콜 메시지 출력
                        can_output_protocol_message(rx_trackers[i].id, &rx_trackers[i].msg, data_id, protocol_data, protocol_length);
                    } else {
                        // 변환 실패 시 원본 메시지 출력
                        can_output_raw_message(rx_trackers[i].id, &rx_trackers[i].msg);
                    }
                    rx_trackers[i].last_protocol_time = current_time;
                }
            } else {
                // 매핑되지 않은 ID: 기본 주기(1000ms)로 출력
                if (current_time - rx_trackers[i].last_protocol_time >= 1000) {
                    can_output_raw_message(rx_trackers[i].id, &rx_trackers[i].msg);
                    rx_trackers[i].last_protocol_time = current_time;
                }
            }
        }
    }
}

// CAN 메시지를 UART로 출력하는 함수
void can_output_raw_message(uint32_t can_id, const can_msg_t *msg)
{
#if DEBUG_CAN
    // DEBUG_CAN이 1일 때는 특정 CAN ID만 출력
    if (!is_debug_can_id(can_id)) {
        all_printf("FILTERED: 0x%lX (not in debug list)\r\n", can_id);
        return;  // 디버그 목록에 없는 ID는 출력하지 않음
    }
    all_printf("ALLOWED: 0x%lX (in debug list)\r\n", can_id);
#endif

#if DEBUG_CAN
    // DEBUG_CAN이 1일 때만 출력
    // CAN ID와 Data를 분리해서 표시
    all_printf("[CAN] ID: 0x%lX | Data: ", can_id);
    
    // Raw DLC 값을 직접 사용 (canGetLen 함수의 버그 우회)
    uint8_t data_length = msg->length;
    if (data_length > 64) data_length = 64;  // 안전장치
    
    for (int i = 0; i < data_length; i++) {
        all_printf("%02X ", msg->data[i]);
    }
    all_printf("| DLC: %d\r\n", data_length);
#endif
}

// 프로토콜 메시지를 설명과 함께 UART로 출력하는 함수
void can_output_protocol_message(uint32_t can_id, const can_msg_t *msg, protocol_data_id_t data_id, 
                                const uint8_t *protocol_data, uint8_t protocol_length)
{
#if DEBUG_CAN
    // DEBUG_CAN이 1일 때는 특정 CAN ID만 출력
    if (!is_debug_can_id(can_id)) {
        return;  // 디버그 목록에 없는 ID는 출력하지 않음
    }
#endif

#if DEBUG_CAN
    // DEBUG_CAN이 1일 때만 출력
    // 프로토콜 설명 가져오기
    const char* description = "Unknown";
    if (data_id < sizeof(protocol_descriptions) / sizeof(protocol_descriptions[0])) {
        description = protocol_descriptions[data_id];
    }
    
    // 프로토콜 프레임 구성
    all_printf("[PROTOCOL] ID: 0x%lX | %s | ", can_id, description);
    all_printf("STX: 02 | Data ID: %02X | Length: %02X | Data: ", (uint8_t)data_id, protocol_length);
    
    for (int i = 0; i < protocol_length; i++) {
        all_printf("%02X ", protocol_data[i]);
    }
    
    // 체크섬 계산 및 출력
    uint8_t checksum = protocol_calculate_checksum((uint8_t)data_id, protocol_length, protocol_data);
    all_printf("| Checksum: %02X | ETX: 03\r\n", checksum);
#endif
}

// 디버깅 모드 초기화
bool can_debug_mode_init(void)
{
    DEBUG_PRINT("CAN DEBUG MODE: CAN과 무관하게 UART만 출력\r\n");
    DEBUG_PRINT("출력 주기: 1000ms\r\n");
    
    debug_output_time = millis();
    debug_counter = 0;
    
    return true;
}

// 디버깅 모드 처리
void can_debug_mode_process(void)
{
    // 1초마다 디버깅 메시지 출력
    if (millis() - debug_output_time >= 1000)
    {
        debug_output_time = millis();
        debug_counter++;
        
        all_printf("DEBUG MODE: Counter=%lu, Time=%lu ms\r\n", debug_counter, millis());
        all_printf("System Status: CAN=%s, UART=%s, USB=%s\r\n", 
                   can_initialized ? "OK" : "FAIL",
                   uartIsOpen(HW_UART_CH_DEBUG) ? "OK" : "FAIL",
                   cdcIsConnect() ? "OK" : "FAIL");
        
        // LED 토글로 동작 표시
        ledToggle(HW_LED_CH_DEBUG);
    }
}

// CAN 모드 전환
void can_mode_switch(void)
{
    // 다음 모드로 전환
    current_can_mode = (current_can_mode + 1) % 5;  // 0~4 모드 순환
    
    // 모드별 초기화
    switch (current_can_mode)
    {
        case CAN_ALL_LISTEN:
            can_all_listen_init();
            break;
        case CAN_FILTER_LISTEN:
            can_filter_listen_init();
            break;
        case CAN_RELEASE:
            can_rx_init();
            break;
        case CAN_DEBUG_MODE:
            can_debug_mode_init();
            break;
        case CAN_SCAN_MODE:
            can_scan_mode_init();
            break;
        default:
            can_rx_init();  // 기본값
            break;
    }
    
    // 모드 정보 출력
    can_print_mode_info();
}

// CAN 모드 정보 출력
void can_print_mode_info(void)
{
    all_printf("\r\n=== CAN MODE SWITCHED ===\r\n");
    
    switch (current_can_mode)
    {
        case CAN_ALL_LISTEN:
            all_printf("Mode 0: ALL LISTEN\r\n");
            all_printf("기능: 모든 CAN ID 수신\r\n");
            all_printf("출력: CAN ID와 Data를 분리해서 표시\r\n");
            all_printf("주기: 1000ms\r\n");
            break;
        case CAN_FILTER_LISTEN:
            all_printf("Mode 1: FILTER LISTEN\r\n");
            all_printf("기능: 모든 CAN ID 수신\r\n");
            all_printf("출력: 매핑된 ID는 프로토콜 설명과 STX/ETX 형식, 나머지는 원본 형식\r\n");
            all_printf("주기: 매핑된 ID는 설정 주기, 나머지는 1000ms\r\n");
            break;
        case CAN_RELEASE:
            all_printf("Mode 2: RELEASE\r\n");
            all_printf("기능: 필터링된 CAN ID를 프로토콜로 변환\r\n");
            all_printf("출력: STX + ID + Length + Data + Checksum + ETX\r\n");
            all_printf("주기: ID별 설정된 주기\r\n");
            break;
        case CAN_DEBUG_MODE:
            all_printf("Mode 3: DEBUG MODE\r\n");
            all_printf("기능: CAN과 무관하게 UART만 출력\r\n");
            all_printf("출력: 시스템 상태 및 디버깅 정보\r\n");
            all_printf("주기: 1000ms\r\n");
            break;
        case CAN_SCAN_MODE:
            all_printf("Mode 4: SCAN MODE\r\n");
            all_printf("기능: CAN ID만 확인 (메모리 최소화)\r\n");
            all_printf("출력: CAN ID 목록, DLC, 수신 카운터\r\n");
            all_printf("주기: 1000ms, 타임아웃: 1000ms\r\n");
            break;
    }
    
    all_printf("========================\r\n\r\n");
}

// SCAN 모드 초기화
bool can_scan_mode_init(void)
{
    DEBUG_PRINT("CAN SCAN MODE: CAN ID만 확인 (메모리 최소화)\r\n");
    DEBUG_PRINT("출력 주기: %lu ms\r\n", scan_output_period);
    DEBUG_PRINT("타임아웃 주기: %lu ms\r\n", scan_timeout_period);
    DEBUG_PRINT("최대 추적 ID 수: %d\r\n", MAX_SCAN_IDS);
    
    // SCAN 모드 추적기 초기화
    for (int i = 0; i < MAX_SCAN_IDS; i++) {
        scan_trackers[i].id = 0;
        scan_trackers[i].dlc = 0;
        scan_trackers[i].has_data = false;
        scan_trackers[i].rx_count = 0;
        scan_trackers[i].last_rx_time = 0;
    }
    scan_active_count = 0;
    
    return true;
}

// SCAN 모드 처리
void can_scan_mode_process(void)
{
    uint32_t current_time = millis();
    
    // 새로운 메시지 수신 시 ID만 저장
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            // 해당 ID의 추적기 찾기
            int tracker_index = -1;
            for (int i = 0; i < MAX_SCAN_IDS; i++) {
                if (scan_trackers[i].has_data && scan_trackers[i].id == rx_msg.id) {
                    tracker_index = i;
                    break;
                }
            }
            
            // 새로운 ID라면 추적기 할당
            if (tracker_index == -1) {
                for (int i = 0; i < MAX_SCAN_IDS; i++) {
                    if (!scan_trackers[i].has_data) {
                        tracker_index = i;
                        scan_trackers[i].id = rx_msg.id;
                        scan_trackers[i].dlc = rx_msg.length;
                        scan_trackers[i].has_data = true;
                        scan_trackers[i].rx_count = 0;
                        scan_trackers[i].last_rx_time = current_time;
                        scan_active_count++;
                        DEBUG_PRINT("SCAN NEW ID: 0x%lX DLC:%d (Total: %d)\r\n", rx_msg.id, rx_msg.length, scan_active_count);
                        break;
                    }
                }
            }
            
            // 추적기에 카운터와 DLC 업데이트
            if (tracker_index != -1) {
                scan_trackers[tracker_index].rx_count++;
                scan_trackers[tracker_index].dlc = rx_msg.length;  // 최신 DLC로 업데이트
                scan_trackers[tracker_index].last_rx_time = current_time;
            }
            
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // 타임아웃 체크 및 삭제
    for (int i = 0; i < MAX_SCAN_IDS; i++) {
        if (scan_trackers[i].has_data) {
            if (current_time - scan_trackers[i].last_rx_time >= scan_timeout_period) {
                DEBUG_PRINT("SCAN TIMEOUT ID: 0x%lX DLC:%d (Count: %u)\r\n", 
                           scan_trackers[i].id, scan_trackers[i].dlc, scan_trackers[i].rx_count);
                // 타임아웃된 ID 삭제
                scan_trackers[i].id = 0;
                scan_trackers[i].dlc = 0;
                scan_trackers[i].has_data = false;
                scan_trackers[i].rx_count = 0;
                scan_trackers[i].last_rx_time = 0;
                scan_active_count--;
            }
        }
    }
    
    // 주기별 출력 체크 (1초 주기)
    static uint32_t last_scan_output = 0;
    if (current_time - last_scan_output >= scan_output_period) {
        last_scan_output = current_time;
        
        // 1초 동안 수신된 ID만 출력
        int active_count = 0;
        for (int i = 0; i < MAX_SCAN_IDS; i++) {
            if (scan_trackers[i].has_data) {
                active_count++;
            }
        }
        
        if (active_count > 0) {
            all_printf("=== CAN SCAN (1s) ===\r\n");
            all_printf("Active IDs: %d\r\n", active_count);
            
            for (int i = 0; i < MAX_SCAN_IDS; i++) {
                if (scan_trackers[i].has_data) {
                    all_printf("ID: 0x%lX, DLC: %d, Count: %u\r\n", 
                              scan_trackers[i].id, scan_trackers[i].dlc, scan_trackers[i].rx_count);
                }
            }
            all_printf("====================\r\n");
        }
    }
}

// ALL LISTEN 모드 타임아웃 설정
void can_set_all_listen_timeout(uint32_t timeout_ms)
{
    all_can_timeout_period = timeout_ms;
    DEBUG_PRINT("ALL LISTEN timeout set to %lu ms\r\n", timeout_ms);
} 
