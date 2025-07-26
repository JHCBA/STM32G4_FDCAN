#ifndef CAN_DB_H_
#define CAN_DB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

// CAN ID 정의 (DBC 파일 기반)
#define CAN_ID_ACCELERATOR          0x35    // 53
#define CAN_ID_GEAR_ALT            0x40    // 64
#define CAN_ID_GEAR                0x45    // 69
#define CAN_ID_ESP_STATUS          0x60    // 96
#define CAN_ID_BRAKE               0x65    // 101
#define CAN_ID_GEAR_ALT_2         0x70    // 112
#define CAN_ID_WHEEL_SPEEDS       0xA0    // 160
#define CAN_ID_MDPS               0xEA    // 234
#define CAN_ID_ACCELERATOR_BRAKE_ALT 0x100  // 256
#define CAN_ID_ACCELERATOR_ALT    0x105   // 261
#define CAN_ID_LKAS_ALT           0x110   // 272
#define CAN_ID_STEERING_SENSORS   0x125   // 293
#define CAN_ID_LFA                0x12A   // 298
#define CAN_ID_GEAR_SHIFTER       0x130   // 304
#define CAN_ID_ADRV_0x160        0x160   // 352
#define CAN_ID_CCNC_0x161        0x161   // 353
#define CAN_ID_CCNC_0x162        0x162   // 354
#define CAN_ID_SPAS1              0x165   // 357
#define CAN_ID_SPAS2              0x16A   // 362
#define CAN_ID_TCS                0x175   // 373
#define CAN_ID_SCC_CONTROL        0x1A0   // 416
#define CAN_ID_CRUISE_BUTTONS_ALT 0x1AA   // 426
#define CAN_ID_CCNC_0x1B5        0x1B5   // 437
#define CAN_ID_BLINDSPOTS_REAR_CORNERS 0x1BA  // 442
#define CAN_ID_CRUISE_BUTTONS    0x1CF   // 463
#define CAN_ID_ADRV_0x1da        0x1DA   // 474
#define CAN_ID_LFAHDA_CLUSTER    0x1E0   // 480
#define CAN_ID_BLINDSPOTS_FRONT_CORNER_1 0x1E5  // 485
#define CAN_ID_ADRV_0x1ea        0x1EA   // 490
#define CAN_ID_CLUSTER_SPEED_LIMIT 0x1FA  // 506
#define CAN_ID_CAM_0x1fb         0x1FB   // 507
#define CAN_ID_ADRV_0x200        0x200   // 512
#define CAN_ID_RADAR_0x251       0x251   // 593
#define CAN_ID_MANUAL_SPEED_LIMIT_ASSIST 0x2E0  // 736
#define CAN_ID_ADRV_0x345        0x345   // 837
#define CAN_ID_CAM_0x362         0x362   // 866
#define CAN_ID_BLINDSPOTS_FRONT_CORNER_2 0x36A  // 874
#define CAN_ID_BLINKER_STALKS    0x3C1   // 961
#define CAN_ID_DOORS_SEATBELTS   0x411   // 1041
#define CAN_ID_BLINKERS          0x413   // 1043
#define CAN_ID_DRIVE_MODE        0x478   // 1144
#define CAN_ID_HVAC_TOUCH_BUTTONS 0x47F  // 1151
#define CAN_ID_CLUSTER_INFO      0x4D8   // 1240
#define CAN_ID_LOCAL_TIME2       0x4EB   // 1259
#define CAN_ID_LOCAL_TIME        0x4F0   // 1264

// 기어 상태 정의
typedef enum {
    GEAR_P = 0,
    GEAR_R = 7,
    GEAR_N = 6,
    GEAR_D = 5
} gear_state_t;

// 차량 방향 정의
typedef enum {
    VEHICLE_DIRECTION_STOPPED = 0,
    VEHICLE_DIRECTION_FORWARD,
    VEHICLE_DIRECTION_BACKWARD,
    VEHICLE_DIRECTION_UNKNOWN
} vehicle_direction_t;

// ACCELERATOR 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t accelerator_pedal;      // 40-47: 8비트 (0-255)
    uint8_t gear;                   // 192-194: 3비트
} accelerator_msg_t;

// GEAR 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t gear;                   // 44-46: 3비트
} gear_msg_t;

// ESP_STATUS 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t traction_and_stability_control; // 42-44: 3비트
    uint16_t brake_pressure;        // 128-137: 10비트
    uint8_t brake_pressed;          // 148: 1비트
} esp_status_msg_t;

// BRAKE 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint16_t brake_position;        // 40-55: 16비트 (signed)
    uint8_t brake_pressed;          // 57: 1비트
} brake_msg_t;

// WHEEL_SPEEDS 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t moving_forward;         // 56: 1비트
    uint8_t moving_backward;        // 57: 1비트
    uint8_t moving_forward2;        // 58: 1비트
    uint8_t moving_backward2;       // 59: 1비트
    uint16_t wheel_speed_1;         // 64-79: 16비트
    uint16_t wheel_speed_2;         // 80-95: 16비트
    uint16_t wheel_speed_3;         // 96-111: 16비트
    uint16_t wheel_speed_4;         // 112-127: 16비트
} wheel_speeds_msg_t;

// MDPS (Steering) 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t lka_active;             // 48: 1비트
    uint8_t lka_fault;              // 54: 1비트
    uint16_t steering_out_torque;   // 64-75: 12비트
    uint16_t steering_col_torque;   // 80-92: 13비트
    uint16_t steering_angle;        // 96-111: 16비트 (signed)
    uint16_t steering_angle_2;      // 128-143: 16비트 (signed)
} mdps_msg_t;

// STEERING_SENSORS 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint16_t steering_angle;        // 24-39: 16비트 (signed)
    uint8_t steering_rate;          // 40-47: 8비트
} steering_sensors_msg_t;

// LKAS_ALT 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint8_t lka_mode;               // 24-26: 3비트
    uint8_t lka_warning;            // 32: 1비트
    uint8_t lka_icon;               // 38-39: 2비트
    uint16_t torque_request;        // 41-51: 11비트
    uint8_t steer_req;              // 52: 1비트
    uint8_t lfa_button;             // 56: 1비트
    uint8_t lka_assist;             // 62: 1비트
    uint8_t steer_mode;             // 65-67: 3비트
    uint8_t has_lane_safety;        // 80: 1비트
} lkas_alt_msg_t;

// SCC_CONTROL 메시지 구조체
typedef struct {
    uint16_t checksum;              // 0-15: 16비트
    uint8_t counter;                // 16-23: 8비트
    uint16_t acc_obj_dist;          // 24-34: 11비트
    int16_t acc_obj_rel_spd;        // 35-43: 9비트 (signed)
    uint8_t obj_valid;              // 46: 1비트
    uint8_t main_mode_acc;          // 66: 1비트
    uint8_t acc_mode;               // 68-70: 3비트
    uint8_t cruise_standstill;      // 76: 1비트
    uint8_t distance_setting;       // 88-90: 3비트
    uint8_t vset_dis;               // 103: 8비트
    int16_t a_req_value;            // 128-138: 11비트 (signed)
    int16_t a_req_raw;              // 140-150: 11비트 (signed)
    uint8_t obj_status;             // 176-178: 3비트
    uint8_t stop_req;               // 184: 1비트
    uint16_t new_signal_15;         // 192-202: 11비트
} scc_control_msg_t;

// CRUISE_BUTTONS 메시지 구조체
typedef struct {
    uint8_t checksum_maybe;         // 7-14: 8비트
    uint8_t counter_alt;            // 15-18: 4비트
    uint8_t cruise_buttons;         // 16-18: 3비트
    uint8_t adaptive_cruise_main_btn; // 19: 1비트
    uint8_t normal_cruise_main_btn; // 21: 1비트
    uint8_t lda_btn;                // 23: 1비트
    uint8_t right_paddle;           // 25: 1비트
    uint8_t left_paddle;            // 27: 1비트
} cruise_buttons_msg_t;

// DOORS_SEATBELTS 메시지 구조체
typedef struct {
    uint8_t checksum_maybe;         // 7-14: 8비트
    uint8_t counter_alt;            // 15-18: 4비트
    uint8_t driver_door;            // 24: 1비트
    uint8_t passenger_door;         // 34: 1비트
    uint8_t passenger_seatbelt;     // 36: 1비트
    uint8_t driver_seatbelt;        // 42: 1비트
    uint8_t driver_rear_door;       // 52: 1비트
    uint8_t passenger_rear_door;    // 56: 1비트
} doors_seatbelts_msg_t;

// BLINKERS 메시지 구조체
typedef struct {
    uint8_t left_stalk;             // 8: 1비트
    uint8_t right_stalk;            // 10: 1비트
    uint8_t counter_alt;            // 15-18: 4비트
    uint8_t left_lamp;              // 20: 1비트
    uint8_t right_lamp;             // 22: 1비트
    uint8_t left_lamp_alt;          // 59: 1비트
    uint8_t right_lamp_alt;         // 61: 1비트
    uint8_t use_alt_lamp;           // 62: 1비트
} blinkers_msg_t;

// 변환 함수들
static inline float wheel_speed_to_kph(uint16_t raw_speed) {
    return (float)raw_speed * 0.03125f;
}

static inline uint16_t kph_to_wheel_speed(float kph) {
    return (uint16_t)(kph / 0.03125f);
}

static inline float steering_angle_to_deg(uint16_t raw_angle) {
    return (float)raw_angle * (-0.1f);
}

static inline uint16_t deg_to_steering_angle(float deg) {
    return (uint16_t)(deg / (-0.1f));
}

static inline float brake_position_to_percent(uint16_t raw_position) {
    return (float)raw_position * 100.0f / 65535.0f;
}

static inline uint16_t percent_to_brake_position(float percent) {
    return (uint16_t)(percent * 65535.0f / 100.0f);
}

// 메시지 파싱 함수들
bool parse_accelerator_message(const uint8_t *data, uint8_t length, accelerator_msg_t *msg);
bool parse_gear_message(const uint8_t *data, uint8_t length, gear_msg_t *msg);
bool parse_esp_status_message(const uint8_t *data, uint8_t length, esp_status_msg_t *msg);
bool parse_brake_message(const uint8_t *data, uint8_t length, brake_msg_t *msg);
bool parse_wheel_speeds_message(const uint8_t *data, uint8_t length, wheel_speeds_msg_t *msg);
bool parse_mdps_message(const uint8_t *data, uint8_t length, mdps_msg_t *msg);
bool parse_steering_sensors_message(const uint8_t *data, uint8_t length, steering_sensors_msg_t *msg);
bool parse_lkas_alt_message(const uint8_t *data, uint8_t length, lkas_alt_msg_t *msg);
bool parse_scc_control_message(const uint8_t *data, uint8_t length, scc_control_msg_t *msg);
bool parse_cruise_buttons_message(const uint8_t *data, uint8_t length, cruise_buttons_msg_t *msg);
bool parse_doors_seatbelts_message(const uint8_t *data, uint8_t length, doors_seatbelts_msg_t *msg);
bool parse_blinkers_message(const uint8_t *data, uint8_t length, blinkers_msg_t *msg);

// 메시지 생성 함수들
bool create_accelerator_message(const accelerator_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_gear_message(const gear_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_esp_status_message(const esp_status_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_brake_message(const brake_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_wheel_speeds_message(const wheel_speeds_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_mdps_message(const mdps_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_steering_sensors_message(const steering_sensors_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_lkas_alt_message(const lkas_alt_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_scc_control_message(const scc_control_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_cruise_buttons_message(const cruise_buttons_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_doors_seatbelts_message(const doors_seatbelts_msg_t *msg, uint8_t *data, uint8_t *length);
bool create_blinkers_message(const blinkers_msg_t *msg, uint8_t *data, uint8_t *length);

// 체크섬 계산 함수들
uint16_t calculate_accelerator_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_gear_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_esp_status_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_brake_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_wheel_speeds_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_mdps_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_steering_sensors_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_lkas_alt_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_scc_control_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_cruise_buttons_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_doors_seatbelts_checksum(const uint8_t *data, uint8_t length);
uint16_t calculate_blinkers_checksum(const uint8_t *data, uint8_t length);

// 메시지 유효성 검사 함수들
bool validate_accelerator_message(const uint8_t *data, uint8_t length);
bool validate_gear_message(const uint8_t *data, uint8_t length);
bool validate_esp_status_message(const uint8_t *data, uint8_t length);
bool validate_brake_message(const uint8_t *data, uint8_t length);
bool validate_wheel_speeds_message(const uint8_t *data, uint8_t length);
bool validate_mdps_message(const uint8_t *data, uint8_t length);
bool validate_steering_sensors_message(const uint8_t *data, uint8_t length);
bool validate_lkas_alt_message(const uint8_t *data, uint8_t length);
bool validate_scc_control_message(const uint8_t *data, uint8_t length);
bool validate_cruise_buttons_message(const uint8_t *data, uint8_t length);
bool validate_doors_seatbelts_message(const uint8_t *data, uint8_t length);
bool validate_blinkers_message(const uint8_t *data, uint8_t length);

// 계산 함수들
float calculate_vehicle_speed(const wheel_speeds_msg_t *msg);
vehicle_direction_t get_vehicle_direction(const wheel_speeds_msg_t *msg);
float calculate_steering_angle_deg(const mdps_msg_t *msg);
float calculate_steering_angle_deg_sensors(const steering_sensors_msg_t *msg);
float calculate_accelerator_pedal_percent(const accelerator_msg_t *msg);
float calculate_brake_pedal_percent(const brake_msg_t *msg);
gear_state_t get_gear_state(const gear_msg_t *msg);

// 디버깅용 출력 함수들
void print_accelerator_message(const accelerator_msg_t *msg);
void print_gear_message(const gear_msg_t *msg);
void print_esp_status_message(const esp_status_msg_t *msg);
void print_brake_message(const brake_msg_t *msg);
void print_wheel_speeds_message(const wheel_speeds_msg_t *msg);
void print_mdps_message(const mdps_msg_t *msg);
void print_steering_sensors_message(const steering_sensors_msg_t *msg);
void print_lkas_alt_message(const lkas_alt_msg_t *msg);
void print_scc_control_message(const scc_control_msg_t *msg);
void print_cruise_buttons_message(const cruise_buttons_msg_t *msg);
void print_doors_seatbelts_message(const doors_seatbelts_msg_t *msg);
void print_blinkers_message(const blinkers_msg_t *msg);

// 테스트 함수들
void test_all_can_messages(void);
void test_wheel_speeds_parsing(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_DB_H_ 