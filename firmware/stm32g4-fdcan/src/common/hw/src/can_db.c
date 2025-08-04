#include "can_db.h"
#include "util.h"

// 비트 마스크 및 시프트 정의
#define CHECKSUM_MASK      0xFFFF
#define COUNTER_MASK       0xFF

// 비트 추출 매크로
#define GET_BITS(data, start, length) \
    (((data)[(start) / 8] >> ((start) % 8)) & ((1 << (length)) - 1))

#define GET_BITS_16(data, start, length) \
    (((uint16_t)(data)[(start) / 8 + 1] << 8) | (data)[(start) / 8]) & ((1 << (length)) - 1)

// 10비트 이상의 값을 읽기 위한 매크로 (리틀 엔디안 고려)
#define GET_BITS_10(data, start, length) \
    (((uint16_t)(data)[(start) / 8 + 1] << 8) | (data)[(start) / 8]) & ((1 << (length)) - 1)

// 바이트 경계를 넘어가는 비트 추출을 위한 매크로
#define GET_BITS_CROSS_BYTES(data, start, length) \
    ({ \
        uint32_t result = 0; \
        int bit_pos = start; \
        int bits_remaining = length; \
        int byte_offset = bit_pos / 8; \
        int bit_offset = bit_pos % 8; \
        \
        while (bits_remaining > 0) { \
            int bits_to_read = (bits_remaining < (8 - bit_offset)) ? bits_remaining : (8 - bit_offset); \
            uint8_t mask = ((1 << bits_to_read) - 1) << bit_offset; \
            uint8_t value = (data[byte_offset] & mask) >> bit_offset; \
            result |= (uint32_t)value << (length - bits_remaining); \
            \
            bits_remaining -= bits_to_read; \
            if (bits_remaining > 0) { \
                byte_offset++; \
                bit_offset = 0; \
            } \
        } \
        result; \
    })

// 리틀 엔디안을 고려한 10비트 읽기 함수
static inline uint16_t get_bits_10_le(const uint8_t *data, int start_bit) {
    int byte_offset = start_bit / 8;
    int bit_offset = start_bit % 8;
    
    // 첫 번째 바이트에서 읽을 비트 수
    int bits_from_first = (bit_offset + 10 <= 8) ? 10 : (8 - bit_offset);
    int bits_from_second = 10 - bits_from_first;
    
    uint16_t result = 0;
    
    // 첫 번째 바이트에서 읽기
    if (bits_from_first > 0) {
        uint8_t mask = ((1 << bits_from_first) - 1) << bit_offset;
        result = (data[byte_offset] & mask) >> bit_offset;
    }
    
    // 두 번째 바이트에서 읽기 (필요한 경우)
    if (bits_from_second > 0) {
        uint8_t mask = (1 << bits_from_second) - 1;
        result |= (uint16_t)(data[byte_offset + 1] & mask) << bits_from_first;
    }
    
    return result;
}
// 비트 설정 매크로
#define SET_BITS(data, start, length, value) \
    do { \
        uint32_t mask = ((1UL << (length)) - 1) << ((start) % 8); \
        (data)[(start) / 8] = ((data)[(start) / 8] & ~(uint8_t)mask) | (((value) << ((start) % 8)) & (uint8_t)mask); \
    } while(0)



/* 
BO_ 160 WHEEL_SPEEDS: 24 XXX
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ MOVING_FORWARD : 56|1@0+ (1,0) [0|1] "" XXX
 SG_ MOVING_BACKWARD : 57|1@0+ (1,0) [0|1] "" XXX
 SG_ MOVING_FORWARD2 : 58|1@0+ (1,0) [0|1] "" XXX
 SG_ MOVING_BACKWARD2 : 59|1@0+ (1,0) [0|1] "" XXX
 SG_ WHEEL_SPEED_1 : 64|16@1+ (0.03125,0) [0|65535] "kph" XXX
 SG_ WHEEL_SPEED_2 : 80|16@1+ (0.03125,0) [0|65535] "kph" XXX
 SG_ WHEEL_SPEED_3 : 96|16@1+ (0.03125,0) [0|65535] "kph" XXX
 SG_ WHEEL_SPEED_4 : 112|16@1+ (0.03125,0) [0|65535] "kph" XXX
 */

// ===== WHEEL_SPEEDS 메시지 구현 =====
bool parse_wheel_speeds_message(const uint8_t *data, uint8_t length, wheel_speeds_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 16) {
        return false;
    }
    
    // 체크섬 추출 (0-15 비트)
    msg->checksum = GET_BITS_16(data, 0, 16);
    
    // 카운터 추출 (16-23 비트)
    msg->counter = GET_BITS(data, 16, 8);
    
    // 이동 방향 정보 추출
    msg->moving_forward = GET_BITS(data, 56, 1);
    msg->moving_backward = GET_BITS(data, 57, 1);
    msg->moving_forward2 = GET_BITS(data, 58, 1);
    msg->moving_backward2 = GET_BITS(data, 59, 1);
    
    // 바퀴 속도 추출
    msg->wheel_speed_1 = GET_BITS_16(data, 64, 16);
    msg->wheel_speed_2 = GET_BITS_16(data, 80, 16);
    msg->wheel_speed_3 = GET_BITS_16(data, 96, 16);
    msg->wheel_speed_4 = GET_BITS_16(data, 112, 16);
    
    return true;
}

bool create_wheel_speeds_message(const wheel_speeds_msg_t *msg, uint8_t *data, uint8_t *length)
{
    if (msg == NULL || data == NULL || length == NULL) {
        return false;
    }
    
    // 데이터 초기화
    memset(data, 0, 16);
    
    // 체크섬 설정
    SET_BITS(data, 0, 16, msg->checksum);
    
    // 카운터 설정
    SET_BITS(data, 16, 8, msg->counter);
    
    // 이동 방향 정보 설정
    SET_BITS(data, 56, 1, msg->moving_forward);
    SET_BITS(data, 57, 1, msg->moving_backward);
    SET_BITS(data, 58, 1, msg->moving_forward2);
    SET_BITS(data, 59, 1, msg->moving_backward2);
    
    // 바퀴 속도 설정
    SET_BITS(data, 64, 16, msg->wheel_speed_1);
    SET_BITS(data, 80, 16, msg->wheel_speed_2);
    SET_BITS(data, 96, 16, msg->wheel_speed_3);
    SET_BITS(data, 112, 16, msg->wheel_speed_4);
    
    *length = 16;
    return true;
}

uint16_t calculate_wheel_speeds_checksum(const uint8_t *data, uint8_t length)
{
    if (data == NULL || length < 16) {
        return 0;
    }
    
    uint16_t checksum = 0;
    
    // 체크섬 필드를 제외한 모든 바이트를 더함
    for (int i = 2; i < length; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

bool validate_wheel_speeds_message(const uint8_t *data, uint8_t length)
{
    if (data == NULL || length < 16) {
        return false;
    }
    
    // 체크섬 검증
    uint16_t calculated_checksum = calculate_wheel_speeds_checksum(data, length);
    uint16_t received_checksum = GET_BITS_16(data, 0, 16);
    
    return (calculated_checksum == received_checksum);
}

float calculate_vehicle_speed(const wheel_speeds_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    
    // 4개 바퀴의 평균 속도 계산
    float speed1 = wheel_speed_to_kph(msg->wheel_speed_1);
    float speed2 = wheel_speed_to_kph(msg->wheel_speed_2);
    float speed3 = wheel_speed_to_kph(msg->wheel_speed_3);
    float speed4 = wheel_speed_to_kph(msg->wheel_speed_4);
    
    return (speed1 + speed2 + speed3 + speed4) / 4.0f;
}

vehicle_direction_t get_vehicle_direction(const wheel_speeds_msg_t *msg)
{
    if (msg == NULL) {
        return VEHICLE_DIRECTION_UNKNOWN;
    }
    
    // 정지 상태 확인 (모든 바퀴 속도가 0)
    if (msg->wheel_speed_1 == 0 && msg->wheel_speed_2 == 0 && 
        msg->wheel_speed_3 == 0 && msg->wheel_speed_4 == 0) {
        return VEHICLE_DIRECTION_STOPPED;
    }
    
    // 전진 상태 확인
    if (msg->moving_forward || msg->moving_forward2) {
        return VEHICLE_DIRECTION_FORWARD;
    }
    
    // 후진 상태 확인
    if (msg->moving_backward || msg->moving_backward2) {
        return VEHICLE_DIRECTION_BACKWARD;
    }
    
    return VEHICLE_DIRECTION_UNKNOWN;
}

void print_wheel_speeds_message(const wheel_speeds_msg_t *msg)
{
    if (msg == NULL) {
        return;
    }
    
    printf("=== WHEEL_SPEEDS Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Moving Forward: %d\n", msg->moving_forward);
    printf("Moving Backward: %d\n", msg->moving_backward);
    printf("Moving Forward2: %d\n", msg->moving_forward2);
    printf("Moving Backward2: %d\n", msg->moving_backward2);
    printf("Wheel Speed 1: %.2f kph (raw: %d)\n", wheel_speed_to_kph(msg->wheel_speed_1), msg->wheel_speed_1);
    printf("Wheel Speed 2: %.2f kph (raw: %d)\n", wheel_speed_to_kph(msg->wheel_speed_2), msg->wheel_speed_2);
    printf("Wheel Speed 3: %.2f kph (raw: %d)\n", wheel_speed_to_kph(msg->wheel_speed_3), msg->wheel_speed_3);
    printf("Wheel Speed 4: %.2f kph (raw: %d)\n", wheel_speed_to_kph(msg->wheel_speed_4), msg->wheel_speed_4);
    
    float avg_speed = calculate_vehicle_speed(msg);
    vehicle_direction_t direction = get_vehicle_direction(msg);
    
    printf("Average Vehicle Speed: %.2f kph\n", avg_speed);
    printf("Vehicle Direction: %d\n", direction);
    printf("===========================\n");
}

// ===== ACCELERATOR 메시지 구현 =====
bool parse_accelerator_message(const uint8_t *data, uint8_t length, accelerator_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 24) {
        return false;
    }
    
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    msg->accelerator_pedal = get_bits_10_le(data, 88);
    msg->gear = GET_BITS(data, 192, 3);
    
    // 디버그: 모든 바이트 출력
    printf("ACCELERATOR DEBUG - Bytes: ");
    for (int i = 0; i < length && i < 32; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    printf("ACCELERATOR DEBUG - Bit 88-97 (bytes 11-12): %02X %02X\n", data[11], data[12]);
    printf("ACCELERATOR DEBUG - Raw accelerator_pedal: %d (0x%04X)\n", msg->accelerator_pedal, msg->accelerator_pedal);
    
    return true;
}

float calculate_accelerator_pedal_percent(const accelerator_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    return (float)msg->accelerator_pedal * (100.0f / 1022.0f);
}

void print_accelerator_message(const accelerator_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== ACCELERATOR Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Accelerator Pedal: %.1f%% (raw: %d)\n", calculate_accelerator_pedal_percent(msg), msg->accelerator_pedal);
    printf("Gear: %d\n", msg->gear);
    printf("===========================\n");
}

// ===== BRAKE 메시지 구현 =====
bool parse_brake_message(const uint8_t *data, uint8_t length, brake_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 8) {
        return false;
    }
    
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    msg->brake_position = get_bits_10_le(data, 128);
    msg->brake_pressed = GET_BITS(data, 57, 1);
    
    // 디버그: 모든 바이트 출력
    printf("BRAKE DEBUG - Bytes: ");
    for (int i = 0; i < length && i < 32; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    printf("BRAKE DEBUG - Bit 40-55 (bytes 5-6): %02X %02X\n", data[5], data[6]);
    printf("BRAKE DEBUG - Bit 128-143 (bytes 16-17): %02X %02X\n", data[16], data[17]);
    printf("BRAKE DEBUG - Raw brake_position: %d\n", msg->brake_position);
    
    return true;
}

float calculate_brake_pedal_percent(const brake_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    return brake_position_to_percent(msg->brake_position);
}

void print_brake_message(const brake_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== BRAKE Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Brake Position: %.1f%% (raw: %d)\n", calculate_brake_pedal_percent(msg), msg->brake_position);
    printf("Brake Pressed: %d\n", msg->brake_pressed);
    printf("=====================\n");
}

// ===== MDPS (Steering) 메시지 구현 =====
bool parse_mdps_message(const uint8_t *data, uint8_t length, mdps_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 18) {
        return false;
    }
    
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    msg->lka_active = GET_BITS(data, 48, 1);
    msg->lka_fault = GET_BITS(data, 54, 1);
    msg->steering_out_torque = GET_BITS_16(data, 64, 12);
    msg->steering_col_torque = GET_BITS_16(data, 80, 13);
    msg->steering_angle = GET_BITS_16(data, 96, 16);
    msg->steering_angle_2 = GET_BITS_16(data, 128, 16);
    
    return true;
}

float calculate_steering_angle_deg(const mdps_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    return steering_angle_to_deg(msg->steering_angle);
}

void print_mdps_message(const mdps_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== MDPS (Steering) Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("LKA Active: %d\n", msg->lka_active);
    printf("LKA Fault: %d\n", msg->lka_fault);
    printf("Steering Out Torque: %d\n", msg->steering_out_torque);
    printf("Steering Col Torque: %d\n", msg->steering_col_torque);
    printf("Steering Angle: %.1f deg (raw: %d)\n", calculate_steering_angle_deg(msg), msg->steering_angle);
    printf("Steering Angle 2: %.1f deg (raw: %d)\n", steering_angle_to_deg(msg->steering_angle_2), msg->steering_angle_2);
    printf("================================\n");
}

// ===== STEERING_SENSORS 메시지 구현 =====
bool parse_steering_sensors_message(const uint8_t *data, uint8_t length, steering_sensors_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 6) {
        return false;
    }
    
    // DBC: BO_ 293 STEERING_SENSORS: 16 XXX
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    
    // SG_ STEERING_ANGLE : 24|16@1- (-0.1,0) [0|255] "deg" XXX
    int16_t angle_raw = (int16_t)GET_BITS_16(data, 24, 16);
    msg->steering_angle = (uint16_t)angle_raw;
    
    // SG_ STEERING_RATE : 40|8@1+ (4,0) [0|1016] "deg/s" XXX
    msg->steering_rate = GET_BITS(data, 40, 8);
    
    return true;
}

float calculate_steering_angle_deg_sensors(const steering_sensors_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    
    // DBC 정의에 따라 변환: (-0.1,0) factor
    int16_t angle_raw = (int16_t)msg->steering_angle;
    return (float)angle_raw * (-0.1f);
}

float calculate_steering_rate_deg_per_sec(const steering_sensors_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    
    // DBC 정의에 따라 변환: (4,0) factor
    return (float)msg->steering_rate * 4.0f;
}

void print_steering_sensors_message(const steering_sensors_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== STEERING_SENSORS Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Steering Angle: %.1f deg (raw: %d)\n", calculate_steering_angle_deg_sensors(msg), msg->steering_angle);
    printf("Steering Rate: %.1f deg/s (raw: %d)\n", calculate_steering_rate_deg_per_sec(msg), msg->steering_rate);
    printf("================================\n");
}

// ===== GEAR 메시지 구현 =====
bool parse_gear_message(const uint8_t *data, uint8_t length, gear_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 4) {
        return false;
    }
    
    // DBC: BO_ 64 GEAR: 24 XXX
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    
    // SG_ GEAR : 32|3@1+ (1,0) [0|7] "" XXX
    msg->gear = GET_BITS(data, 32, 3);
    
    return true;
}

gear_state_t get_gear_state(const gear_msg_t *msg)
{
    if (msg == NULL) {
        return GEAR_P;
    }
    
    switch (msg->gear) {
        case 0: return GEAR_P;  // P
        case 5: return GEAR_D;  // D
        case 6: return GEAR_N;  // N
        case 7: return GEAR_R;  // R
        default: return GEAR_P;
    }
}

void print_gear_message(const gear_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== GEAR Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Gear: %d (", msg->gear);
    
    switch (msg->gear) {
        case 0: printf("P"); break;
        case 5: printf("D"); break;
        case 6: printf("N"); break;
        case 7: printf("R"); break;
        default: printf("Unknown"); break;
    }
    printf(")\n");
    printf("===================\n");
}

// ===== ESP_STATUS 메시지 구현 =====
bool parse_esp_status_message(const uint8_t *data, uint8_t length, esp_status_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 8) {
        return false;
    }
    
    // DBC: BO_ 96 ESP_STATUS: 32 XXX
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    
    // SG_ TRACTION_AND_STABILITY_CONTROL : 42|3@1+ (1,0) [0|63] "" XXX
    msg->traction_and_stability_control = GET_BITS(data, 42, 3);
    
    // SG_ BRAKE_PRESSURE : 128|10@1+ (1,0) [0|65535] "" XXX
    msg->brake_pressure = GET_BITS_16(data, 128, 10);
    
    // SG_ BRAKE_PRESSED : 148|1@1+ (1,0) [0|3] "" XXX
    msg->brake_pressed = GET_BITS(data, 148, 1);
    
    return true;
}

void print_esp_status_message(const esp_status_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== ESP_STATUS Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Traction & Stability Control: %d\n", msg->traction_and_stability_control);
    printf("Brake Pressure: %d\n", msg->brake_pressure);
    printf("Brake Pressed: %d\n", msg->brake_pressed);
    printf("==========================\n");
}

// ===== LKAS_ALT 메시지 구현 =====
bool parse_lkas_alt_message(const uint8_t *data, uint8_t length, lkas_alt_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 12) {
        return false;
    }
    
    // DBC: BO_ 272 LKAS_ALT: 32 XXX
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    
    // SG_ LKA_MODE : 24|3@1+ (1,0) [0|7] "" XXX
    msg->lka_mode = GET_BITS(data, 24, 3);
    
    // SG_ LKA_WARNING : 32|1@1+ (1,0) [0|1] "" XXX
    msg->lka_warning = GET_BITS(data, 32, 1);
    
    // SG_ LKA_ICON : 38|2@1+ (1,0) [0|255] "" XXX
    msg->lka_icon = GET_BITS(data, 38, 2);
    
    // SG_ TORQUE_REQUEST : 41|11@1+ (1,-1024) [0|4095] "" XXX
    msg->torque_request = GET_BITS_16(data, 41, 11);
    
    // SG_ STEER_REQ : 52|1@1+ (1,0) [0|1] "" XXX
    msg->steer_req = GET_BITS(data, 52, 1);
    
    // SG_ LFA_BUTTON : 56|1@1+ (1,0) [0|255] "" XXX
    msg->lfa_button = GET_BITS(data, 56, 1);
    
    // SG_ LKA_ASSIST : 62|1@1+ (1,0) [0|1] "" XXX
    msg->lka_assist = GET_BITS(data, 62, 1);
    
    // SG_ STEER_MODE : 65|3@1+ (1,0) [0|1] "" XXX
    msg->steer_mode = GET_BITS(data, 65, 3);
    
    // SG_ HAS_LANE_SAFETY : 80|1@0+ (1,0) [0|1] "" XXX
    msg->has_lane_safety = GET_BITS(data, 80, 1);
    
    return true;
}

void print_lkas_alt_message(const lkas_alt_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== LKAS_ALT Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("LKA Mode: %d\n", msg->lka_mode);
    printf("LKA Warning: %d\n", msg->lka_warning);
    printf("LKA Icon: %d\n", msg->lka_icon);
    printf("Torque Request: %d\n", msg->torque_request);
    printf("Steer Req: %d\n", msg->steer_req);
    printf("LFA Button: %d\n", msg->lfa_button);
    printf("LKA Assist: %d\n", msg->lka_assist);
    printf("Steer Mode: %d\n", msg->steer_mode);
    printf("Has Lane Safety: %d\n", msg->has_lane_safety);
    printf("=======================\n");
}

// ===== BLINDSPOTS_REAR_CORNERS 메시지 구현 =====
bool parse_blindspots_rear_corners_message(const uint8_t *data, uint8_t length, blindspots_rear_corners_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 24) {
        return false;
    }
    
    // DBC: BO_ 442 BLINDSPOTS_REAR_CORNERS: 24 XXX
    msg->checksum = GET_BITS_16(data, 0, 16);
    msg->counter = GET_BITS(data, 16, 8);
    
    // SG_ LEFT_BLOCKED : 24|1@0+ (1,0) [0|1] "" XXX
    msg->left_blocked = GET_BITS(data, 24, 1);
    
    // SG_ LEFT_MB : 30|1@0+ (1,0) [0|3] "" XXX
    msg->left_mb = GET_BITS(data, 30, 1);
    
    // SG_ MORE_LEFT_PROB : 32|1@1+ (1,0) [0|3] "" XXX
    msg->more_left_prob = GET_BITS(data, 32, 1);
    
    // SG_ FL_INDICATOR : 46|6@0+ (1,0) [0|1] "" XXX
    msg->fl_indicator = GET_BITS(data, 46, 6);
    
    // SG_ FR_INDICATOR : 54|6@0+ (1,0) [0|63] "" XXX
    msg->fr_indicator = GET_BITS(data, 54, 6);
    
    // SG_ RIGHT_BLOCKED : 64|1@0+ (1,0) [0|1] "" XXX
    msg->right_blocked = GET_BITS(data, 64, 1);
    
    // SG_ COLLISION_AVOIDANCE_ACTIVE : 68|1@0+ (1,0) [0|1] "" XXX
    msg->collision_avoidance_active = GET_BITS(data, 68, 1);
    
    // SG_ NEW_SIGNAL_2 : 96|1@0+ (1,0) [0|1] "" XXX
    msg->new_signal_2 = GET_BITS(data, 96, 1);
    
    // SG_ FL_INDICATOR_ALT : 138|1@0+ (1,0) [0|1] "" XXX
    msg->fl_indicator_alt = GET_BITS(data, 138, 1);
    
    // SG_ FR_INDICATOR_ALT : 141|1@0+ (1,0) [0|1] "" XXX
    msg->fr_indicator_alt = GET_BITS(data, 141, 1);
    
    return true;
}

void print_blindspots_rear_corners_message(const blindspots_rear_corners_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== BLINDSPOTS_REAR_CORNERS Message ===\n");
    printf("Checksum: 0x%04X\n", msg->checksum);
    printf("Counter: %d\n", msg->counter);
    printf("Left Blocked: %s\n", msg->left_blocked ? "Yes" : "No");
    printf("Left MB: %s\n", msg->left_mb ? "Yes" : "No");
    printf("More Left Prob: %s\n", msg->more_left_prob ? "Yes" : "No");
    printf("FL Indicator: %d\n", msg->fl_indicator);
    printf("FR Indicator: %d\n", msg->fr_indicator);
    printf("Right Blocked: %s\n", msg->right_blocked ? "Yes" : "No");
    printf("Collision Avoidance Active: %s\n", msg->collision_avoidance_active ? "Yes" : "No");
    printf("New Signal 2: %s\n", msg->new_signal_2 ? "Yes" : "No");
    printf("FL Indicator Alt: %s\n", msg->fl_indicator_alt ? "Yes" : "No");
    printf("FR Indicator Alt: %s\n", msg->fr_indicator_alt ? "Yes" : "No");
    printf("=====================================\n");
}

// ===== CRUISE_BUTTONS 메시지 구현 =====
bool parse_cruise_buttons_message(const uint8_t *data, uint8_t length, cruise_buttons_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 2) {
        return false;
    }
    
    // DBC: BO_ 463 CRUISE_BUTTONS: 8 XXX
    msg->checksum_maybe = GET_BITS(data, 7, 8);
    msg->counter_alt = GET_BITS(data, 15, 4);
    msg->cruise_buttons = GET_BITS(data, 16, 3);
    msg->adaptive_cruise_main_btn = GET_BITS(data, 19, 1);
    msg->normal_cruise_main_btn = GET_BITS(data, 21, 1);
    msg->lda_btn = GET_BITS(data, 23, 1);
    msg->right_paddle = GET_BITS(data, 25, 1);
    msg->left_paddle = GET_BITS(data, 27, 1);
    
    return true;
}

void print_cruise_buttons_message(const cruise_buttons_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== CRUISE_BUTTONS Message ===\n");
    printf("Checksum Maybe: %d\n", msg->checksum_maybe);
    printf("Counter Alt: %d\n", msg->counter_alt);
    printf("Cruise Buttons: %d\n", msg->cruise_buttons);
    printf("Adaptive Cruise Main Btn: %d\n", msg->adaptive_cruise_main_btn);
    printf("Normal Cruise Main Btn: %d\n", msg->normal_cruise_main_btn);
    printf("LDA Btn: %d\n", msg->lda_btn);
    printf("Right Paddle: %d\n", msg->right_paddle);
    printf("Left Paddle: %d\n", msg->left_paddle);
    printf("=============================\n");
}

// ===== DOORS_SEATBELTS 메시지 구현 =====
bool parse_doors_seatbelts_message(const uint8_t *data, uint8_t length, doors_seatbelts_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 2) {
        return false;
    }
    
    // DBC: BO_ 1041 DOORS_SEATBELTS: 8 XXX
    msg->checksum_maybe = GET_BITS(data, 7, 8);
    msg->counter_alt = GET_BITS(data, 15, 4);
    msg->driver_door = GET_BITS(data, 24, 1);
    msg->passenger_door = GET_BITS(data, 34, 1);
    msg->passenger_seatbelt = GET_BITS(data, 36, 1);
    msg->driver_seatbelt = GET_BITS(data, 42, 1);
    msg->driver_rear_door = GET_BITS(data, 52, 1);
    msg->passenger_rear_door = GET_BITS(data, 56, 1);
    
    return true;
}

void print_doors_seatbelts_message(const doors_seatbelts_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== DOORS_SEATBELTS Message ===\n");
    printf("Checksum Maybe: %d\n", msg->checksum_maybe);
    printf("Counter Alt: %d\n", msg->counter_alt);
    printf("Driver Door: %s\n", msg->driver_door ? "Open" : "Closed");
    printf("Passenger Door: %s\n", msg->passenger_door ? "Open" : "Closed");
    printf("Passenger Seatbelt: %s\n", msg->passenger_seatbelt ? "Unlatched" : "Latched");
    printf("Driver Seatbelt: %s\n", msg->driver_seatbelt ? "Unlatched" : "Latched");
    printf("Driver Rear Door: %s\n", msg->driver_rear_door ? "Open" : "Closed");
    printf("Passenger Rear Door: %s\n", msg->passenger_rear_door ? "Open" : "Closed");
    printf("==============================\n");
}

// ===== BLINKERS 메시지 구현 =====
bool parse_blinkers_message(const uint8_t *data, uint8_t length, blinkers_msg_t *msg)
{
    if (data == NULL || msg == NULL || length < 2) {
        return false;
    }
    
    // DBC: BO_ 1043 BLINKERS: 8 XXX
    msg->left_stalk = GET_BITS(data, 8, 1);
    msg->right_stalk = GET_BITS(data, 10, 1);
    msg->counter_alt = GET_BITS(data, 15, 4);
    msg->left_lamp = GET_BITS(data, 20, 1);
    msg->right_lamp = GET_BITS(data, 22, 1);
    msg->left_lamp_alt = GET_BITS(data, 59, 1);
    msg->right_lamp_alt = GET_BITS(data, 61, 1);
    msg->use_alt_lamp = GET_BITS(data, 62, 1);
    
    return true;
}

void print_blinkers_message(const blinkers_msg_t *msg)
{
    if (msg == NULL) return;
    
    printf("=== BLINKERS Message ===\n");
    printf("Left Stalk: %s\n", msg->left_stalk ? "On" : "Off");
    printf("Right Stalk: %s\n", msg->right_stalk ? "On" : "Off");
    printf("Counter Alt: %d\n", msg->counter_alt);
    printf("Left Lamp: %s\n", msg->left_lamp ? "On" : "Off");
    printf("Right Lamp: %s\n", msg->right_lamp ? "On" : "Off");
    printf("Left Lamp Alt: %s\n", msg->left_lamp_alt ? "On" : "Off");
    printf("Right Lamp Alt: %s\n", msg->right_lamp_alt ? "On" : "Off");
    printf("Use Alt Lamp: %s\n", msg->use_alt_lamp ? "Yes" : "No");
    printf("=======================\n");
}

// 메시지 생성 함수들 (기본 구현)
bool create_accelerator_message(const accelerator_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_gear_message(const gear_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_esp_status_message(const esp_status_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_brake_message(const brake_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_mdps_message(const mdps_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_steering_sensors_message(const steering_sensors_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_lkas_alt_message(const lkas_alt_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_scc_control_message(const scc_control_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_cruise_buttons_message(const cruise_buttons_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_doors_seatbelts_message(const doors_seatbelts_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_blinkers_message(const blinkers_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }
bool create_blindspots_rear_corners_message(const blindspots_rear_corners_msg_t *msg, uint8_t *data, uint8_t *length) { return false; }

// 체크섬 계산 함수들 (기본 구현)
uint16_t calculate_accelerator_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_gear_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_esp_status_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_brake_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_mdps_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_steering_sensors_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_lkas_alt_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_scc_control_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_cruise_buttons_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_doors_seatbelts_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_blinkers_checksum(const uint8_t *data, uint8_t length) { return 0; }
uint16_t calculate_blindspots_rear_corners_checksum(const uint8_t *data, uint8_t length) { return 0; }

// 메시지 유효성 검사 함수들 (기본 구현)
bool validate_accelerator_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_gear_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_esp_status_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_brake_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_mdps_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_steering_sensors_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_lkas_alt_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_scc_control_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_cruise_buttons_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_doors_seatbelts_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_blinkers_message(const uint8_t *data, uint8_t length) { return false; }
bool validate_blindspots_rear_corners_message(const uint8_t *data, uint8_t length) { return false; }


// 테스트 함수들
void test_all_can_messages(void)
{
    printf("=== Testing All CAN Messages ===\n");
    test_wheel_speeds_parsing();
    printf("===============================\n");
}

void test_wheel_speeds_parsing(void)
{
    // 샘플 WHEEL_SPEEDS 메시지 데이터 (실제 데이터와 유사)
    uint8_t sample_data[16] = {
        0x12, 0x34,  // Checksum
        0x56,         // Counter
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Reserved
        0x01,         // Moving flags (bit 56: forward)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Wheel speeds (총 16개)
    };
    
    // 바퀴 속도 설정 (예: 50 kph)
    uint16_t speed_50kph = kph_to_wheel_speed(50.0f);
    sample_data[8] = (speed_50kph >> 8) & 0xFF;   // Wheel speed 1 high byte
    sample_data[9] = speed_50kph & 0xFF;           // Wheel speed 1 low byte
    sample_data[10] = (speed_50kph >> 8) & 0xFF;  // Wheel speed 2 high byte
    sample_data[11] = speed_50kph & 0xFF;          // Wheel speed 2 low byte
    sample_data[12] = (speed_50kph >> 8) & 0xFF;  // Wheel speed 3 high byte
    sample_data[13] = speed_50kph & 0xFF;          // Wheel speed 3 low byte
    sample_data[14] = (speed_50kph >> 8) & 0xFF;  // Wheel speed 4 high byte
    sample_data[15] = speed_50kph & 0xFF;          // Wheel speed 4 low byte
    
    wheel_speeds_msg_t parsed_msg;
    
    if (parse_wheel_speeds_message(sample_data, 16, &parsed_msg)) {
        printf("=== Test WHEEL_SPEEDS Parsing ===\n");
        print_wheel_speeds_message(&parsed_msg);
        
        // 메시지 재생성 테스트
        uint8_t regenerated_data[16];
        uint8_t regenerated_length;
        
        if (create_wheel_speeds_message(&parsed_msg, regenerated_data, &regenerated_length)) {
            printf("Message regeneration successful\n");
        }
    } else {
        printf("Failed to parse WHEEL_SPEEDS message\n");
    }
} 