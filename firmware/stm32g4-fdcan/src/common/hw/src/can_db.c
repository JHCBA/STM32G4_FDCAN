#include "can_db.h"
#include "util.h"

// 비트 마스크 및 시프트 정의
#define CHECKSUM_MASK      0xFFFF
#define COUNTER_MASK       0xFF

// 비트 추출 매크로
#define GET_BITS(data, start, length) \
    (((data)[(start) / 8] >> ((start) % 8)) & ((1 << (length)) - 1))

#define GET_BITS_16(data, start, length) \
    ((((uint16_t)(data)[(start) / 8] << 8) | (data)[(start) / 8 + 1]) >> ((start) % 8)) & ((1 << (length)) - 1)

// 비트 설정 매크로
#define SET_BITS(data, start, length, value) \
    do { \
        uint8_t mask = ((1 << (length)) - 1) << ((start) % 8); \
        (data)[(start) / 8] = ((data)[(start) / 8] & ~mask) | (((value) << ((start) % 8)) & mask); \
    } while(0)

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
    msg->accelerator_pedal = GET_BITS(data, 40, 8);
    msg->gear = GET_BITS(data, 192, 3);
    
    return true;
}

float calculate_accelerator_pedal_percent(const accelerator_msg_t *msg)
{
    if (msg == NULL) {
        return 0.0f;
    }
    return (float)msg->accelerator_pedal * 100.0f / 255.0f;
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
    msg->brake_position = GET_BITS_16(data, 40, 16);
    msg->brake_pressed = GET_BITS(data, 57, 1);
    
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

// ===== 기타 함수들 (기본 구현) =====
bool parse_gear_message(const uint8_t *data, uint8_t length, gear_msg_t *msg) { return false; }
bool parse_esp_status_message(const uint8_t *data, uint8_t length, esp_status_msg_t *msg) { return false; }
bool parse_steering_sensors_message(const uint8_t *data, uint8_t length, steering_sensors_msg_t *msg) { return false; }
bool parse_lkas_alt_message(const uint8_t *data, uint8_t length, lkas_alt_msg_t *msg) { return false; }
bool parse_scc_control_message(const uint8_t *data, uint8_t length, scc_control_msg_t *msg) { return false; }
bool parse_cruise_buttons_message(const uint8_t *data, uint8_t length, cruise_buttons_msg_t *msg) { return false; }
bool parse_doors_seatbelts_message(const uint8_t *data, uint8_t length, doors_seatbelts_msg_t *msg) { return false; }
bool parse_blinkers_message(const uint8_t *data, uint8_t length, blinkers_msg_t *msg) { return false; }

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

// 계산 함수들 (기본 구현)
float calculate_steering_angle_deg_sensors(const steering_sensors_msg_t *msg) { return 0.0f; }
gear_state_t get_gear_state(const gear_msg_t *msg) { return GEAR_P; }

// 디버깅용 출력 함수들 (기본 구현)
void print_gear_message(const gear_msg_t *msg) {}
void print_esp_status_message(const esp_status_msg_t *msg) {}
void print_steering_sensors_message(const steering_sensors_msg_t *msg) {}
void print_lkas_alt_message(const lkas_alt_msg_t *msg) {}
void print_scc_control_message(const scc_control_msg_t *msg) {}
void print_cruise_buttons_message(const cruise_buttons_msg_t *msg) {}
void print_doors_seatbelts_message(const doors_seatbelts_msg_t *msg) {}
void print_blinkers_message(const blinkers_msg_t *msg) {}

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
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // Wheel speeds
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