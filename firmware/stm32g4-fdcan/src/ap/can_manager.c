#include "can_manager.h"

// DEBUG 출력 매크로
#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) all_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

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

// CAN ID to Protocol ID 매핑 테이블
static const can_to_protocol_map_t can_protocol_map[] = {
    {0x100, PROTOCOL_ID_VEHICLE_SPEED,   100},  // Vehicle Speed
    {0x101, PROTOCOL_ID_APS,            100},  // APS
    {0x102, PROTOCOL_ID_BPS,            100},  // BPS
    {0x103, PROTOCOL_ID_STEERING_ANGLE, 100},  // Steering Angle
    {0x104, PROTOCOL_ID_EPS_ERR,        100},  // EPS Error
    {0x105, PROTOCOL_ID_GEAR,           100},  // Gear
    {0x106, PROTOCOL_ID_TURN_SIGNAL,    100},  // Turn Signal
    {0x107, PROTOCOL_ID_DOOR_OPEN,     1000},  // Door Open
    {0x108, PROTOCOL_ID_SEAT_BELT,     1000},  // Seat Belt
    {0x109, PROTOCOL_ID_RADAR,          100},  // Radar
};

#define CAN_PROTOCOL_MAP_SIZE (sizeof(can_protocol_map) / sizeof(can_protocol_map[0]))

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

// RX 관련 변수들
#define MAX_RX_IDS 10  // 최대 추적할 수 있는 CAN ID 개수
static uint32_t rx_display_time = 0;

typedef struct {
    uint32_t id;
    can_msg_t msg;
    bool has_data;
    uint32_t rx_count;
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
            // 차량 속도 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // 0~255 m/s
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_APS:
            // 엑셀 페달 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // 0~100%
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_BPS:
            // 브레이크 페달 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // 0~100%
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_STEERING_ANGLE:
            // 스티어링 앵글 (6바이트)
            if (can_length >= 6) {
                // Byte0-1: 각도(deg) [-900 ~ 900]
                protocol_data[0] = can_data[0];
                protocol_data[1] = can_data[1];
                // Byte2: 각속도(deg/s) [-127 ~ 128]
                protocol_data[2] = can_data[2];
                // Byte3-4: 토크 (Nm) [-1000 ~ 1000]
                protocol_data[3] = can_data[3];
                protocol_data[4] = can_data[4];
                // Byte5: 핸들 입력 여부 [0 ~ 1]
                protocol_data[5] = can_data[5];
                *protocol_length = 6;
                return true;
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
            
        case PROTOCOL_ID_GEAR:
            // 기어 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // 0:P, 1:R, 2:N, 3:D
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_TURN_SIGNAL:
            // 방향지시등 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // Bit0: Left, Bit1: Right
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_DOOR_OPEN:
            // 차량 문 열림 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // Bit별 문 상태
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_SEAT_BELT:
            // 안전벨트 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // Bit0: 미착용 여부
                *protocol_length = 1;
                return true;
            }
            break;
            
        case PROTOCOL_ID_RADAR:
            // 레이더 상태 (1바이트)
            if (can_length >= 1) {
                protocol_data[0] = can_data[0];  // Bit0: 좌측, Bit1: 우측
                *protocol_length = 1;
                return true;
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
        // 순수 바이너리 데이터만 16진수로 출력
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
        can_rx_init();
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
    
    // RX 처리 (모든 모드에서 수신 모니터링)
    can_rx_process();
    
    // TX 처리 (TX 모드에서만)
    if (is_tx_mode)
    {
        can_tx_process();
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
    DEBUG_PRINT("CAN TEST MODE: RX (Receiver)\r\n");
    DEBUG_PRINT("Protocol output enabled via UART\r\n");
    
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
                        DEBUG_PRINT("ID=0x%03lX->0x%02X, Count=%lu, DLC=%d\r\n", 
                                   rx_trackers[i].id, (uint8_t)data_id, 
                                   rx_trackers[i].rx_count, rx_trackers[i].msg.length);
                    } else {
                        DEBUG_PRINT("ID=0x%03lX (unmapped), Count=%lu, DLC=%d\r\n", 
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
