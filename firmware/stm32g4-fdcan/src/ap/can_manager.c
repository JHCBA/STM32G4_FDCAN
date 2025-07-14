#include "can_manager.h"

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

// 정적 변수들
static uint32_t tx_time = 0;
static uint32_t tx_counter = 0;
static uint32_t error_check_time = 0;
static uint32_t last_tx_err_count = 0;
static uint32_t last_rx_err_count = 0;
static uint32_t tx_fail_count = 0;
static bool can_initialized = false;
static bool is_tx_mode = false;  // TX 모드 여부를 저장

// RX 관련 변수들
#define MAX_RX_IDS 10  // 최대 추적할 수 있는 CAN ID 개수
static uint32_t rx_display_time = 0;

typedef struct {
    uint32_t id;
    can_msg_t msg;
    bool has_data;
    uint32_t rx_count;
} rx_id_tracker_t;

static rx_id_tracker_t rx_trackers[MAX_RX_IDS];
static uint8_t active_id_count = 0;

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
        all_printf("CAN mode: NORMAL (TX capable)\r\n");
    }
    else
    {
    	can_mode = CAN_NORMAL;
        //can_mode = CAN_MONITOR; // 수신 전용 모드
        all_printf("CAN mode: MONITOR (RX only)\r\n");
    }
    
    if (canOpen(_DEF_CAN1, can_mode, CAN_FD_BRS, CAN_500K, CAN_2M) == false)
    {
        all_printf("CAN init failed\r\n");
        return false;
    }
    
    all_printf("CAN initialized: 500K/2M, CAN-FD BRS\r\n");
    
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
    all_printf("CAN TEST MODE: TX (Transmitter)\r\n");
    all_printf("Will send %d sample CAN FD messages every 10ms\r\n", SAMPLE_MSG_COUNT);
    
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
            all_printf("CAN TX: ID=0x%03lX, DLC=%d, Data=", tx_msg.id, data_length);
            for (int i = 0; i < data_length; i++)
            {
                all_printf("%02X ", tx_msg.data[i]);
            }
            all_printf("(Seq=%lu)\r\n", tx_counter);
            ledToggle(HW_LED_CH_TX);
            tx_fail_count = 0; // 송신 성공 시 실패 카운터 리셋
        }
        else
        {
            tx_fail_count++;
            all_printf("CAN TX Failed! ID=0x%03lX (Fail count: %lu)\r\n", sample->id, tx_fail_count);
            
            // 연속 실패 시 복구 시도
            if (tx_fail_count >= 3)
            {
                all_printf("Multiple TX failures detected. Attempting CAN recovery...\r\n");
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
    all_printf("CAN TEST MODE: RX (Receiver)\r\n");
    all_printf("Listening for CAN FD messages (summary every 1 second)\r\n");
    
    rx_display_time = millis();
    // Initialize trackers
    for (int i = 0; i < MAX_RX_IDS; i++) {
        rx_trackers[i].id = 0;
        rx_trackers[i].has_data = false;
        rx_trackers[i].rx_count = 0;
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
            // 해당 ID의 추적기 찾기 또는 새로 할당
            int tracker_index = -1;
            
            // 먼저 기존에 추적 중인 ID인지 확인
            for (int i = 0; i < MAX_RX_IDS; i++) {
                if (rx_trackers[i].has_data && rx_trackers[i].id == rx_msg.id) {
                    tracker_index = i;
                    break;
                }
            }
            
            // 기존에 없다면 빈 슬롯 찾기
            if (tracker_index == -1) {
                for (int i = 0; i < MAX_RX_IDS; i++) {
                    if (!rx_trackers[i].has_data) {
                        tracker_index = i;
                        break;
                    }
                }
            }
            
            // 추적기에 데이터 저장
            if (tracker_index != -1) {
                rx_trackers[tracker_index].id = rx_msg.id;
                rx_trackers[tracker_index].msg = rx_msg;
                rx_trackers[tracker_index].has_data = true;
                rx_trackers[tracker_index].rx_count++;
            }
            
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // 1초마다 모든 ID의 최신 메시지 출력
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
            all_printf("=== CAN FD RX Summary (1sec) ===\r\n");
            for (int i = 0; i < MAX_RX_IDS; i++) {
                if (rx_trackers[i].has_data) {
                    all_printf("ID=0x%03lX, Count=%lu, DLC=%d, Data=", 
                              rx_trackers[i].id, rx_trackers[i].rx_count, rx_trackers[i].msg.length);
                    for (int j = 0; j < rx_trackers[i].msg.length; j++) {
                        all_printf("%02X ", rx_trackers[i].msg.data[j]);
                    }
                    all_printf("\r\n");
                }
            }
            all_printf("================================\r\n");
            
            // 카운터만 리셋하고 데이터는 유지 (다음 주기를 위해)
            for (int i = 0; i < MAX_RX_IDS; i++) {
                rx_trackers[i].rx_count = 0;
            }
        }
    }
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
            all_printf("CAN Err: TX=%d, RX=%d, Code=0x%08lX\r\n", 
                      tx_err_count, rx_err_count, can_error);
            last_tx_err_count = tx_err_count;
            last_rx_err_count = rx_err_count;
        }
        
        // Bus-Off 또는 Error Passive 상태 감지 시 복구 시도
        if (can_error & (CAN_ERR_BUS_OFF | CAN_ERR_PASSIVE))
        {
            all_printf("CAN Error detected! Attempting recovery...\r\n");
            can_error_recovery();
        }
        
        // 에러 카운터가 너무 높으면 경고
        if (tx_err_count > 100 || rx_err_count > 100)
        {
            all_printf("WARNING: High error count detected!\r\n");
            
            // 에러 카운터가 매우 높으면 강제 복구 시도
            if (tx_err_count > 200 || rx_err_count > 200)
            {
                all_printf("CRITICAL: Error count too high, forcing recovery!\r\n");
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
                all_printf("Persistent warning detected, attempting recovery...\r\n");
                can_error_recovery();
                warning_count = 0;
            }
        }
    }
}

// CAN 에러 복구
void can_error_recovery(void)
{
    all_printf("Performing CAN recovery...\r\n");
    canRecovery(_DEF_CAN1);
    tx_fail_count = 0; // 복구 후 실패 카운터 리셋
    
    // 에러 카운터 리셋
    last_tx_err_count = 0;
    last_rx_err_count = 0;
} 
