#include "can_tx.h"
#include "can_manager.h"
#include "can_db.h"
#include "uds_handler.h"
#include "swtimer.h"

// DEBUG 출력 매크로
#if DEBUG_CAN_PROTOCOL
#define DEBUG_PRINT(fmt, ...) all_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

#define CAN_TX_INTERVAL_MS 10  // Send 2 messages per 10ms cycle

// 샘플 CAN FD 메시지 정의
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[64];
} sample_can_msg_t;

// 실제 CAN FD 메시지 샘플들 (DBC 기반)
static const sample_can_msg_t sample_messages[] = {
    // TPMS_01_200ms (ID: 928 = 0x3A0, 16 bytes)
    // Tire Pressure (PSI): FL=32, FR=32, RL=30, RR=30
    // Signal start bit 32, 40, 48, 56 (각 8bit, scale=1, offset=0)
   {
       .id = 0x3A0,
       .dlc = CAN_DLC_16,
       .data = {0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x1E, 0x1E,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // WHL_01_10ms (ID: 160 = 0xA0, 24 bytes)
   // Wheel Speed (km/h): FL=60, FR=60, RL=60, RR=60
   // Start bit 64, 80, 96, 112 (각 14bit, scale=0.03125)
   // 60km/h = 60/0.03125 = 1920 = 0x780
   {
       .id = 0xA0,
       .dlc = CAN_DLC_24,
       .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x80, 0x07, 0x80, 0x07, 0x80, 0x07, 0x80, 0x07,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // MCU_01_10ms (ID: 266 = 0x10A, 32 bytes)
   // Motor1 Speed: 1500rpm (bit 24, 16bit, scale=1)
   // Motor1 Torque: 40Nm (bit 192, 14bit, scale=0.125) -> 40/0.125=320=0x140
   {
       .id = 0x10A,
       .dlc = CAN_DLC_32,
       .data = {0x00, 0x00, 0x00, 0x05, 0xDC, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // MCU_02_10ms (ID: 288 = 0x120, 32 bytes)
   // Motor2 Speed: 1500rpm (bit 24, 16bit, scale=1)
   // Motor2 Torque: 35Nm (bit 192, 14bit, scale=0.125) -> 35/0.125=280=0x118
   {
       .id = 0x120,
       .dlc = CAN_DLC_32,
       .data = {0x00, 0x00, 0x00, 0x05, 0xDC, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // CLU_01_20ms (ID: 426 = 0x1AA, 16 bytes)
   // Display Speed: 65 km/h (bit 64, 9bit, scale=1)
   {
       .id = 0x1AA,
       .dlc = CAN_DLC_16,
       .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // IMU_01_10ms / YRS_01_10ms (ID: 74 = 0x4A, 32 bytes)
   // Yaw Rate: 0 deg/s (bit 64, 16bit, scale=0.005, offset=-163.84)
   //   -> (0+163.84)/0.005 = 32768 = 0x8000
   // Long Accel: 0.15g (bit 96, 16bit, scale=0.00012746, offset=-4.17677312)
   //   -> (0.15+4.17677312)/0.00012746 = 33949 = 0x849D
   {
       .id = 0x4A,
       .dlc = CAN_DLC_32,
       .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x80, 0x00, 0x00, 0x9D, 0x84, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // ICU_02_200ms (ID: 1041 = 0x411, 8 bytes)
   // Seatbelt Status: All buckled (value=2 for each 2-bit signal)
   // Bits: 36(Asst), 42(Drv), 50(RrCtr), 54(RrLft), 58(RrRt)
   {
       .id = 0x411,
       .dlc = CAN_DLC_8,
       .data = {0x00, 0x00, 0x00, 0x00, 0x20, 0x0A, 0x8A, 0x00}
   },
   // BCM_10_200ms (ID: 1058 = 0x422, 8 bytes)
   // Wiper Parking Position: 1 (bit 16, 2-bit, scale=0.5)
   {
       .id = 0x422,
       .dlc = CAN_DLC_8,
       .data = {0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
   // CLU_02_100ms (ID: 549 = 0x225, 16 bytes)
   // Odometer: 5432.1 km (bit 72, 24bit, scale=0.1)
   //   -> 5432.1/0.1 = 54321 = 0xD431
   {
       .id = 0x225,
       .dlc = CAN_DLC_16,
       .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x31, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00}
   },
    // DATC_01_20ms (ID: 325 = 0x145, 32 bytes)
    // Outside Temperature: 20C (bit 168=byte21, 8bit, scale=0.5, offset=-40)
    //   -> (20+40)/0.5 = 120 = 0x78
    {
        .id = 0x145,
        .dlc = CAN_DLC_32,
        .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    }
};

#define SAMPLE_MSG_COUNT (sizeof(sample_messages) / sizeof(sample_messages[0]))

// TX 관련 정적 변수들
static uint32_t tx_counter = 0;
static uint32_t tx_fail_count = 0;
static uint8_t tx_seq_counter[SAMPLE_MSG_COUNT] = {0};
static uint32_t next_msg_index = 0;  // Round-robin starting index
static volatile bool tx_timer_flag = false;  // Timer interrupt flag
static swtimer_handle_t tx_timer_handle;

// Timer callback function (called from interrupt)
static void can_tx_timer_callback(void *arg)
{
    (void)arg;
    tx_timer_flag = true;
}

// CAN TX 초기화
bool can_tx_init(void)
{
    DEBUG_PRINT("CAN TEST MODE: TX (Transmitter) with UDS Support\r\n");
    DEBUG_PRINT("Will send %d sample CAN FD messages every 10ms\r\n", SAMPLE_MSG_COUNT);
    DEBUG_PRINT("UDS Request: 0x7D4 | 03 22 01 01 55 55 55 55\r\n");
    DEBUG_PRINT("UDS Response: 0x7DC | Multi-frame sequence\r\n");
    
    tx_counter = 0;
    tx_fail_count = 0;
    
    // UDS 핸들러 초기화
    if (!uds_init()) {
        DEBUG_PRINT("UDS initialization failed\r\n");
        return false;
    }
    
    // Setup 10ms timer for CAN TX
    tx_timer_handle = swtimerGetHandle();
    if (tx_timer_handle >= 0)
    {
        swtimerSet(tx_timer_handle, CAN_TX_INTERVAL_MS, LOOP_TIME, can_tx_timer_callback, NULL);
        swtimerStart(tx_timer_handle);
        DEBUG_PRINT("CAN TX Timer started: %dms interval\r\n", CAN_TX_INTERVAL_MS);
    }
    else
    {
        DEBUG_PRINT("Failed to get timer handle\r\n");
        return false;
    }
    
    return true;
}

// CAN TX 처리
void can_tx_process(void)
{
    // UDS 핸들러 처리 (우선순위가 높음)
    uds_process();
    
    // CAN RX 메시지 확인 및 UDS 처리
    if (canMsgAvailable(_DEF_CAN1))
    {
        can_msg_t rx_msg;
        if (canMsgRead(_DEF_CAN1, &rx_msg))
        {
            can_tx_handle_rx_message(&rx_msg);
            ledToggle(HW_LED_CH_RX);
        }
    }
    
    // UDS가 활성화되어 있지 않을 때만 샘플 메시지 송신
    if (!uds_is_active() && tx_timer_flag)
    {
        tx_timer_flag = false;  // Clear flag
        
        uint32_t cycle_start = millis();
        uint32_t sent_this_cycle = 0;

        // Try to send all messages this cycle
        for (uint32_t i = 0; i < SAMPLE_MSG_COUNT; i++)
        {
            uint32_t msg_index = i;
                const sample_can_msg_t *sample = &sample_messages[msg_index];
                
                can_msg_t tx_msg;
                canMsgInit(&tx_msg, CAN_FD_BRS, CAN_STD, sample->dlc);
                tx_msg.id = sample->id;

                uint8_t data_length = canGetLen(sample->dlc);
                if (data_length == 0)
                {
                    continue;
                }

                for (int i = 0; i < data_length; i++)
                {
                    if (i < 4)
                    {
                        tx_msg.data[i] = sample->data[i]; // 필요 시 변화 데이터 삽입 가능
                    }
                    else
                    {
                        tx_msg.data[i] = sample->data[i];
                    }
                }

                if (data_length > 0)
                {
                    tx_msg.data[0] = tx_seq_counter[msg_index];
                }

                if (canMsgWrite(_DEF_CAN1, &tx_msg, 2))  // 2ms timeout to wait for FIFO space
                {
                    tx_counter++;
                    tx_seq_counter[msg_index]++;  // Only increment on success
                    sent_this_cycle++;
                    //DEBUG_PRINT("[%lu] OK ID=0x%03lX, cnt=0x%02X\r\n", millis(), tx_msg.id, tx_msg.data[0]);
                    ledToggle(HW_LED_CH_TX);
                    tx_fail_count = 0;
                }
                else
                {
                    // Failed to send - will retry next cycle with same counter
                    DEBUG_PRINT("[%lu ms] TX Failed! ID=0x%03lX\r\n", millis(), sample->id);
                }
        }
            
        uint32_t cycle_end = millis();
        DEBUG_PRINT("[%lu ms] TX Cycle: sent=%lu/%lu, duration=%lu ms\r\n", 
                    cycle_end, sent_this_cycle, (uint32_t)SAMPLE_MSG_COUNT, cycle_end - cycle_start);
    }
}

// CAN RX 메시지 처리 (UDS 용)
void can_tx_handle_rx_message(const can_msg_t *msg)
{
    DEBUG_PRINT("[RX] ID=0x%03lX, DLC=%d, Data=", msg->id, msg->length);
    for (int i = 0; i < msg->length && i < 8; i++) {
        DEBUG_PRINT("%02X ", msg->data[i]);
    }
    DEBUG_PRINT("\r\n");
    
    // UDS 요청 처리
    if (uds_handle_request(msg)) {
        DEBUG_PRINT("[TX] UDS request handled successfully\r\n");
        return;
    }
    
    // UDS Flow Control 처리
    if (uds_check_flow_control(msg)) {
        DEBUG_PRINT("[TX] UDS Flow Control handled successfully\r\n");
        return;
    }
    
    // 기타 메시지는 로그만 출력
    DEBUG_PRINT("[TX] Message not UDS-related\r\n");
}
