#include "ap.h"

static uint32_t counter = 0;
static uint32_t pre_time = 0;
static uint32_t rx_counter = 0;
static uint32_t init_time = 0;
static bool is_initialized = false;
#define FILTERED_ID_COUNT 5

const uint32_t filter_ids[FILTERED_ID_COUNT] = {0xA0, 0x60, 0x100, 0xEA, 0x125};

// 해당 ID가 필터 대상인지 확인하는 함수
bool is_filtered_id(uint32_t id) {
    for (int i = 0; i < FILTERED_ID_COUNT; i++) {
        if (filter_ids[i] == id)
            return true;
    }
    return false;
}


// CAN 메시지 송신을 위한 구조체 정의
typedef struct {
    uint32_t id;
    CanDlc_t dlc;
    uint32_t period;
    uint32_t last_time;
} can_msg_config_t;

void SendUARTALL(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uartPrintf(HW_UART_CH_USB, buf);
    uartPrintf(HW_UART_CH_EXT, buf);
}

void SendUARTEXT(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uartPrintf(HW_UART_CH_EXT, buf);
}

void apInit(void)
{
  threadInit();
  
  // Initialize UART for debug output
  uartOpen(HW_UART_CH_USB, 115200);
  uartOpen(HW_UART_CH_EXT, 115200);
  
  // Initialize CAN FD
	canInit();
  if (canOpen(_DEF_CAN1, CAN_MONITOR, CAN_FD_BRS, CAN_500K, CAN_2M) == false)
  {
	  SendUARTALL("CAN FD initialization failed\r\n");
  }
  else
  {
	  SendUARTALL("CAN FD initialized in normal mode\r\n");
  }
}

// CAN 메시지 송신 함수
void sendCanMessage(can_msg_t *msg, uint32_t id, CanDlc_t dlc, uint32_t counter)
{
    msg->id = id;
    msg->dlc = dlc;
    
    // 데이터 채우기
    for(int i = 0; i < dlc; i++)
    {
        msg->data[i] = (i * counter) & 0xFF;
    }
    
    // 메시지 송신
    (void)canMsgWrite(_DEF_CAN1, msg, 100);
}

void apMain(void)
{
    can_msg_t can_msg;
    static uint32_t button_count = 0;
    uint8_t is_tx = 0;
    
    // For RX board: store latest messages for each ID
    #define MAX_MSG_IDS 100  // 최대 저장할 ID 개수
    static struct {
        uint32_t id;
        can_msg_t msg;
        bool has_new_msg;
    } latest_msgs[MAX_MSG_IDS] = {0};
    static uint32_t rx_print_time = 0;
    
    // CAN 메시지 설정 배열
    static can_msg_config_t msg_configs[] = {
        {0x801, CAN_DLC_8,  10, 0},  // 10ms, DLC 8
        {0x7df, CAN_DLC_16, 10, 0},  // 10ms, DLC 16
        {0x712, CAN_DLC_32, 100, 0}, // 100ms, DLC 32
        {0x1BA, CAN_DLC_32, 10, 0}   // 10ms, DLC 32
    };
    const uint8_t num_configs = sizeof(msg_configs) / sizeof(msg_configs[0]);
    
    while(1)
    {
        threadUpdate();
        
        if (is_tx == 1)
        {
            uint32_t current_time = millis();
            
            // 각 메시지 설정에 대해 주기적으로 송신
            for(int i = 0; i < num_configs; i++)
            {
                if (current_time - msg_configs[i].last_time >= msg_configs[i].period)
                {
                    msg_configs[i].last_time = current_time;
                    sendCanMessage(&can_msg, msg_configs[i].id, msg_configs[i].dlc, button_count);
                }
            }
            
            button_count++;
        }
        else
        {
            // RX board: store latest messages for each ID
            if (canMsgAvailable(_DEF_CAN1))
            {
                can_msg_t rx_msg;
                canMsgRead(_DEF_CAN1, &rx_msg);

                // Find or create slot for this ID
                int slot = -1;
                for(int i = 0; i < MAX_MSG_IDS; i++)
                {
                    if (latest_msgs[i].id == rx_msg.id || latest_msgs[i].id == 0)
                    {
                        slot = i;
                        break;
                    }
                }

                if (slot != -1)
                {
                    latest_msgs[slot].id = rx_msg.id;
                    latest_msgs[slot].msg = rx_msg;
                    latest_msgs[slot].has_new_msg = true;
                }
            }

            // Print all updated messages every second
            if (millis() - rx_print_time >= 100)
            {
                rx_print_time = millis();

                for (int i = 0; i < MAX_MSG_IDS; i++)
                {
                    if (latest_msgs[i].has_new_msg)
                    {
                        // 필터링된 ID만 출력
                        if (!is_filtered_id(latest_msgs[i].msg.id))
                            continue;

                        char rx_buf[256];
                        int len = 0;

                        len = snprintf(rx_buf, sizeof(rx_buf), "CAN FD RX: ID=0x%lX, DLC=%d, Data=",
                                       latest_msgs[i].msg.id, latest_msgs[i].msg.length);

                        for (int j = 0; j < latest_msgs[i].msg.length; j++)
                        {
                            if (len < sizeof(rx_buf) - 4)
                            {
                                len += snprintf(rx_buf + len, sizeof(rx_buf) - len, "%02X ", latest_msgs[i].msg.data[j]);
                            }
                        }

                        snprintf(rx_buf + len, sizeof(rx_buf) - len, "\r\n");

                        SendUARTALL("%s", rx_buf);

                        latest_msgs[i].has_new_msg = false;
                    }
                }
            }
        }
    }
}

