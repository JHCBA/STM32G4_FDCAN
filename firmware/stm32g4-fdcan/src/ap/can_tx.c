#include "can_tx.h"
#include "can_manager.h"
#include "can_db.h"

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
    // {
    //     .id = 0x3E3,
    //     .dlc = CAN_DLC_8,
    //     .data = {0x7E, 0x41, 0xBB, 0x00, 0x01, 0x41, 0x00, 0x00}
    // },
    // {
    //     .id = 0xEA,
    //     .dlc = CAN_DLC_24,
    //     .data = {0x7E, 0x41, 0xBB, 0x00, 0x01, 0x41, 0x00, 0x00, 
    //              0x01, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 
    //              0xAC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    // },
	 {
		 .id = 0xA0,
		 .dlc = CAN_DLC_24,
		 .data = {0x7E, 0x41, 0xBB, 0x00, 0x01, 0x41, 0x00, 0x00,
				  0x01, 0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
				  0xAC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	 },
    // {
    //     .id = 0x125,
    //     .dlc = CAN_DLC_16,
    //     .data = {0x3E, 0x9F, 0xB3, 0xBB, 0xFF, 0x00, 0x07, 0x00, 
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    // },
    // {
    //     .id = 0x60,
    //     .dlc = CAN_DLC_32,
    //     .data = {0x2B, 0x59, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 
    //              0x02, 0x02, 0x00, 0x02, 0x00, 0xFF, 0x00, 0xFF, 
    //              0xAD, 0x00, 0x15, 0x00, 0x00, 0x08, 0x03, 0x01, 
    //              0x40, 0x00, 0x00, 0x04, 0xFF, 0xFA, 0x00, 0x00}
    // },
    // {
    //     .id = 0x100,
    //     .dlc = CAN_DLC_32,
    //     .data = {0xED, 0xA6, 0xF6, 0x00, 0x11, 0xAD, 0x7E, 0x6C, 
    //              0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x57, 
    //              0x23, 0x00, 0x48, 0x00, 0xE5, 0x00, 0x00, 0x00, 
    //              0xFE, 0x0F, 0x00, 0x90, 0x93, 0x00, 0x00, 0x00}
    // },
    // UDS 진단 메시지들 (순서대로 정렬)
//    {
//        .id = 0x7E1,  // [1] Diagnostic Request
//        .dlc = CAN_DLC_8,
//        .data = {0x03, 0x22, 0xC1, 0x01, 0x00, 0x00, 0x00, 0x00}
//    },
//    {
//        .id = 0x7E9,  // [2] First Frame Response
//        .dlc = CAN_DLC_8,
//        .data = {0x10, 0x2A, 0x62, 0xC1, 0x01, 0x48, 0xD7, 0xE7}
//    },
//    {
//        .id = 0x7E1,  // [3] Flow Control
//        .dlc = CAN_DLC_8,
//        .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
//    },
//    {
//        .id = 0x7E9,  // [4] Consecutive Frame 1
//        .dlc = CAN_DLC_8,
//        .data = {0x21, 0x00, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00}
//    },
//    {
//        .id = 0x7E9,  // [5] Consecutive Frame 2
//        .dlc = CAN_DLC_8,
//        .data = {0x22, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF}
//    },
//    {
//        .id = 0x7E9,  // [6] Consecutive Frame 3
//        .dlc = CAN_DLC_8,
//        .data = {0x23, 0xFF, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00}
//    },
//    {
//        .id = 0x7E9,  // [7] Consecutive Frame 4
//        .dlc = CAN_DLC_8,
//        .data = {0x24, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00}
//    },
//    {
//        .id = 0x7E9,  // [8] Consecutive Frame 5
//        .dlc = CAN_DLC_8,
//        .data = {0x25, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFF}
//    },
//    {
//        .id = 0x7E9,  // [9] Consecutive Frame 6
//        .dlc = CAN_DLC_8,
//        .data = {0x26, 0xFF, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}
//    }
};

#define SAMPLE_MSG_COUNT (sizeof(sample_messages) / sizeof(sample_messages[0]))

// TX 관련 정적 변수들
static uint32_t tx_time = 0;
static uint32_t tx_counter = 0;
static uint32_t tx_fail_count = 0;

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
    if (millis() - tx_time >= 100)  // 100ms마다 송신 (순서 확인용)
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
                tx_msg.data[i] = sample->data[i];// ^ (tx_counter & 0xFF);
            }
            else
            {
                tx_msg.data[i] = sample->data[i];
            }
        }
        
        if (canMsgWrite(_DEF_CAN1, &tx_msg, 10))
        {
            DEBUG_PRINT("CAN TX [%lu/%d]: ID=0x%03lX, DLC=%d, Data=", 
                       msg_index + 1, SAMPLE_MSG_COUNT, tx_msg.id, data_length);
            for (int i = 0; i < data_length; i++)
            {
                DEBUG_PRINT("%02X ", tx_msg.data[i]);
            }
            DEBUG_PRINT("(Total=%lu)\r\n", tx_counter);
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
