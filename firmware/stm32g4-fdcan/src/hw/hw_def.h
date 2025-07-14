#ifndef HW_DEF_H_
#define HW_DEF_H_



#include "bsp.h"


#define _DEF_FIRMWARE_VERSION    "I240711R1"  // 통합 버전 (UART+CAN+Boot)
#define _DEF_BOARD_NAME          "STM32G4-FDCAN-SIMPLE"




#define _USE_HW_FLASH
#define _USE_HW_FAULT
#define _USE_HW_MINI_PRINTF
#include "mini-printf.h"


// LED (상태 표시용)
#define _USE_HW_LED
#define      HW_LED_MAX_CH          5
#define      HW_LED_CH_DEBUG        _DEF_LED1
#define      HW_LED_CH_RX           _DEF_LED2
#define      HW_LED_CH_TX           _DEF_LED3
#define      HW_LED_CH_CAN          _DEF_LED4
#define      HW_LED_CH_RS485        _DEF_LED5
#define      HW_LED_CH_DOWN         HW_LED_CH_RX
#define      HW_LED_CH_UPDATE       HW_LED_CH_TX
#define      HW_LED_CH_FAULT        HW_LED_CH_RS485

// UART (디버그 출력용)
#define _USE_HW_UART
#define      HW_UART_MAX_CH        4
#define      HW_UART_CH_DEBUG       _DEF_UART1
#define      HW_UART_CH_EXT         _DEF_UART3
#define      HW_UART_CH_USB         _DEF_UART4

#define _USE_HW_LOG
#define      HW_LOG_CH              _DEF_UART1
#define      HW_LOG_BOOT_BUF_MAX    1024
#define      HW_LOG_LIST_BUF_MAX    1024

// 부트 버튼
#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       3
#define      HW_BUTTON_CH_BOOT      _DEF_BUTTON1
#define      HW_BUTTON_CH_S1        _DEF_BUTTON2
#define      HW_BUTTON_CH_S2        _DEF_BUTTON3

#define _USE_HW_SWTIMER
#define      HW_SWTIMER_MAX_CH      4

// CAN (핵심 기능)
#define _USE_HW_CAN
#define      HW_CAN_MAX_CH          1
#define      HW_CAN_MSG_RX_BUF_MAX  32

// 부트로더 필수
#define _USE_HW_RESET
#define      HW_RESET_BOOT          1

#define _USE_HW_RTC
#define      HW_RTC_BOOT_MODE       RTC_BKP_DR3
#define      HW_RTC_RESET_BITS      RTC_BKP_DR4

#define _USE_HW_CMD
#define      HW_CMD_MAX_DATA_LENGTH 1024

// USB CDC (Virtual COM Port)  
#define _USE_HW_USB
#define _USE_HW_CDC
#define      HW_USE_CDC             1
#define      HW_USE_MSC             0

// #define _USE_HW_SPI_FLASH  // Not needed for simple version  
// #define      HW_SPI_FLASH_ADDR      0x90000000


// 메모리 맵 (전체 플래시 사용)
#define FLASH_SIZE_TAG              0x400
#define FLASH_SIZE_VER              0x400
#define FLASH_SIZE_FIRM             (126*1024)

#define FLASH_ADDR_BOOT             0x08000000
#define FLASH_ADDR_FIRM             0x08006000

#define FLASH_ADDR_UPDATE           0x90000000


#endif