#include "ap.h"
#include "thread/boot/boot.h"
#include "thread/cmd/cmd_thread.h"
#include "comm_handler.h"
#include "can_manager.h"
#include "uart.h"

bool is_run_fw = true;
bool is_update_fw = false;
bool is_can_mode = true;  // 간단한 CAN 모드

// CAN 테스트 모드 설정
// ===============================================
// TX 테스트용 보드: #define CAN_TEST_MODE_TX true
// RX 테스트용 보드: #define CAN_TEST_MODE_TX false  
// ===============================================
#define CAN_TEST_MODE_TX    false   // true: TX 송신 모드, false: RX 수신 모드
bool can_test_tx_mode = CAN_TEST_MODE_TX;

// CAN 모드 설정
// ===============================================
// 0: CAN_ALL_LISTEN - 모든 CAN ID 수신 (기본 주기: 1000ms)
// 1: CAN_FILTER_LISTEN - 필터링된 CAN ID만 수신
// 2: CAN_RELEASE - 필터링된 CAN ID를 프로토콜로 변환하여 송신
// 3: CAN_DEBUG_MODE - 디버깅 모드 (CAN과 무관하게 UART만 출력)
// ===============================================
#define CAN_MODE_SETTING    CAN_RELEASE

// UART 테스트 함수
void uart_test_send_message(void)
{
#if _DEF_DEBUG_UART_TEST_ENABLE
  static uint32_t test_counter = 0;
  static uint32_t last_time = 0;
  
  // 1초마다 테스트 메시지 전송
  if (millis() - last_time >= 1000)
  {
    last_time = millis();
    test_counter++;
    
    // 간단한 테스트 메시지 전송
    all_printf("UART TEST: Counter=%lu, Time=%lu ms\r\n", test_counter, millis());
    
    // UART DEBUG 채널로 직접 테스트
    if (uartIsOpen(HW_UART_CH_DEBUG))
    {
      uartPrintf(HW_UART_CH_DEBUG, "UART DEBUG TEST: %lu\r\n", test_counter);
    }
    
    // UART EXT 채널로 직접 테스트
    if (uartIsOpen(HW_UART_CH_EXT))
    {
      uartPrintf(HW_UART_CH_EXT, "UART EXT TEST: %lu\r\n", test_counter);
    }
  }
#endif
}

void apInit(void)
{
  uint32_t boot_param;
  uint16_t err_code;

  // 강제 부트 모드 플래그 클리어
  resetSetBootMode(0);

  boot_param = resetGetBootMode();

  // CAN 모드 초기화
  if (is_can_mode)
  {
    logPrintf("DEBUG: Initializing CAN mode\r\n");
    // UART 초기화 - 모든 채널 열기
    uartOpen(HW_UART_CH_DEBUG, 115200);  // USART1 DEBUG
    uartOpen(HW_UART_CH_EXT, 115200);    // USART3 EXT
    // uartOpen(_DEF_UART2, 115200);     // USART2 RS485 (필요시 활성화)
    
    all_printf("Starting CAN Application...\r\n");
    all_printf("USB CDC Status: %s\r\n", cdcIsConnect() ? "Connected" : "Not Connected");
    all_printf("Press BOOT button for bootloader\r\n");
    all_printf("Press S2 button to switch CAN modes\r\n");
    
    // UART 테스트 모드 안내
#if _DEF_DEBUG_UART_TEST_ENABLE
    all_printf("UART Test Mode: ENABLED\r\n");
    all_printf("Test messages will be sent to UART DEBUG and UART EXT channels every 1 second\r\n");
    
    // UART 상태 확인
    all_printf("UART DEBUG Status: %s\r\n", uartIsOpen(HW_UART_CH_DEBUG) ? "OPEN" : "CLOSED");
    all_printf("UART EXT Status: %s\r\n", uartIsOpen(HW_UART_CH_EXT) ? "OPEN" : "CLOSED");
    
    // 즉시 테스트 메시지 전송
    uartPrintf(HW_UART_CH_DEBUG, "UART DEBUG INIT TEST\r\n");
    uartPrintf(HW_UART_CH_EXT, "UART EXT INIT TEST\r\n");
#else
    all_printf("UART Test Mode: DISABLED\r\n");
#endif
    
    // CAN 모드 설정
    current_can_mode = CAN_MODE_SETTING;
    
    // CAN 모드별 안내 메시지
    switch (current_can_mode)
    {
        case CAN_ALL_LISTEN:
            all_printf("CAN Mode: ALL LISTEN (모든 CAN ID 수신)\r\n");
            all_printf("출력 주기: 1000ms\r\n");
            break;
        case CAN_FILTER_LISTEN:
            all_printf("CAN Mode: FILTER LISTEN (모든 CAN ID 수신, 매핑된 ID는 설정 주기)\r\n");
            break;
        case CAN_RELEASE:
            all_printf("CAN Mode: RELEASE (프로토콜 변환 송신)\r\n");
            break;
        case CAN_DEBUG_MODE:
            all_printf("CAN Mode: DEBUG MODE (디버깅 모드)\r\n");
            all_printf("출력 주기: 1000ms\r\n");
            break;
        case CAN_SCAN_MODE:
            all_printf("CAN Mode: SCAN MODE (CAN ID만 확인)\r\n");
            all_printf("출력 주기: 1000ms\r\n");
            break;
        default:
            all_printf("CAN Mode: UNKNOWN\r\n");
            break;
    }
    
    // CAN 매니저 초기화 (TX 모드로 설정)
    if (can_manager_init(can_test_tx_mode))
    {
        all_printf("CAN mode ready\r\n");
    }
    else
    {
        logPrintf("DEBUG: CAN mode initialization failed, switching to bootloader\r\n");
        all_printf("CAN mode initialization failed\r\n");
        is_can_mode = false;
    }
  }
  else
  {
    logPrintf("DEBUG: Entering bootloader mode\r\n");
    logPrintf("Bootloader mode\n");  
  }
}

void apMain(void)
{
  uint32_t pre_time;
  bool can_mode_prev = is_can_mode;
  bool button_prev = false;
  bool s2_button_prev = false;

  cmdThreadInit();
  pre_time = millis();
  while(1)
  {
    // UART TEST MODE는 항상 동작
    uart_test_send_message();

    // BOOT 버튼 상태 디버깅
    bool button_current = buttonGetPressed(HW_BUTTON_CH_BOOT);
    if (button_current != button_prev)
    {
      if (button_current)
      {
        all_printf("DEBUG: BOOT button PRESSED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
        uartPrintf(HW_UART_CH_DEBUG, "DEBUG: BOOT button PRESSED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
        uartPrintf(HW_UART_CH_EXT, "DEBUG: BOOT button PRESSED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
      }
      else
      {
        all_printf("DEBUG: BOOT button RELEASED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
        uartPrintf(HW_UART_CH_DEBUG, "DEBUG: BOOT button RELEASED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
        uartPrintf(HW_UART_CH_EXT, "DEBUG: BOOT button RELEASED, is_can_mode=%s\r\n", is_can_mode ? "true" : "false");
      }
      button_prev = button_current;
    }

    // S2 버튼을 사용한 CAN 모드 전환
    bool s2_button_current = buttonGetPressed(HW_BUTTON_CH_S2);
    if (s2_button_current && !s2_button_prev)
    {
      if (is_can_mode)
      {
        all_printf("S2 Button Pressed: CAN Mode Switching...\r\n");
        can_mode_switch();
      }
      else
      {
        all_printf("S2 Button Pressed: Not in CAN mode\r\n");
      }
    }
    s2_button_prev = s2_button_current;

    if (is_can_mode)
    {
      // CAN 애플리케이션 모드
      can_manager_process();
      
      // 부트로더 모드로 전환 체크
      if (buttonGetPressed(HW_BUTTON_CH_BOOT) == true)
      {
        all_printf("DEBUG: Switching to bootloader (button pressed)\r\n");
        uartPrintf(HW_UART_CH_DEBUG, "DEBUG: Switching to bootloader (button pressed)\r\n");
        uartPrintf(HW_UART_CH_EXT, "DEBUG: Switching to bootloader (button pressed)\r\n");
        is_can_mode = false;
        continue;
      }
      
      // 상태 LED (느리게)
      if (millis() - pre_time >= 2000)
      {
        pre_time = millis();
        ledToggle(HW_LED_CH_DEBUG);
      }
    }
    else
    {
      // can_mode가 false로 바뀌는 순간 indication
      if (can_mode_prev != is_can_mode)
      {
        uartPrintf(HW_UART_CH_DEBUG, "CAN MODE OFF\r\n");
        uartPrintf(HW_UART_CH_EXT,   "CAN MODE OFF\r\n");
        can_mode_prev = is_can_mode;
      }
      // 부트로더 모드 (기존 로직)
      if (millis()-pre_time >= 1000)
      {
        pre_time = millis();
        ledToggle(HW_LED_CH_DEBUG);
      }    
      if (cmdThreadUpdate() == true)
      {
        ledToggle(HW_LED_CH_DOWN);
      }
    }
  }
}

