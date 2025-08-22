#include "ap.h"
#include "thread/boot/boot.h"
#include "thread/cmd/cmd_thread.h"
#include "comm_handler.h"
#include "can_manager.h"
#include "uart.h"

bool is_run_fw = true;
bool is_update_fw = false;
bool is_can_mode = true;  // UDS 전용 CAN 모드

// 모드 관리용 변수들

static uds_mode_t current_mode = UDS_MODE_NORMAL;
static uint32_t last_button_time = 0;

// 모드 상태를 가져오는 함수
uds_mode_t get_current_mode(void)
{
    return current_mode;
}

// 모드 변경 함수
void change_mode(void)
{
    current_mode = (current_mode + 1) % UDS_MODE_MAX;
    
    const char* mode_names[] = {"NORMAL", "STEERING"};
    all_printf("\r\n[MODE CHANGE] Current Mode: %s\r\n", mode_names[current_mode]);
    
    if (current_mode == UDS_MODE_STEERING) {
        all_printf("[STEERING MODE] Only BYTE 7-8 extraction and UART transmission\r\n");
    } else {
        all_printf("[NORMAL MODE] Full UDS message analysis\r\n");
    }
}

void apInit(void)
{
  uint32_t boot_param;
  uint16_t err_code;

  // 강제 부트 모드 플래그 클리어
  resetSetBootMode(0);

  boot_param = resetGetBootMode();

  // UDS CAN 모드 초기화
  if (is_can_mode)
  {
    logPrintf("DEBUG: Initializing UDS CAN mode\r\n");
    // UART 초기화 - 디버깅용 및 스티어링 컬럼 통신용
    uartOpen(HW_UART_CH_DEBUG, 115200);  // USART1 DEBUG
    uartOpen(HW_UART_CH_EXT, 115200);    // USART3 EXT - Steering Column 통신용
    
    all_printf("Starting UDS Application...\r\n");
    all_printf("USB CDC Status: %s\r\n", cdcIsConnect() ? "Connected" : "Not Connected");
    all_printf("Press BOOT button for bootloader\r\n");
    all_printf("Press BUTTON 4 (S2) to change mode\r\n");
    all_printf("UDS Message Mode: Receive and Debug Output\r\n");
    all_printf("Steering Column UART: USART3 (115200 bps)\r\n");
    all_printf("Current Mode: NORMAL (Full UDS analysis)\r\n");
    
    // UART 상태 확인
    all_printf("UART DEBUG Status: %s\r\n", uartIsOpen(HW_UART_CH_DEBUG) ? "OPEN" : "CLOSED");
    all_printf("UART EXT (Steering) Status: %s\r\n", uartIsOpen(HW_UART_CH_EXT) ? "OPEN" : "CLOSED");
    
    // 스티어링 컬럼 초기화 메시지 전송
    if (uartIsOpen(HW_UART_CH_EXT)) {
        uartPrintf(HW_UART_CH_EXT, "STEERING_INIT:OK\r\n");
    }
    
    // UDS CAN 매니저 초기화 (RX 전용 모드)
    if (can_manager_init(false))  // false = RX 모드
    {
        all_printf("UDS CAN mode ready\r\n");
    }
    else
    {
        logPrintf("DEBUG: UDS CAN mode initialization failed, switching to bootloader\r\n");
        all_printf("UDS CAN mode initialization failed\r\n");
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
  bool button_s2_prev = false;

  cmdThreadInit();
  pre_time = millis();
  while(1)
  {
    // BOOT 버튼 상태 확인 (부트로더 모드 전환용)
    bool button_current = buttonGetPressed(HW_BUTTON_CH_BOOT);
    if (button_current != button_prev)
    {
      if (button_current)
      {
        all_printf("DEBUG: BOOT button PRESSED, switching to bootloader\r\n");
      }
      button_prev = button_current;
    }

    // BUTTON 4 (S2) 상태 확인 (모드 변경용)
    bool button_s2_current = buttonGetPressed(HW_BUTTON_CH_S2);
    if (button_s2_current != button_s2_prev)
    {
      if (button_s2_current && is_can_mode)
      {
        // 디바운싱을 위한 시간 체크
        if (millis() - last_button_time > 500) // 500ms 디바운싱
        {
          change_mode();
          last_button_time = millis();
        }
      }
      button_s2_prev = button_s2_current;
    }

    if (is_can_mode)
    {
      // UDS CAN 애플리케이션 모드
      can_manager_process();
      
      // 부트로더 모드로 전환 체크
      if (buttonGetPressed(HW_BUTTON_CH_BOOT) == true)
      {
        all_printf("DEBUG: Switching to bootloader (button pressed)\r\n");
        is_can_mode = false;
        continue;
      }
      
      // 상태 LED (UDS 동작 표시)
      if (millis() - pre_time >= 2000)
      {
        pre_time = millis();
        ledToggle(HW_LED_CH_DEBUG);
      }
    }
    else
    {
      // UDS 모드가 종료되는 순간 표시
      if (can_mode_prev != is_can_mode)
      {
        all_printf("UDS MODE OFF - Switching to Bootloader\r\n");
        can_mode_prev = is_can_mode;
      }
      // 부트로더 모드
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

