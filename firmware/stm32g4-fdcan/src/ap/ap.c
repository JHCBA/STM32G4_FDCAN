#include "ap.h"
#include "thread/boot/boot.h"
#include "thread/cmd/cmd_thread.h"
#include "comm_handler.h"
#include "can_manager.h"

bool is_run_fw = true;
bool is_update_fw = false;
bool is_can_mode = false;  // 간단한 CAN 모드

// CAN 테스트 모드 설정
// ===============================================
// TX 테스트용 보드: #define CAN_TEST_MODE_TX true
// RX 테스트용 보드: #define CAN_TEST_MODE_TX false  
// ===============================================
#define CAN_TEST_MODE_TX    false   // true: TX 송신 모드, false: RX 수신 모드
bool can_test_tx_mode = CAN_TEST_MODE_TX;

void apInit(void)
{
  uint32_t boot_param;
  uint16_t err_code;

  boot_param = resetGetBootMode();

  // 부트로더 모드 진입 조건
  if (boot_param & (1<<MODE_BIT_BOOT))
  {
    boot_param &= ~(1<<MODE_BIT_BOOT);
    resetSetBootMode(boot_param);
    
    is_run_fw = false;
    is_can_mode = false;
  }
  else if (buttonGetPressed(HW_BUTTON_CH_BOOT) == true)
  {
    is_run_fw = false;
    is_can_mode = false;
  }
  else
  {
    // 기본적으로 CAN 모드로 시작
    is_run_fw = false;
    is_can_mode = true;
  }

  if (boot_param & (1<<MODE_BIT_UPDATE))
  {
    boot_param &= ~(1<<MODE_BIT_UPDATE);
    resetSetBootMode(boot_param);
    
    is_run_fw = true;
    is_update_fw = true;
    is_can_mode = false;
  }
  
  logPrintf("\n");
  logPrintf("STM32G4 FDCAN Simple Boot+App\n");
  logPrintf("Version: %s\n", _DEF_FIRMWARE_VERSION);

  // 업데이트 처리
  if (is_update_fw)
  {
    logPrintf("[  ] bootUpdateFirm()\r");
    err_code = bootUpdateFirm();
    logPrintf("[%s]\n", err_code==CMD_OK ? "OK":"NG");
    if (err_code != CMD_OK)
      logPrintf("     err : 0x%04X\n", err_code);
  }

  // 에러 상태 확인
  if (faultIsReady())
  {
    logPrintf("[  ] fault ready\n");
    ledOn(HW_LED_CH_FAULT);
    is_run_fw = false;
    is_can_mode = false;
  }

  // 펌웨어 점프 (사용하지 않음)
  if (is_run_fw)
  {
    logPrintf("[  ] bootJumpFirm()\r");
    err_code = bootJumpFirm();
    logPrintf("[%s]\n", err_code==CMD_OK ? "OK":"NG");
    if (err_code != CMD_OK)
      logPrintf("     err : 0x%04X\n", err_code);
  }

  // CAN 모드 초기화
  if (is_can_mode)
  {
    // UART 초기화
    uartOpen(HW_UART_CH_DEBUG, 115200);
    
    all_printf("Starting CAN Application...\r\n");
    all_printf("USB CDC Status: %s\r\n", cdcIsConnect() ? "Connected" : "Not Connected");
    all_printf("Press BOOT button for bootloader\r\n");
    
    // CAN 매니저 초기화 (TX 모드로 설정)
    if (can_manager_init(can_test_tx_mode))
    {
        all_printf("CAN mode ready\r\n");
    }
    else
    {
        all_printf("CAN mode initialization failed\r\n");
        is_can_mode = false;
    }
  }
  else
  {
    logPrintf("Bootloader mode\n");  
  }
}

void apMain(void)
{
  uint32_t pre_time;
  
  cmdThreadInit();

  pre_time = millis();
  while(1)
  {
    if (is_can_mode)
    {
      // CAN 애플리케이션 모드
      can_manager_process();
      
      // 부트로더 모드로 전환 체크
      if (buttonGetPressed(HW_BUTTON_CH_BOOT) == true)
      {
        all_printf("Switching to bootloader...\r\n");
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

