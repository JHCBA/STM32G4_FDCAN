#include "ap.h"
#include <string.h>
#include <stdio.h>
#include "thread/boot/boot.h"
#include "thread/cmd/cmd_thread.h"
#include "comm_handler.h"
#include "can_manager.h"
#include "uart.h"
#include "cdc.h"

bool is_run_fw = true;
bool is_update_fw = false;
bool is_can_mode = true;  // UDS 전용 CAN 모드

// 모드 관리용 변수들
static uint32_t default_can_id = 0x7DF;  // 기본 CAN ID

static uds_mode_t current_mode = UDS_MODE_TALK;
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
    
    const char* mode_names[] = {"NORMAL", "UDS_PATH", "TALK"};
    all_printf("\r\n[MODE CHANGE] Current Mode: %s\r\n", mode_names[current_mode]);
    
    switch (current_mode) {
        case UDS_MODE_NORMAL:
            all_printf("[NORMAL MODE] Full UDS message analysis\r\n");
            break;
        case UDS_MODE_UDS_PATH:
            all_printf("[UDS_PATH MODE] UDS data extraction and UART transmission\r\n");
            break;
        case UDS_MODE_TALK:
            all_printf("[TALK MODE] Terminal direct CAN TX mode\r\n");
            all_printf("Commands:\r\n");
            all_printf("  id 7e2        - Set default CAN ID to 0x7E2\r\n");
            all_printf("  f             - Send quick macro: 30 08 02 00 00 00 00 00\r\n");
            all_printf("  30 08 02      - Send data using default ID\r\n");
            all_printf("  7e2:30 08     - Send data with specific ID (ID:data format)\r\n");
            
            // CAN 상태 확인
            uint32_t can_error = canGetError(_DEF_CAN1);
            uint16_t tx_err = canGetTxErrCount(_DEF_CAN1);
            uint16_t rx_err = canGetRxErrCount(_DEF_CAN1);
            bool can_open = canIsOpen(_DEF_CAN1);
            
            all_printf("CAN Status: %s | Error: 0x%lX | TX Err: %d | RX Err: %d\r\n", 
                      can_open ? "OPEN" : "CLOSED", can_error, tx_err, rx_err);
            all_printf("> ");
            break;
    }
}

// CAN 메시지 전송 함수
bool can_tx_message(uint32_t id, uint8_t *data, uint8_t length)
{
    can_msg_t tx_msg;
    
    // ID 타입 결정 (11-bit: STD, 29-bit: EXT)
    CanIdType_t id_type = (id <= 0x7FF) ? CAN_STD : CAN_EXT;
    
    // CAN 메시지 구성
    canMsgInit(&tx_msg, CAN_FD_BRS, id_type, canGetDlc(length));
    tx_msg.id = id;
    tx_msg.length = length;
    
    // 8바이트보다 적으면 8바이트로 패딩
    uint8_t actual_length = (length < 8) ? 8 : length;
    
    // 데이터 복사
    for (int i = 0; i < length && i < 64; i++) {
        tx_msg.data[i] = data[i];
    }
    
    // 8바이트까지 0x55로 패딩
    for (int i = length; i < actual_length; i++) {
        tx_msg.data[i] = 0x55;
    }
    
    // 나머지 바이트는 0으로 채움 (8바이트 이후)
    for (int i = actual_length; i < 64; i++) {
        tx_msg.data[i] = 0;
    }
    
    // DLC와 length를 실제 길이로 업데이트
    tx_msg.length = actual_length;
    tx_msg.dlc = canGetDlc(actual_length);
    
    all_printf("[CAN TX] ID: 0x%lX | Data: ", id);
    for (int i = 0; i < actual_length; i++) {
        all_printf("%02X ", tx_msg.data[i]);
    }
    all_printf("| DLC: %d | Length: %d (%s)\r\n", 
              tx_msg.dlc, actual_length, (id_type == CAN_STD) ? "STD" : "EXT");
    
    // CAN 전송
    if (canMsgWrite(_DEF_CAN1, &tx_msg, 100)) {
        all_printf("[TX SUCCESS] Message sent\r\n");
        return true;
    } else {
        all_printf("[TX ERROR] Failed to send CAN message\r\n");
        
        // 에러 정보 출력
        uint32_t can_error = canGetError(_DEF_CAN1);
        uint16_t tx_err_count = canGetTxErrCount(_DEF_CAN1);
        uint16_t rx_err_count = canGetRxErrCount(_DEF_CAN1);
        uint32_t can_state = canGetState(_DEF_CAN1);
        bool can_open = canIsOpen(_DEF_CAN1);
        
        all_printf("CAN Error: 0x%08lX, TX Err: %d, RX Err: %d\r\n", 
                  can_error, tx_err_count, rx_err_count);
        all_printf("CAN State: 0x%08lX, Open: %s\r\n", 
                  can_state, can_open ? "YES" : "NO");
        
        // CAN 복구 시도
        all_printf("Attempting CAN recovery...\r\n");
        canRecovery(_DEF_CAN1);
        
        return false;
    }
}

// CAN 데이터 파싱 함수 (개선됨: 항상 기본 ID 사용, 명시적 ID는 "ID:data" 형식으로)
bool parse_can_data(const char* input, uint32_t* id, uint8_t* data, uint8_t* length)
{
    const char* current = input;
    *length = 0;
    
    // 앞뒤 공백 제거
    while (*current == ' ' || *current == '\t') current++;
    if (*current == '\0') return false;
    
    // 콜론(:) 형식으로 ID 명시가 있는지 확인 (예: "7e2:30 08 02")
    const char* colon_pos = strchr(current, ':');
    
    if (colon_pos != NULL) {
        // 콜론이 있으면 "ID:data" 형식
        if (sscanf(current, "%lx", id) != 1) {
            all_printf("Invalid ID format. Use hex format (e.g., 7e2:30 08)\r\n");
            return false;
        }
        
        // 콜론 이후부터 데이터 파싱
        current = colon_pos + 1;
        while (*current == ' ' || *current == '\t') current++;  // 콜론 후 공백 제거
    } else {
        // 콜론이 없으면 모든 입력을 데이터로 처리하고 기본 ID 사용
        *id = get_default_can_id();
        
        // 기본 ID로 전송한다고 미리 표시
        all_printf("[TX ATTEMPT] ID: 0x%lX (STD) | Length: ", *id);
    }
    
    // 데이터 바이트들 파싱
    while (*current && *length < 64) {
        unsigned int byte_val;
        if (sscanf(current, "%x", &byte_val) == 1) {
            data[(*length)++] = (uint8_t)byte_val;
            
            // 다음 바이트로 이동
            while (*current && *current != ' ') current++;
            while (*current == ' ') current++;
        } else {
            break;
        }
    }
    
    if (*length == 0) {
        all_printf("No valid data bytes found\r\n");
        return false;
    }
    
    // 기본 ID 사용 시에만 데이터 길이와 패딩 표시
    if (colon_pos == NULL) {
        all_printf("%d->8 | Data: ", *length);
        for (int i = 0; i < *length; i++) {
            all_printf("%02X ", data[i]);
        }
        // 8바이트까지 패딩 표시
        for (int i = *length; i < 8; i++) {
            all_printf("55 ");
        }
        all_printf("\r\n");
    }
    
    return true;
}

// 매크로 명령 처리 함수
bool process_macro_command(const char* input)
{
    // "id xx" 명령 처리
    if (strncmp(input, "id ", 3) == 0) {
        uint32_t new_id;
        if (sscanf(input + 3, "%lx", &new_id) == 1) {
            set_default_can_id(new_id);
            all_printf("ID SET: %lX\r\n", new_id);
            return true;
        } else {
            all_printf("Invalid ID format. Use: id 7df\r\n");
            return true;
        }
    }
    
    // "f" 명령 처리
    if (strcmp(input, "f") == 0) {
        uint8_t data[] = {0x30, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint32_t id = get_default_can_id();
        all_printf("[TX ATTEMPT] ID: 0x%lX (STD) | Length: 8 | Data: 30 08 02 00 00 00 00 00\r\n", id);
        can_tx_message(id, data, 8);
        return true;
    }
    
    return false; // 매크로 명령이 아니므
}

// 기본 CAN ID 설정
void set_default_can_id(uint32_t id)
{
    default_can_id = id;
}

// 기본 CAN ID 가져오기
uint32_t get_default_can_id(void)
{
    return default_can_id;
}

// TALK 모드 입력 처리 함수
void process_talk_input(void)
{
    static char input_buffer[256];
    static uint8_t buffer_index = 0;
    static uint32_t debug_counter = 0;
    static bool first_call = true;
    static uint32_t last_enter_time = 0;
    static bool processing_command = false;
    
    // 첫 호출 확인
    if (first_call) {
        all_printf("*** process_talk_input() FIRST CALL ***\r\n");
        first_call = false;
    }
    
    // CDC에서 문자 읽기 시도 (USB 터미널)
    if (cdcAvailable() > 0) {
        char ch = cdcRead();
        
        if (ch == '\r' || ch == '\n') {
            // ENTER 입력 시 처리 (강화된 중복 방지)
            uint32_t current_time = millis();
            
            all_printf("[DEBUG] ENTER detected: ch=0x%02X, time=%lu, last_time=%lu, processing=%d, buffer_idx=%d\r\n", 
                      (uint8_t)ch, current_time, last_enter_time, processing_command, buffer_index);
            
            // 이미 처리 중이면 완전히 무시
            if (processing_command) {
                all_printf("[DEBUG] Command already processing, ignoring\r\n");
                return;
            }
            
            // 500ms 이내의 중복 ENTER는 무시 (시간 증가)
            if (current_time - last_enter_time < 500) {
                all_printf("[DEBUG] Ignoring duplicate ENTER (time diff: %lu ms)\r\n", 
                          current_time - last_enter_time);
                return;
            }
            
            if (buffer_index > 0) {
                processing_command = true;
                last_enter_time = current_time;
                
                input_buffer[buffer_index] = '\0';
                all_printf("[DEBUG] Processing command: '%s'\r\n", input_buffer);
                
                // 매크로 명령 체크 먼저
                if (process_macro_command(input_buffer)) {
                    // 매크로 명령이었으면 프롬프트만 출력
                    all_printf("> ");
                } else {
                    // 매크로 명령이 아니면 기존 CAN 데이터 파싱 및 전송
                    uint32_t id;
                    uint8_t data[64];
                    uint8_t length;
                    
                    if (parse_can_data(input_buffer, &id, data, &length)) {
                        can_tx_message(id, data, length);
                    }
                    
                    // 프롬프트 출력
                    all_printf("> ");
                }
                
                // 처리 완료 후 플래그 클리어를 지연시켜 중복 방지 강화
                for (volatile int i = 0; i < 100000; i++); // 짧은 지연
                processing_command = false;
            }
            buffer_index = 0;
        } else if (ch == '\b' || ch == 127) {
            // 백스페이스 처리
            if (buffer_index > 0) {
                buffer_index--;
                all_printf("\b \b"); // 백스페이스 + 공백 + 백스페이스
            }
        } else if (ch >= 32 && ch <= 126) {
            // 일반 문자 처리
            if (buffer_index < sizeof(input_buffer) - 1) {
                input_buffer[buffer_index++] = ch;
                all_printf("%c", ch); // 에코
            }
        }
        return; // CDC로 입력이 들어왔으면 UART는 체크하지 않음
    }
    
    // UART에서 문자 읽기 (시리얼 터미널)
    if (uartAvailable(HW_UART_CH_DEBUG) > 0) {
        char ch = uartRead(HW_UART_CH_DEBUG);
        
        // ch 값 디버깅용 출력
        all_printf("UART Received char: '%c' (0x%02X)\r\n", ch, (uint8_t)ch);
        
        if (ch == '\r' || ch == '\n') {
            // ENTER 입력 시 처리
            if (buffer_index > 0) {
                input_buffer[buffer_index] = '\0';
                
                // 매크로 명령 체크 먼저
                if (process_macro_command(input_buffer)) {
                    // 매크로 명령이었으면 프롬프트만 출력
                    all_printf("> ");
                } else {
                    // 매크로 명령이 아니면 기존 CAN 데이터 파싱 및 전송
                    uint32_t id;
                    uint8_t data[64];
                    uint8_t length;
                    
                    if (parse_can_data(input_buffer, &id, data, &length)) {
                        can_tx_message(id, data, length);
                    }
                    
                    // 프롬프트 출력
                    all_printf("> ");
                }
            }
            buffer_index = 0;
        } else if (ch == '\b' || ch == 127) {
            // 백스페이스 처리
            if (buffer_index > 0) {
                buffer_index--;
                all_printf("\b \b"); // 백스페이스 + 공백 + 백스페이스
            }
        } else if (ch >= 32 && ch <= 126) {
            // 일반 문자 처리
            if (buffer_index < sizeof(input_buffer) - 1) {
                input_buffer[buffer_index++] = ch;
                all_printf("%c", ch); // 에코
            }
        }
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
    all_printf("Available Modes: NORMAL -> UDS_PATH -> TALK\r\n");
    
    // UART 상태 확인
    all_printf("UART DEBUG Status: %s\r\n", uartIsOpen(HW_UART_CH_DEBUG) ? "OPEN" : "CLOSED");
    all_printf("UART EXT (Steering) Status: %s\r\n", uartIsOpen(HW_UART_CH_EXT) ? "OPEN" : "CLOSED");
    
    // 스티어링 컬럼 초기화 메시지 전송
    if (uartIsOpen(HW_UART_CH_EXT)) {
        uartPrintf(HW_UART_CH_EXT, "STEERING_INIT:OK\r\n");
    }
    
    // UDS CAN 매니저 초기화 (RX/TX 모드)
    if (can_manager_init(true))  // true = TX 모드 활성화
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
      
      // TALK 모드에서 터미널 입력 처리
      static uint32_t mode_debug_counter = 0;
      mode_debug_counter++;
      if (mode_debug_counter % 1000000 == 0) {
        printf("Current mode: %d, UDS_MODE_TALK: %d\n", current_mode, UDS_MODE_TALK);
      }
      
      if (current_mode == UDS_MODE_TALK) {
        process_talk_input();
      }
      
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

