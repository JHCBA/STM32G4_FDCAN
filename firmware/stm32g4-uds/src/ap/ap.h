#ifndef AP_H_
#define AP_H_

#include "ap_def.h"

// 모드 타입 정의
typedef enum {
    UDS_MODE_UDS_PATH = 0,  // UDS 데이터 추출 모드 (스티어링 데이터 처리)
    UDS_MODE_TALK = 1,      // 터미널 직접 TX 모드
    UDS_MODE_MAX
} uds_mode_t;

void apInit(void);
void apMain(void);
const char* uds_get_service_name(uint8_t service_id);

// 모드 관리 함수들
uds_mode_t get_current_mode(void);
void change_mode(void);

// TALK 모드 함수들
bool can_tx_message(uint32_t id, uint8_t *data, uint8_t length);
void process_talk_input(void);
bool parse_can_data(const char* input, uint32_t* id, uint8_t* data, uint8_t* length);
bool process_macro_command(const char* input);
void set_default_can_id(uint32_t id);
uint32_t get_default_can_id(void);

#endif