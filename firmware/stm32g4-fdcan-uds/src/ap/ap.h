#ifndef AP_H_
#define AP_H_

#include "ap_def.h"

// 모드 타입 정의
typedef enum {
    UDS_MODE_NORMAL = 0,    // 기존 상세 출력 모드
    UDS_MODE_STEERING = 1,  // 스티어링 데이터 추출 모드
    UDS_MODE_MAX
} uds_mode_t;

void apInit(void);
void apMain(void);
const char* uds_get_service_name(uint8_t service_id);

// 모드 관리 함수들
uds_mode_t get_current_mode(void);
void change_mode(void);

#endif