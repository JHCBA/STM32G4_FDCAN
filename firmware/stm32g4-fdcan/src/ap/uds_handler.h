#ifndef UDS_HANDLER_H_
#define UDS_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "can.h"

// UDS 관련 정의
#define UDS_REQUEST_ID      0x7D4
#define UDS_RESPONSE_ID     0x7DC
#define UDS_TIMEOUT_MS      100

// UDS Frame 타입
typedef enum {
    UDS_SINGLE_FRAME = 0,
    UDS_FIRST_FRAME = 1,
    UDS_CONSECUTIVE_FRAME = 2,
    UDS_FLOW_CONTROL = 3
} uds_frame_type_t;

// UDS 상태
typedef enum {
    UDS_STATE_IDLE,
    UDS_STATE_WAITING_FLOW_CONTROL,
    UDS_STATE_SENDING_CONSECUTIVE,
    UDS_STATE_ERROR
} uds_state_t;

// UDS 핸들러 구조체
typedef struct {
    uds_state_t state;
    uint32_t timeout_start;
    uint8_t sequence_number;
    bool active;
} uds_handler_t;

// UDS 관련 함수들
bool uds_init(void);
void uds_process(void);
bool uds_handle_request(const can_msg_t *msg);
bool uds_send_first_frame(void);
bool uds_send_consecutive_frames(void);
bool uds_check_flow_control(const can_msg_t *msg);
void uds_reset_state(void);

// 디버그 함수
void uds_print_message(const char* prefix, const can_msg_t *msg);

// UDS 상태 확인 함수
bool uds_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* UDS_HANDLER_H_ */
