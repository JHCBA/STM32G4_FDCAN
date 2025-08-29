#ifndef CAN_TX_H_
#define CAN_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"

// DEBUG 플래그 - 디버깅 메시지 출력 제어
#ifndef DEBUG_CAN_PROTOCOL
#define DEBUG_CAN_PROTOCOL  1  // 0: 디버깅 비활성화, 1: 디버깅 활성화
#endif

// CAN TX 관련 함수들
bool can_tx_init(void);
void can_tx_process(void);
void can_tx_handle_rx_message(const can_msg_t *msg);

// CAN 에러 복구 함수 (can_manager.c에서 정의된 함수)
void can_error_recovery(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_H_ */