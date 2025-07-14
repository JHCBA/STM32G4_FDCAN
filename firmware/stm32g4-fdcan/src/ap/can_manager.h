#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "comm_handler.h"

// CAN 설정
extern bool can_test_tx_mode;

// CAN 초기화 및 관리 함수들
bool can_manager_init(bool tx_mode);
void can_manager_process(void);

// CAN TX 관련 함수들
bool can_tx_init(void);
void can_tx_process(void);

// CAN RX 관련 함수들  
bool can_rx_init(void);
void can_rx_process(void);

// CAN 에러 처리 함수들
void can_error_check(void);
void can_error_recovery(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MANAGER_H_ */ 