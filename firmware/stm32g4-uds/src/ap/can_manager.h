#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "comm_handler.h"

// UDS CAN 매니저 함수들
bool can_manager_init(bool tx_mode);
void can_manager_process(void);

// UDS 메시지 처리 함수들
void uds_rx_process(void);
void uds_print_message_info(uint32_t can_id, uint8_t *data, uint8_t length);
const char* uds_get_service_name(uint8_t service_id);
void uds_auto_flow_control(can_msg_t *rx_msg);

// CAN 에러 처리 함수들
void can_error_check(void);
void can_error_recovery(void);

// ISO-TP 상태 관리 함수들
bool can_manager_is_isotp_active(void);
bool can_manager_get_isotp_data(uint32_t *can_id, uint8_t **data, uint16_t *length);
void can_manager_reset_isotp_session(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MANAGER_H_ */ 