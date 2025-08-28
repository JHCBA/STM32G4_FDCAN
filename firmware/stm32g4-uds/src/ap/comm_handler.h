#ifndef COMM_HANDLER_H_
#define COMM_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "diag_db.h"

// 통신 출력 함수들
void uart_printf(const char *fmt, ...);
void cdc_printf(const char *fmt, ...);
void all_printf(const char *fmt, ...);

// UDS 디버그 및 데이터 처리 함수들
void uds_debug_output(uint32_t can_id, uint8_t *data, uint8_t length);
void steering_data_handler(uint32_t can_id, uint8_t *data, uint8_t length);
void parse_steering_data(uint8_t *complete_data, uint8_t data_length);
void parse_data(uint8_t *complete_data, uint8_t data_length, data_type_t data_type);
void parse_multiple_data(uint8_t *complete_data, uint8_t data_length, uint32_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* COMM_HANDLER_H_ */ 