#ifndef COMM_HANDLER_H_
#define COMM_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"

// 통신 출력 함수들
void uart_printf(const char *fmt, ...);
void cdc_printf(const char *fmt, ...);
void all_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* COMM_HANDLER_H_ */ 