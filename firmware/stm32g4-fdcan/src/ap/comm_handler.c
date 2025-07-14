#include "comm_handler.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 간단한 UART 출력 함수
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uartPrintf(HW_UART_CH_DEBUG, buf);
}

// USB CDC 출력 함수
void cdc_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (cdcIsConnect()) {
        cdcWrite((uint8_t*)buf, strlen(buf));
    }
}

// 모든 채널로 출력하는 함수
void all_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 물리적 UART로 출력
    uartPrintf(HW_UART_CH_DEBUG, buf);
    
    // USB CDC로도 출력
    if (cdcIsConnect()) {
        cdcWrite((uint8_t*)buf, strlen(buf));
    }
} 