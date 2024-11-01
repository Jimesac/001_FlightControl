#ifndef __APP_LOGGER_H
#define __APP_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

typedef enum {
    kNoLog,
    kFatal,
    kError,
    kWarn,
    kInfo,
    kDebug
} LoggerLevel;



void logger_printf_convert_to_usart_debugger(LoggerLevel level, uint32_t time_cnt, const char* sFormat, ...);

#define logger_printf(...)  logger_printf_convert_to_usart_debugger

#endif   // __APP_LOGGER_H
