#include "logger.h"
#include "string.h"
#include "bsp_uart.h"
#include <stdlib.h>

void logger_printf_convert_to_usart_debugger(LoggerLevel level, uint32_t time_cnt, const char* sFormat, ...)
{
    va_list ParamList;
    char c_now, c_s;
    char* var_s;
    int32_t var_int;
    uint8_t var_c;
    float var_f;
    uint16_t pbuf_len = 0;
    

    char string[20] = {0};
    int ipart;
    int  fpart;
    uint8_t i = 0;

    uint8_t* spbuf; 
    spbuf = bsp_uart_dbg_get_tx_pbuf();

    switch (level) 
    {
        case kFatal:
            //memcpy(spbuf, "[Fatal]", 7);
            spbuf[0] = '[';
            spbuf[1] = 'F';
            spbuf[2] = 'a';
            spbuf[3] = 't';
            spbuf[4] = 'a';
            spbuf[5] = 'l';
            spbuf[6] = ']';
            pbuf_len = 7;
            break;
        case kError:
            //memcpy(spbuf, "[Error]", 7);
            spbuf[0] = '[';
            spbuf[1] = 'E';
            spbuf[2] = 'r';
            spbuf[3] = 'r';
            spbuf[4] = 'o';
            spbuf[5] = 'r';
            spbuf[6] = ']';
            pbuf_len = 7;
            break;
        case kWarn:
            //memcpy(spbuf, "[Warn]", 6);
            spbuf[0] = '[';
            spbuf[1] = 'W';
            spbuf[2] = 'a';
            spbuf[3] = 'r';
            spbuf[4] = 'n';
            spbuf[5] = ']';
            pbuf_len = 6;
            break;
        case kInfo:
            //memcpy(spbuf, "[Info]", 6);
            spbuf[0] = '[';
            spbuf[1] = 'I';
            spbuf[2] = 'n';
            spbuf[3] = 'f';
            spbuf[4] = 'o';
            spbuf[5] = ']';
            pbuf_len = 6;
            break;
        case kDebug:
            //memcpy(spbuf, "[Debug]", 7);
            spbuf[0] = '[';
            spbuf[1] = 'D';
            spbuf[2] = 'e';
            spbuf[3] = 'b';
            spbuf[4] = 'u';
            spbuf[5] = 'g';
            spbuf[6] = ']';
            pbuf_len = 7;
            break;
        default:
            break;
    }

    c_now = *sFormat;
    va_start(ParamList, sFormat);
    while (c_now != 0u) 
    {

        if (c_now == '%') {
            sFormat++;
            c_now = *sFormat;
            switch (c_now) {
                case 'd':
                    var_int = va_arg(ParamList, int);
                    itoa(var_int, string, 10);
                    i = 0;
                    while(string[i] != 0)
                    {
                        spbuf[pbuf_len++] = string[i++];
                    }
                    break;
                case 'f':
                    var_f = va_arg(ParamList, double);
                    if (var_f < 0)
                    {
                        spbuf[pbuf_len++] = '-';
                        var_f = -var_f;
                    }
                    ipart = var_f;
                    fpart = (var_f - ipart)*100000;

                    itoa(ipart, string, 10);
                    i = 0;
                    while(string[i] != 0)
                    {
                        spbuf[pbuf_len++] = string[i++];
                    }
                    spbuf[pbuf_len++] = '.';
                    itoa(fpart, string, 10);
                    i = 0;
                    while(string[i] != 0)
                    {
                        spbuf[pbuf_len++] = string[i++];
                    }
                    break;
                case 'c':
                    var_c = va_arg(ParamList, int);
                    spbuf[pbuf_len++] = (uint8_t)var_c;
                    break;
                case 's':
                    var_s = va_arg(ParamList, char*);
                    c_s = *var_s;
                    while (c_s != 0u) {
                        spbuf[pbuf_len++] = (uint8_t)c_s;
                        var_s++;
                        c_s = *var_s;
                    }
                    break;
                default:
                    break;
            }
        } else {
            spbuf[pbuf_len++] = (uint8_t)c_now;
        }
        sFormat++;
        c_now = *sFormat;
    }
    
    bsp_uart_dbg_set_tx_size(pbuf_len);

    return;
}