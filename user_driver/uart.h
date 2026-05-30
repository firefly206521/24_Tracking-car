#ifndef UART_H
#define UART_H
#define RX_BUF_SIZE 101
#include <stdio.h>
#include "ti_msp_dl_config.h"
extern uint8_t rx_buf[RX_BUF_SIZE];  // 全局接收缓冲区
extern uint16_t rx_index;         // 接收计数
extern uint8_t rx_complete;       // 接收完成标记：1=收到回车，数据完整

void UART_send_encoder(UART_Regs *uart, uint32_t left_encoder_value,uint32_t right_encoder_value);

void UART_send_string(UART_Regs *uart, const char *str);

void UART_send_char(UART_Regs *uart, uint8_t chr);


#endif //UART_H