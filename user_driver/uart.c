#include "uart.h"
#include <stdio.h>
uint8_t rx_buf[RX_BUF_SIZE];  // 全局接收缓冲区
uint16_t rx_index=0;         // 接收计数
uint8_t rx_complete=0;       // 接收完成标记：1=收到回车，数据完整

//发送字符串函数
void UART_send_char(UART_Regs *uart, uint8_t chr)//函数仿照头文件里面的收发函数来写。
{
    DL_UART_transmitDataBlocking(uart,chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, *str++);
    }
}//字符串发送函数

void UART_send_encoder(UART_Regs *uart, uint32_t left_encoder_value,uint32_t right_encoder_value)
{
    char buffer[12]; // 32-bit integer can be up to 10 digits + sign + null terminator

    UART_send_string(uart, "Left Encoder: ");
    snprintf(buffer, sizeof(buffer), "%d", left_encoder_value);
    UART_send_string(uart, buffer);
    UART_send_string(uart, "\r\n");

    UART_send_string(uart, " Right Encoder:");
    snprintf(buffer, sizeof(buffer), "%d", right_encoder_value);
    UART_send_string(uart, buffer);
    UART_send_string(uart, "\r\n");
    
}//编码器数值发送函数，先把数值转换成字符串，再调用字符串发送函数。
    
// void PRINT_INST_IRQHandler()
// {
//     //用swicth判断是哪种中断
//     switch(DL_UART_getPendingInterrupt(PRINT_INST)) {
//         case DL_UART_IIDX_RX: // 发送中断
//         {
//              // 【测试代码】收到1字节，立刻翻转LED
//             DL_GPIO_togglePins(LED_PORT, LED_LED_01_PIN);
//             uint8_t rec = DL_UART_receiveData(PRINT_INST); // 读取接收到的数据
//                 // 防止缓冲区溢出
//             if(rx_index < RX_BUF_SIZE - 1)
//             {
//                 rx_buf[rx_index++] = rec;
//             }

//             // 收到回车符，表示一帧结束
//             if(rec == '\r'||rec == '\n')
//             {
//                 rx_buf[rx_index] = '\0'; // 加上字符串结束符
//                 rx_complete = 1;         // 标记数据收完了
//             }
//             break;
//         }

//         case DL_UART_IIDX_TX: // 接收中断
//             // 处理接收到的数据，例如读取数据寄存器
//             break;
//         default:
//             break;
//     }
// }
// //记得写中断服务函数

