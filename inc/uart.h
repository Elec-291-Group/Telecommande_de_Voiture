#ifndef UART_H
#define UART_H

// UART0: P0.4 = TX, P0.5 = RX
void UART_init(void);
void UART_send_char(char c);
void UART_send_string(const char *s);
char UART_read(void);
bit  UART_available(void);

// UART1: P3.0 = TX, P3.1 = RX
void UART1_init(void);
void UART1_send_char(char c);
void UART1_send_string(const char *s);
char UART1_read(void);
bit  UART1_available(void);

#endif