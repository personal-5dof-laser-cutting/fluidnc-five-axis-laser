#pragma once
#define UART_NUM_MAX 3
typedef enum { UART_DATA_5_BITS=0, UART_DATA_6_BITS, UART_DATA_7_BITS, UART_DATA_8_BITS } uart_word_length_t;
typedef enum { UART_STOP_BITS_1=0, UART_STOP_BITS_1_5, UART_STOP_BITS_2 } uart_stop_bits_t;
typedef enum { UART_PARITY_DISABLE=0, UART_PARITY_EVEN, UART_PARITY_ODD } uart_parity_t;
