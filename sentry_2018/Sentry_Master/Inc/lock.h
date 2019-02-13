#ifndef LOCK_H_
#define LOCK_H_

#include <stdint.h>
#define __UART_LOCK() UART_LOCK = 1
#define __UART_UNLOCK() UART_LOCK = 0
#define __UART_ISLOCK() UART_LOCK

extern volatile uint8_t UART_LOCK; 

#endif
 
