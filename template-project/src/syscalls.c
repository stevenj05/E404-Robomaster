#ifndef PLATFORM_HOSTED
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <stdint.h>

#define STM32F427xx
#include "stm32f4xx.h"

// -------------------------
// UART base addresses (adjust per board)
// -------------------------
#define DEBUG_UART   USART3   // Debug UART for printf
#define REF_UART     USART6   // Referee system UART

// -------------------------
// _write - printf / logging
// -------------------------
int _write(int file, const char *ptr, int len) {
    (void)file;

    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            while (!(DEBUG_UART->SR & USART_SR_TXE)) {}
            DEBUG_UART->DR = '\r';
        }
        while (!(DEBUG_UART->SR & USART_SR_TXE)) {}
        DEBUG_UART->DR = ptr[i];
    }

    return len;
}

// -------------------------
// _read - referee system
// -------------------------
#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

// Call this in your USART6 IRQ handler
void REF_UART_IRQHandler(void) {
    if (REF_UART->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(REF_UART->DR & 0xFF);
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) { // prevent overflow
            rx_buf[rx_head] = byte;
            rx_head = next;
        }
    }
}

int _read(int file, char *ptr, int len) {
    int count = 0;
    (void)file;

    while (count < len && rx_head != rx_tail) {
        ptr[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    }

    if (count == 0) {
        errno = EAGAIN;
        return -1;
    }

    return count;
}

// -------------------------
// Other minimal syscalls stubs
// -------------------------
int _close(int file) {
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st) {
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    (void)file; (void)ptr; (void)dir;
    return 0;
}

int _kill(int pid, int sig) {
    (void)pid; (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}
#endif