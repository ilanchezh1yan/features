/**
 * @file uart.h
 * @brief UART interface definitions.
 *
 * Contains UART-related macros, data structures, and function
 * declarations used for serial communication.
 */
#ifndef UART_H
#define UART_H

/**
 * @brief Transmit data through the UART interface.
 *
 * Sends a data buffer over the serial communication channel
 * for monitoring, debugging, or external data acquisition.
 *
 * @param data Pointer to the transmit buffer.
 * @param len Number of bytes to transmit.
 *
 * @return None.
 */
void send_data(uint8_t *data, uint8_t len);

#endif
