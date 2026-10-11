/**
  ******************************************************************************
  * @file    uart_bsp.h
  * @brief   Interrupt-driven USART1 driver built on ring_buffer.
  *
  * Both directions are buffered, so neither reading nor writing ever blocks on
  * the wire: the main loop drops bytes into a TX queue and picks bytes out of
  * an RX queue, while the interrupt handler moves data between the queues and
  * the USART data register one byte at a time.
  *
  * The USART interrupt is handled at register level. HAL_UART_IRQHandler() is
  * deliberately NOT called -- see stm32f1xx_it.c and uart_bsp.c for why.
  * HAL is used only for the one-off configuration in MX_USART1_UART_Init().
  *
  * Calling convention: every function below may be called from the main loop.
  * Only uart_bsp_irq_handler() runs in interrupt context.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_BSP_H
#define __UART_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/**
  * Storage sizes, in bytes, of the two queues. Must each be a power of two.
  * The usable capacity is one byte less (see ring_buffer.h). Raise them if the
  * main loop can stall long enough for RX to overflow at 115200 baud.
  */
#define UART_RX_BUFFER_SIZE   256U
#define UART_TX_BUFFER_SIZE   256U

/** Give uart_tx_flush() a way out if the transmitter never reports idle. */
#define UART_TX_FLUSH_TIMEOUT_MS   1000U

/* Exported types ------------------------------------------------------------*/

/** Outcome of uart_read_line(). */
typedef enum
{
  UART_LINE_OK = 0,   /**< A complete line was returned, terminator stripped.  */
  UART_LINE_TIMEOUT,  /**< No complete line yet; nothing was consumed.          */
  UART_LINE_OVERFLOW  /**< Line longer than max_len-1; truncated, rest dropped. */
} uart_line_status_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Set up the queues and start receiving.
  * @note   Call once, after MX_USART1_UART_Init(). The USART1 interrupt itself
  *         is enabled by HAL_UART_MspInit(), which HAL_UART_Init() has already
  *         run by that point.
  */
void     uart_bsp_init(void);

/**
  * @brief  Service the USART1 interrupt.
  * @note   Interrupt context. Called from USART1_IRQHandler() in
  *         stm32f1xx_it.c; not meant to be called from the main loop.
  */
void     uart_bsp_irq_handler(void);

/* --- Receive ---------------------------------------------------------------*/

/** @brief Bytes waiting in the RX queue. */
uint16_t uart_available(void);

/** @brief Non-blocking read of one byte. @retval The byte, or -1 if none. */
int      uart_getc(void);

/** @brief Non-blocking read of up to @p len bytes. @return Bytes read. */
uint16_t uart_read(uint8_t *dst, uint16_t len);

/**
  * @brief  Throw away everything currently queued for reception.
  *
  * The escape hatch for uart_read_line()'s do-not-consume promise: a partial
  * line that never gets a terminator would otherwise sit at the front of the
  * queue for good, prefixing every line that arrives after it. Also the way to
  * resynchronise after a burst of framing errors.
  *
  * Bytes that arrive while this runs are untouched -- only the read index
  * moves, and that index belongs to the caller's side.
  */
void     uart_flush_rx(void);

/**
  * @brief  Wait for one line, terminated by LF or CRLF.
  *
  * Nothing is consumed unless a terminator is found, so a partial line left by
  * a timeout stays queued and is completed by the next call -- a line split
  * across two timeout windows is never lost.
  *
  * @param  line      Destination, always NUL-terminated on success.
  * @param  max_len   Capacity of @p line, including the terminator; >= 2.
  * @param  timeout_ms How long to keep waiting. 0 polls once and never blocks.
  * @retval UART_LINE_OK       @p line holds a complete line.
  * @retval UART_LINE_TIMEOUT  No terminator before the timeout; nothing consumed.
  * @retval UART_LINE_OVERFLOW The line did not fit; @p line is truncated to
  *                            max_len-1 bytes and the rest of the line, up to
  *                            and including its terminator, was discarded so
  *                            the next call resynchronises on the next line.
  */
uart_line_status_t uart_read_line(char *line, uint16_t max_len, uint32_t timeout_ms);

/* --- Transmit --------------------------------------------------------------*/

/**
  * @brief  Queue bytes for transmission and return immediately.
  * @note   Bytes that do not fit are dropped and counted by uart_tx_dropped().
  */
void     uart_write(const uint8_t *data, uint16_t len);

/** @brief Queue one byte. Equivalent to uart_write() with len 1. */
void     uart_putc(uint8_t byte);

/** @brief Queue a NUL-terminated string, excluding the terminator. */
void     uart_puts(const char *s);

/**
  * @brief  Block until the queue has drained and the last stop bit has left
  *         the shift register (the TC flag), or UART_TX_FLUSH_TIMEOUT_MS
  *         elapses.
  * @note   Only needed before something that depends on the line actually being
  *         idle -- disabling the peripheral, entering low-power, or driving a
  *         half-duplex direction pin.
  */
void     uart_tx_flush(void);

/* --- Diagnostics -----------------------------------------------------------*/

/** @brief Bytes dropped because the RX queue was full. */
uint32_t uart_rx_dropped(void);

/** @brief Bytes dropped because the TX queue was full. */
uint32_t uart_tx_dropped(void);

/** @brief Receive error events (overrun, framing, noise or parity). */
uint32_t uart_rx_errors(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_BSP_H */
