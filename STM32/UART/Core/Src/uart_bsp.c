/**
  ******************************************************************************
  * @file    uart_bsp.c
  * @brief   Interrupt-driven USART1 driver built on ring_buffer.
  *
  * WHY HAL_UART_IRQHandler() IS NOT CALLED
  * ---------------------------------------
  * HAL only ever moves bytes while one of its own transfers is in progress, and
  * this driver never starts one, so huart1.RxState/gState sit permanently at
  * READY. HAL_UART_IRQHandler() does not know that. It reacts to our interrupt
  * by calling UART_Receive_IT()/UART_Transmit_IT(), each of which takes one
  * look at its state field, finds it is not BUSY, and returns HAL_BUSY without
  * reading or writing DR (stm32f1xx_hal_uart.c:3683 and :3557).
  *
  * The result is not a polite "reception stops working". The source flag --
  * RXNE or TXE -- is never cleared and RXNEIE/TXEIE stay enabled, so the NVIC
  * line stays asserted, the handler re-enters the instant it returns, and the
  * main loop never runs again. The board looks hung with the CPU pinned inside
  * the interrupt.
  *
  * So the USART1 interrupt is serviced entirely at register level and HAL is
  * left to do nothing beyond the one-off configuration in
  * MX_USART1_UART_Init(). Every flag this handler enables an interrupt for must
  * be cleared in here, or the same livelock returns.
  *
  * Reading and writing the USART data register from here also means HAL's
  * blocking HAL_UART_Transmit()/Receive() must not be used on USART1: they poll
  * the same flags and would race this handler for the same bytes.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_bsp.h"
#include "ring_buffer.h"
#include "main.h"

#include <stddef.h>

/* Private variables ---------------------------------------------------------*/

/** Transmit/receive flag helpers used below read CR1/SR directly, so the only
    thing taken from HAL is the instance pointer and the configuration. */
extern UART_HandleTypeDef huart1;   /* defined in main.c */

static uint8_t s_rx_storage[UART_RX_BUFFER_SIZE];
static uint8_t s_tx_storage[UART_TX_BUFFER_SIZE];

static ring_buffer_t s_rx;
static ring_buffer_t s_tx;

/* Touched from both the handler and the main loop; 32-bit accesses are atomic
   on Cortex-M3, so a plain volatile count is enough. */
static volatile uint32_t s_rx_dropped;
static volatile uint32_t s_tx_dropped;
static volatile uint32_t s_rx_errors;

/* Private helpers -----------------------------------------------------------*/

/** Every flag that signals "a byte finished arriving", errors included. */
#define UART_RX_EVENT_FLAGS  (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)

/** Subset of the above that means the byte in DR cannot be trusted. */
#define UART_RX_CORRUPT_FLAGS (USART_SR_FE | USART_SR_NE | USART_SR_PE)

/* Exported functions --------------------------------------------------------*/

void uart_bsp_init(void)
{
  (void)rb_init(&s_rx, s_rx_storage, (uint16_t)UART_RX_BUFFER_SIZE);
  (void)rb_init(&s_tx, s_tx_storage, (uint16_t)UART_TX_BUFFER_SIZE);

  s_rx_dropped = 0U;
  s_tx_dropped = 0U;
  s_rx_errors  = 0U;

  /* Discard whatever the peripheral latched before we got here. */
  __HAL_UART_CLEAR_OREFLAG(&huart1);

  /* Nothing forces the first of these to be the last thing we do, but it is
     the point at which the driver goes live. The NVIC line itself was already
     enabled by HAL_UART_MspInit() during HAL_UART_Init(). */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void uart_bsp_irq_handler(void)
{
  /* Latch SR once. Note this read is also the first half of the SR-then-DR
     sequence that clears the receive flags, so DR must be read below no matter
     which of them is set. */
  uint32_t sr = USART1->SR;
  int byte;

  /* --- Receive ----------------------------------------------------------- */

  /* All five of these belong to one "a byte finished arriving" event and the
     SR-then-DR read pair clears them together, so they are tested as a group
     and DR is read exactly once. Testing RXNE on its own would leave ORE (or
     FE/NE/PE) standing whenever RXNE happened to be clear, and any uncleared
     flag we have an interrupt enabled for re-enters this handler forever. */
  if ((sr & UART_RX_EVENT_FLAGS) != 0U)
  {
    uint8_t data = (uint8_t)(USART1->DR & 0xFFU);   /* clears the whole group */

    if ((sr & UART_RX_CORRUPT_FLAGS) != 0U)
    {
      /* Framing, noise or parity problem: the byte itself is not trustworthy,
         so it is counted and dropped rather than fed to the parser. */
      s_rx_errors++;
    }
    else
    {
      if (rb_putc(&s_rx, data) != 0)
      {
        s_rx_dropped++;   /* queue full: drop the newest, keep the older ones */
      }

      /* ORE means a byte was still sitting unread in DR when this one landed,
         so an earlier byte was lost while this one survived. The hardware
         latches this once per overrun rather than counting lost bytes, so it
         is an event count, not a byte count. */
      if ((sr & USART_SR_ORE) != 0U)
      {
        s_rx_errors++;
      }
    }
  }

  /* --- Transmit ---------------------------------------------------------- */

  /* Read SR fresh rather than reusing the latch: TXE is unrelated to the
     receive flags above, and by now it may have been set by the last byte
     moving out of DR -- catching that here saves a whole extra interrupt. */
  if (((USART1->SR & USART_SR_TXE) != 0U) && ((USART1->CR1 & USART_CR1_TXEIE) != 0U))
  {
    byte = rb_getc(&s_tx);

    if (byte >= 0)
    {
      USART1->DR = (uint8_t)byte;
    }
    else
    {
      /* Queue drained. Switching TXEIE off here is what stops an idle
         transmitter from re-entering this handler forever; uart_write() turns
         it back on whenever it queues something new. */
      USART1->CR1 &= ~USART_CR1_TXEIE;
    }
  }
}

/* --- Receive ---------------------------------------------------------------*/

uint16_t uart_available(void)
{
  return rb_used(&s_rx);
}

int uart_getc(void)
{
  return rb_getc(&s_rx);
}

uint16_t uart_read(uint8_t *dst, uint16_t len)
{
  return rb_read(&s_rx, dst, len);
}

void uart_flush_rx(void)
{
  /* rb_reset() only advances the read index, which belongs to this side, so it
     cannot race the handler: any byte it appends while this runs is simply
     left in the queue. */
  rb_reset(&s_rx);
}

uart_line_status_t uart_read_line(char *line, uint16_t max_len, uint32_t timeout_ms)
{
  uint32_t deadline;
  uint16_t used;
  uint16_t terminator = 0U;
  uint16_t copy;
  uint16_t consume;
  uint16_t i;
  int      found = 0;
  uart_line_status_t status;

  if ((line == NULL) || (max_len < 2U))
  {
    return UART_LINE_TIMEOUT;   /* caller error: nowhere to put a line */
  }

  deadline = HAL_GetTick() + timeout_ms;

  for (;;)
  {
    used = rb_used(&s_rx);

    /* Hunt for the terminator without consuming anything. Scanning up to a
       snapshot of the queue length is safe: the producer only ever appends
       past the end of the data and drops bytes rather than overwriting when
       full, so nothing in [0, used) can change under us. */
    for (i = 0U; i < used; i++)
    {
      if ((uint8_t)rb_peek_at(&s_rx, i) == (uint8_t)'\n')
      {
        terminator = i;
        found = 1;
        break;
      }
    }

    if (found != 0)
    {
      break;
    }

    if (timeout_ms == 0U)
    {
      return UART_LINE_TIMEOUT;   /* poll-once mode: never block */
    }

    /* Signed difference, so this stays correct when HAL_GetTick() wraps. */
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      /* Nothing consumed: the partial line stays queued and the next call
         picks it up, so a line arriving across two calls is not lost. */
      return UART_LINE_TIMEOUT;
    }
  }

  /* Number of payload bytes sitting before the terminator. */
  if (terminator <= (uint16_t)(max_len - 1U))
  {
    copy   = terminator;
    status = UART_LINE_OK;
  }
  else
  {
    copy   = (uint16_t)(max_len - 1U);
    status = UART_LINE_OVERFLOW;
  }

  consume = (uint16_t)(terminator + 1U);   /* payload plus the LF itself */

  (void)rb_read(&s_rx, (uint8_t *)line, copy);

  /* Drop whatever did not fit, terminator included, so that the next call
     resynchronises on the following line instead of returning the tail of
     this one as a bogus short line. */
  for (i = copy; i < consume; i++)
  {
    (void)rb_getc(&s_rx);
  }

  line[copy] = '\0';

  /* A CR before the LF belongs to the wire protocol, not to the payload. Only
     meaningful when the line was not truncated. */
  if ((status == UART_LINE_OK) && (copy > 0U) && (line[copy - 1U] == '\r'))
  {
    line[copy - 1U] = '\0';
  }

  return status;
}

/* --- Transmit --------------------------------------------------------------*/

void uart_write(const uint8_t *data, uint16_t len)
{
  uint16_t written;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /* ORDER MATTERS: queue the bytes first, enable TXEIE second.
     Enabling TXEIE is what arms the handler, and the handler disables it again
     the instant it finds the queue empty. Reversed, the handler can run against
     an empty queue, switch the interrupt back off, and leave the bytes queued a
     moment later stranded forever with nothing left to re-arm it. Done in this
     order the worst case is one spurious interrupt that immediately cancels
     itself, which is harmless. */
  written = rb_write(&s_tx, data, len);
  s_tx_dropped += (uint32_t)(len - written);

  if (written > 0U)
  {
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  }
}

void uart_putc(uint8_t byte)
{
  uart_write(&byte, 1U);
}

void uart_puts(const char *s)
{
  const char *end = s;

  if (s == NULL)
  {
    return;
  }

  while (*end != '\0')
  {
    end++;
  }

  uart_write((const uint8_t *)s, (uint16_t)(end - s));
}

void uart_tx_flush(void)
{
  uint32_t deadline = HAL_GetTick() + UART_TX_FLUSH_TIMEOUT_MS;

  /* Let the queue drain into the peripheral first. */
  while (rb_is_empty(&s_tx) == 0)
  {
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      return;
    }
  }

  /* Then wait for the last byte to clear the shift register. TC can read as
     set for a moment between two queued bytes, so it only means "the line is
     idle" once the queue is empty. */
  while ((USART1->SR & USART_SR_TC) == 0U)
  {
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      return;
    }
  }
}

/* --- Diagnostics -----------------------------------------------------------*/

uint32_t uart_rx_dropped(void)
{
  return s_rx_dropped;
}

uint32_t uart_tx_dropped(void)
{
  return s_tx_dropped;
}

uint32_t uart_rx_errors(void)
{
  return s_rx_errors;
}
