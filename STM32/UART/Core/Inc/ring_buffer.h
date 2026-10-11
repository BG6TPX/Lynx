/**
  ******************************************************************************
  * @file    ring_buffer.h
  * @brief   Generic single-producer / single-consumer byte ring buffer.
  *
  * Lock-free by construction: the producer only ever writes @c head and the
  * consumer only ever writes @c tail, so no critical section is needed as long
  * as those two roles stay on different execution contexts -- the intended use
  * is an ISR producing while the main loop consumes. Having two producers, two
  * consumers, or producing from the main loop while an ISR also produces is NOT
  * supported and will corrupt the buffer.
  *
  * Capacity is @c size - 1 bytes: one slot is always left unused so that the
  * "empty" and "full" conditions stay distinguishable without a separate
  * counter, which also keeps the head/tail update a single store.
  *
  * @c size must be a power of two so the wrap-around modulo becomes a mask.
  *
  * There is no memory barrier: on a single-core Cortex-M3 the exception return
  * is itself a context synchronisation event, so stores made by the ISR are
  * visible to the main loop on return. A @c __DMB() would only become necessary
  * if another core or a DMA controller were to observe the same buffer.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Ring buffer control block.
  *
  * The storage is owned by the caller and passed to rb_init(); this struct
  * never allocates and holds no copy of the data.
  */
typedef struct
{
  uint8_t          *buf;   /**< Caller-owned storage, @c size bytes long        */
  uint16_t          size;  /**< Storage size in bytes, power of two, <= 32768   */
  volatile uint16_t head;  /**< Write index; written by the producer only       */
  volatile uint16_t tail;  /**< Read index;  written by the consumer only       */
} ring_buffer_t;

/* Exported constants --------------------------------------------------------*/

/** Largest accepted @c size : 2^15, so that @c size - 1 still fits a uint16_t. */
#define RB_MAX_SIZE   ((uint16_t)32768U)

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Bind caller-owned storage to a ring buffer.
  * @param  rb     Buffer to initialise.
  * @param  buf    Storage to use; must stay valid for the buffer's lifetime.
  * @param  size   Storage size in bytes; power of two, 2 .. RB_MAX_SIZE.
  * @retval 0 on success, -1 if the arguments are unusable (rb is then untouched).
  */
int      rb_init(ring_buffer_t *rb, uint8_t *buf, uint16_t size);

/**
  * @brief  Discard all buffered data. Only the consumer may call this.
  */
void     rb_reset(ring_buffer_t *rb);

/** @brief Bytes currently readable. */
uint16_t rb_used(const ring_buffer_t *rb);

/** @brief Bytes that rb_putc() would still accept. */
uint16_t rb_free(const ring_buffer_t *rb);

/** @brief Non-zero when there is nothing to read. */
int      rb_is_empty(const ring_buffer_t *rb);

/** @brief Non-zero when nothing more can be written. */
int      rb_is_full(const ring_buffer_t *rb);

/**
  * @brief  Append one byte (producer side).
  * @retval 0 on success, -1 if the buffer is full and the byte was dropped.
  */
int      rb_putc(ring_buffer_t *rb, uint8_t byte);

/**
  * @brief  Remove one byte (consumer side).
  * @retval The byte, or -1 if the buffer is empty.
  */
int      rb_getc(ring_buffer_t *rb);

/**
  * @brief  Append up to @p len bytes (producer side).
  * @retval Number of bytes actually written; less than @p len when full.
  */
uint16_t rb_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len);

/**
  * @brief  Remove up to @p len bytes (consumer side).
  * @retval Number of bytes actually read; less than @p len when drained.
  */
uint16_t rb_read(ring_buffer_t *rb, uint8_t *dst, uint16_t len);

/**
  * @brief  Inspect a byte without consuming it (consumer side).
  * @param  offset 0 is the oldest buffered byte.
  * @retval The byte, or -1 if @p offset is past the end of the data.
  */
int      rb_peek_at(const ring_buffer_t *rb, uint16_t offset);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H */
