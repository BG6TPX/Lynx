/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   Generic single-producer / single-consumer byte ring buffer.
  *
  * See ring_buffer.h for the concurrency contract. Every function here is a
  * handful of loads and stores; none of them masks interrupts.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ring_buffer.h"

#include <stddef.h>

/* Private helpers -----------------------------------------------------------*/

/** Wrap-around modulo; valid because size is always a power of two. */
static uint16_t rb_mask(const ring_buffer_t *rb)
{
  return (uint16_t)(rb->size - 1U);
}

/* Exported functions --------------------------------------------------------*/

int rb_init(ring_buffer_t *rb, uint8_t *buf, uint16_t size)
{
  if ((rb == NULL) || (buf == NULL))
  {
    return -1;
  }
  if ((size < 2U) || (size > RB_MAX_SIZE))
  {
    return -1;
  }
  if ((size & (uint16_t)(size - 1U)) != 0U)
  {
    return -1;  /* not a power of two */
  }

  rb->buf  = buf;
  rb->size = size;
  rb->head = 0U;
  rb->tail = 0U;

  return 0;
}

void rb_reset(ring_buffer_t *rb)
{
  if (rb == NULL)
  {
    return;
  }

  rb->tail = rb->head;
}

uint16_t rb_used(const ring_buffer_t *rb)
{
  if (rb == NULL)
  {
    return 0U;
  }

  /* The subtraction is taken modulo size, so it stays correct across the wrap
     even though the unsigned difference could otherwise go negative. */
  return (uint16_t)((rb->head - rb->tail) & rb_mask(rb));
}

uint16_t rb_free(const ring_buffer_t *rb)
{
  if (rb == NULL)
  {
    return 0U;
  }

  /* One slot is deliberately left unused to tell "full" from "empty". */
  return (uint16_t)(rb_mask(rb) - rb_used(rb));
}

int rb_is_empty(const ring_buffer_t *rb)
{
  return (rb_used(rb) == 0U) ? 1 : 0;
}

int rb_is_full(const ring_buffer_t *rb)
{
  return (rb_free(rb) == 0U) ? 1 : 0;
}

int rb_putc(ring_buffer_t *rb, uint8_t byte)
{
  if (rb_is_full(rb))
  {
    return -1;
  }

  rb->buf[rb->head] = byte;
  rb->head = (uint16_t)((rb->head + 1U) & rb_mask(rb));

  return 0;
}

int rb_getc(ring_buffer_t *rb)
{
  uint8_t byte;

  if (rb_is_empty(rb))
  {
    return -1;
  }

  byte = rb->buf[rb->tail];
  rb->tail = (uint16_t)((rb->tail + 1U) & rb_mask(rb));

  return (int)byte;
}

uint16_t rb_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len)
{
  uint16_t head;
  uint16_t first;
  uint16_t i;

  if ((rb == NULL) || (data == NULL))
  {
    return 0U;
  }

  if (len > rb_free(rb))
  {
    len = rb_free(rb);
  }
  if (len == 0U)
  {
    return 0U;
  }

  head = rb->head;

  /* Copy in at most two runs so the wrap-around costs one mask, not one per
     byte. The head index itself is advanced by a single masked store. */
  first = (uint16_t)(rb->size - head);
  if (first > len)
  {
    first = len;
  }

  for (i = 0U; i < first; i++)
  {
    rb->buf[head + i] = data[i];
  }
  for (i = first; i < len; i++)
  {
    rb->buf[(uint16_t)((head + i) & rb_mask(rb))] = data[i];
  }

  rb->head = (uint16_t)((head + len) & rb_mask(rb));

  return len;
}

uint16_t rb_read(ring_buffer_t *rb, uint8_t *dst, uint16_t len)
{
  uint16_t tail;
  uint16_t first;
  uint16_t i;

  if ((rb == NULL) || (dst == NULL))
  {
    return 0U;
  }

  if (len > rb_used(rb))
  {
    len = rb_used(rb);
  }
  if (len == 0U)
  {
    return 0U;
  }

  tail = rb->tail;

  first = (uint16_t)(rb->size - tail);
  if (first > len)
  {
    first = len;
  }

  for (i = 0U; i < first; i++)
  {
    dst[i] = rb->buf[tail + i];
  }
  for (i = first; i < len; i++)
  {
    dst[i] = rb->buf[(uint16_t)((tail + i) & rb_mask(rb))];
  }

  rb->tail = (uint16_t)((tail + len) & rb_mask(rb));

  return len;
}

int rb_peek_at(const ring_buffer_t *rb, uint16_t offset)
{
  if ((rb == NULL) || (offset >= rb_used(rb)))
  {
    return -1;
  }

  return (int)rb->buf[(uint16_t)((rb->tail + offset) & rb_mask(rb))];
}
