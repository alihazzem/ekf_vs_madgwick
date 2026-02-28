#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct {
  uint8_t *buf;
  size_t   size;
  volatile size_t head;
  volatile size_t tail;
} ringbuf_t;

void   rb_init(ringbuf_t *rb, uint8_t *storage, size_t size);
int    rb_push(ringbuf_t *rb, uint8_t b);      // 0 ok, -1 full
int    rb_pop (ringbuf_t *rb, uint8_t *out);   // 0 ok, -1 empty
