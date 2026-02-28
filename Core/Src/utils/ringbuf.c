#include "utils/ringbuf.h"

void rb_init(ringbuf_t *rb, uint8_t *storage, size_t size) {
  rb->buf = storage;
  rb->size = size;
  rb->head = 0;
  rb->tail = 0;
}

static size_t next_i(const ringbuf_t *rb, size_t i) {
  return (i + 1U) % rb->size;
}

int rb_push(ringbuf_t *rb, uint8_t b) {
  size_t n = next_i(rb, rb->head);
  if (n == rb->tail) return -1; // full
  rb->buf[rb->head] = b;
  rb->head = n;
  return 0;
}

int rb_pop(ringbuf_t *rb, uint8_t *out) {
  if (rb->tail == rb->head) return -1; // empty
  *out = rb->buf[rb->tail];
  rb->tail = next_i(rb, rb->tail);
  return 0;
}
