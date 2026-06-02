#include "ringbuf.h"

#include <string.h>

static inline bool is_pow2(size_t x) { return x && ((x & (x - 1)) == 0); }

bool kia_rb_init(kia_rb_t *rb, uint8_t *backing, size_t cap_pow2)
{
    if (!rb || !backing || !is_pow2(cap_pow2)) return false;
    rb->buf = backing;
    rb->cap = cap_pow2;
    atomic_store_explicit(&rb->head, 0, memory_order_relaxed);
    atomic_store_explicit(&rb->tail, 0, memory_order_relaxed);
    return true;
}

size_t kia_rb_write(kia_rb_t *rb, const uint8_t *src, size_t n)
{
    size_t head = atomic_load_explicit(&rb->head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&rb->tail, memory_order_acquire);
    size_t mask = rb->cap - 1;
    size_t free_ = rb->cap - (head - tail);
    if (n > free_) n = free_;

    size_t first = rb->cap - (head & mask);
    if (first > n) first = n;
    memcpy(rb->buf + (head & mask), src, first);
    if (n > first) memcpy(rb->buf, src + first, n - first);

    atomic_store_explicit(&rb->head, head + n, memory_order_release);
    return n;
}

size_t kia_rb_read(kia_rb_t *rb, uint8_t *dst, size_t n)
{
    size_t head = atomic_load_explicit(&rb->head, memory_order_acquire);
    size_t tail = atomic_load_explicit(&rb->tail, memory_order_relaxed);
    size_t mask = rb->cap - 1;
    size_t avail = head - tail;
    if (n > avail) n = avail;

    size_t first = rb->cap - (tail & mask);
    if (first > n) first = n;
    memcpy(dst, rb->buf + (tail & mask), first);
    if (n > first) memcpy(dst + first, rb->buf, n - first);

    atomic_store_explicit(&rb->tail, tail + n, memory_order_release);
    return n;
}

size_t kia_rb_available(const kia_rb_t *rb)
{
    size_t head = atomic_load_explicit(&rb->head, memory_order_acquire);
    size_t tail = atomic_load_explicit(&rb->tail, memory_order_acquire);
    return head - tail;
}

size_t kia_rb_free(const kia_rb_t *rb)
{
    return rb->cap - kia_rb_available(rb);
}

void kia_rb_reset(kia_rb_t *rb)
{
    atomic_store_explicit(&rb->head, 0, memory_order_relaxed);
    atomic_store_explicit(&rb->tail, 0, memory_order_relaxed);
}
