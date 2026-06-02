#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *buf;
    size_t   cap;
    _Atomic size_t head;
    _Atomic size_t tail;
} kia_rb_t;

bool   kia_rb_init(kia_rb_t *rb, uint8_t *backing, size_t cap_pow2);
size_t kia_rb_write(kia_rb_t *rb, const uint8_t *src, size_t n);
size_t kia_rb_read(kia_rb_t *rb, uint8_t *dst, size_t n);
size_t kia_rb_available(const kia_rb_t *rb);
size_t kia_rb_free(const kia_rb_t *rb);
void   kia_rb_reset(kia_rb_t *rb);
