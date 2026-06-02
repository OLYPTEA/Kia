#pragma once

#include <stddef.h>
#include <stdint.h>

/* Consistent Overhead Byte Stuffing. 0x00 is reserved as packet delimiter. */

size_t kia_cobs_encode(const uint8_t *src, size_t n, uint8_t *dst, size_t dst_cap);
size_t kia_cobs_decode(const uint8_t *src, size_t n, uint8_t *dst, size_t dst_cap);
