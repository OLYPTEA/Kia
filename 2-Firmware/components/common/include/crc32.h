#pragma once

#include <stddef.h>
#include <stdint.h>

/* CRC-32/ISO-HDLC (poly 0xEDB88320, init 0xFFFFFFFF, refin/refout=true, xorout=0xFFFFFFFF) */
uint32_t kia_crc32(const void *data, size_t n);
uint32_t kia_crc32_update(uint32_t crc, const void *data, size_t n);
