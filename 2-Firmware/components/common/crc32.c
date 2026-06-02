#include "crc32.h"

uint32_t kia_crc32_update(uint32_t crc, const void *data, size_t n)
{
    const uint8_t *p = (const uint8_t *)data;
    crc              = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1));
    }
    return ~crc;
}

uint32_t kia_crc32(const void *data, size_t n)
{
    return kia_crc32_update(0, data, n);
}
