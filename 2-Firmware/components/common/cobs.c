#include "cobs.h"

size_t kia_cobs_encode(const uint8_t *src, size_t n, uint8_t *dst, size_t dst_cap)
{
    if (dst_cap < n + 2) return 0;
    size_t   code_idx = 0;
    size_t   out      = 1;
    uint8_t  code     = 0x01;
    for (size_t i = 0; i < n; ++i) {
        if (src[i] == 0) {
            dst[code_idx] = code;
            code_idx      = out++;
            code          = 0x01;
        } else {
            dst[out++] = src[i];
            ++code;
            if (code == 0xFF) {
                dst[code_idx] = code;
                code_idx      = out++;
                code          = 0x01;
            }
        }
        if (out >= dst_cap - 1) return 0;
    }
    dst[code_idx] = code;
    dst[out++]    = 0x00;
    return out;
}

size_t kia_cobs_decode(const uint8_t *src, size_t n, uint8_t *dst, size_t dst_cap)
{
    if (n < 2 || src[n - 1] != 0x00) return 0;
    size_t out = 0;
    size_t i   = 0;
    while (i < n - 1) {
        uint8_t code = src[i++];
        if (code == 0) return 0;
        for (uint8_t k = 1; k < code && i < n - 1; ++k) {
            if (out >= dst_cap) return 0;
            dst[out++] = src[i++];
        }
        if (code != 0xFF && i < n - 1) {
            if (out >= dst_cap) return 0;
            dst[out++] = 0x00;
        }
    }
    return out;
}
