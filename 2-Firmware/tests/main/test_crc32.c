#include <string.h>

#include "unity.h"

#include "crc32.h"

TEST_CASE("CRC32 check '123456789'", "[crc32]")
{
    const char *s = "123456789";
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926, kia_crc32(s, strlen(s)));
}

TEST_CASE("CRC32 empty input", "[crc32]")
{
    TEST_ASSERT_EQUAL_HEX32(0x00000000, kia_crc32("", 0));
}

TEST_CASE("CRC32 streaming equals one-shot", "[crc32]")
{
    const uint8_t data[64] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    uint32_t      one      = kia_crc32(data, sizeof data);
    uint32_t      a        = 0;
    a                      = kia_crc32_update(a, data, 16);
    a                      = kia_crc32_update(a, data + 16, 16);
    a                      = kia_crc32_update(a, data + 32, 32);
    TEST_ASSERT_EQUAL_HEX32(one, a);
}
