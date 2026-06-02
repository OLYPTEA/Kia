#include <string.h>

#include "unity.h"

#include "cobs.h"

TEST_CASE("COBS roundtrip simple", "[cobs]")
{
    const uint8_t src[]  = {1, 2, 3, 0, 4, 5};
    uint8_t       enc[16] = {0};
    uint8_t       dec[16] = {0};
    size_t e = kia_cobs_encode(src, sizeof src, enc, sizeof enc);
    TEST_ASSERT_GREATER_THAN(0, e);
    TEST_ASSERT_EQUAL(0x00, enc[e - 1]);
    size_t d = kia_cobs_decode(enc, e, dec, sizeof dec);
    TEST_ASSERT_EQUAL(sizeof src, d);
    TEST_ASSERT_EQUAL_MEMORY(src, dec, sizeof src);
}

TEST_CASE("COBS roundtrip all-zeros", "[cobs]")
{
    uint8_t src[10] = {0};
    uint8_t enc[32] = {0}, dec[32] = {0};
    size_t  e = kia_cobs_encode(src, sizeof src, enc, sizeof enc);
    size_t  d = kia_cobs_decode(enc, e, dec, sizeof dec);
    TEST_ASSERT_EQUAL(sizeof src, d);
    TEST_ASSERT_EQUAL_MEMORY(src, dec, sizeof src);
}

TEST_CASE("COBS roundtrip 255-block boundary", "[cobs]")
{
    uint8_t src[300];
    for (size_t i = 0; i < sizeof src; ++i) src[i] = (uint8_t)((i % 250) + 1);
    uint8_t enc[400] = {0}, dec[400] = {0};
    size_t  e = kia_cobs_encode(src, sizeof src, enc, sizeof enc);
    size_t  d = kia_cobs_decode(enc, e, dec, sizeof dec);
    TEST_ASSERT_EQUAL(sizeof src, d);
    TEST_ASSERT_EQUAL_MEMORY(src, dec, sizeof src);
}
