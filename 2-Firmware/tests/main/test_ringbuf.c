#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

#include "ringbuf.h"

TEST_CASE("Ringbuf basic write/read", "[ringbuf]")
{
    static uint8_t backing[64];
    kia_rb_t       rb;
    TEST_ASSERT_TRUE(kia_rb_init(&rb, backing, sizeof backing));

    const uint8_t src[] = "hello world";
    size_t        w     = kia_rb_write(&rb, src, sizeof src);
    TEST_ASSERT_EQUAL(sizeof src, w);
    TEST_ASSERT_EQUAL(sizeof src, kia_rb_available(&rb));

    uint8_t out[32] = {0};
    size_t  r       = kia_rb_read(&rb, out, sizeof out);
    TEST_ASSERT_EQUAL(sizeof src, r);
    TEST_ASSERT_EQUAL_MEMORY(src, out, sizeof src);
    TEST_ASSERT_EQUAL(0, kia_rb_available(&rb));
}

TEST_CASE("Ringbuf wraparound", "[ringbuf]")
{
    static uint8_t backing[8];
    kia_rb_t       rb;
    TEST_ASSERT_TRUE(kia_rb_init(&rb, backing, sizeof backing));

    uint8_t pad[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    kia_rb_write(&rb, pad, sizeof pad);
    uint8_t junk[5];
    kia_rb_read(&rb, junk, sizeof junk);

    uint8_t payload[6] = {1, 2, 3, 4, 5, 6};
    TEST_ASSERT_EQUAL(6, kia_rb_write(&rb, payload, sizeof payload));
    uint8_t out[6] = {0};
    TEST_ASSERT_EQUAL(6, kia_rb_read(&rb, out, sizeof out));
    TEST_ASSERT_EQUAL_MEMORY(payload, out, sizeof payload);
}

TEST_CASE("Ringbuf rejects non-power-of-two", "[ringbuf]")
{
    static uint8_t backing[100];
    kia_rb_t       rb;
    TEST_ASSERT_FALSE(kia_rb_init(&rb, backing, sizeof backing));
}
