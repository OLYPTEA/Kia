#include "move_queue.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "kia_err.h"

static struct {
    move_t          *ring;
    uint16_t         cap;
    volatile uint16_t head, tail;
    SemaphoreHandle_t mtx;
    bool             ready;
} Q;

esp_err_t move_queue_init(uint16_t depth)
{
    if (Q.ready) return ESP_OK;
    Q.ring = calloc(depth, sizeof(move_t));
    if (!Q.ring) return ESP_ERR_NO_MEM;
    Q.cap   = depth;
    Q.mtx   = xSemaphoreCreateMutex();
    Q.ready = Q.mtx != NULL;
    return Q.ready ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t move_queue_push(const move_t *m)
{
    if (!Q.ready) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(Q.mtx, portMAX_DELAY);
    uint16_t nh = (Q.head + 1) % Q.cap;
    esp_err_t e = ESP_OK;
    if (nh == Q.tail) e = ESP_ERR_NO_MEM;
    else {
        Q.ring[Q.head] = *m;
        Q.head         = nh;
    }
    xSemaphoreGive(Q.mtx);
    return e;
}

bool move_queue_pop(move_t *out)
{
    if (!Q.ready) return false;
    bool ok = false;
    xSemaphoreTake(Q.mtx, portMAX_DELAY);
    if (Q.head != Q.tail) {
        *out   = Q.ring[Q.tail];
        Q.tail = (Q.tail + 1) % Q.cap;
        ok     = true;
    }
    xSemaphoreGive(Q.mtx);
    return ok;
}

size_t move_queue_size(void)
{
    if (!Q.ready) return 0;
    return (Q.head + Q.cap - Q.tail) % Q.cap;
}

void move_queue_clear(void)
{
    if (!Q.ready) return;
    xSemaphoreTake(Q.mtx, portMAX_DELAY);
    Q.head = Q.tail = 0;
    xSemaphoreGive(Q.mtx);
}
