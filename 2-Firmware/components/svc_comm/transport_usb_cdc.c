#include "svc_comm.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "kia_err.h"

static const char *TAG = "tr_usb";

static int usb_send(comm_transport_id_t t, const uint8_t *buf, size_t n)
{
    KIA_UNUSED(t);
    return usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(50));
}

static void rx_task(void *arg)
{
    KIA_UNUSED(arg);
    uint8_t buf[256];
    while (true) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(50));
        if (n > 0) svc_comm_on_rx(COMM_TRANSPORT_USB, buf, n);
    }
}

esp_err_t transport_usb_cdc_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    KIA_RET(usb_serial_jtag_driver_install(&cfg));
    svc_comm_register_sender(COMM_TRANSPORT_USB, usb_send);
    xTaskCreatePinnedToCore(rx_task, "usb_rx", 4096, NULL, KIA_PRIO_COMM, NULL, KIA_CORE_IO);
    ESP_LOGI(TAG, "USB CDC up");
    return ESP_OK;
}
