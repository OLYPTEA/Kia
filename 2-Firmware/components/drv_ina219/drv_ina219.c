#include "drv_ina219.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "app_config.h"
#include "kia_err.h"

#define REG_CFG     0x00
#define REG_SHUNT   0x01
#define REG_BUS     0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CAL     0x05

static const char *TAG = "ina219";

static struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint16_t                shunt_mohm;
    int32_t                 lsb_ua;  /* current LSB in uA */
    bool                    ready;
} S;

static esp_err_t wr16(uint8_t reg, uint16_t val)
{
    uint8_t b[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_transmit(S.dev, b, sizeof b, 100);
}

static esp_err_t rd16(uint8_t reg, uint16_t *val)
{
    uint8_t r[2];
    KIA_RET(i2c_master_transmit_receive(S.dev, &reg, 1, r, 2, 100));
    *val = ((uint16_t)r[0] << 8) | r[1];
    return ESP_OK;
}

esp_err_t drv_ina219_set_shunt_mohm(uint16_t mohm)
{
    S.shunt_mohm = mohm;
    /* I_max = 8 A → current_lsb = 8/32767 ≈ 244 uA. Use 250 uA for clean numbers. */
    S.lsb_ua    = 250;
    uint16_t cal = (uint16_t)(40960 / (S.lsb_ua * mohm / 1000));
    return wr16(REG_CAL, cal);
}

esp_err_t drv_ina219_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));

    i2c_master_bus_config_t bcfg = {
        .i2c_port                     = KIA_I2C_PORT,
        .sda_io_num                   = KIA_GPIO_I2C_SDA,
        .scl_io_num                   = KIA_GPIO_I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    /* Bus may already be created elsewhere — tolerate. */
    esp_err_t e = i2c_new_master_bus(&bcfg, &S.bus);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;

    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = KIA_I2C_INA219_ADDR,
        .scl_speed_hz    = KIA_I2C_FREQ_HZ,
    };
    KIA_RET(i2c_master_bus_add_device(S.bus, &dcfg, &S.dev));

    /* Probe */
    uint16_t cfg;
    e = i2c_master_probe(S.bus, KIA_I2C_INA219_ADDR, 100);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "INA219 not found");
        return e;
    }

    /* Cfg : BRNG=32V, PG=/8 (320 mV), BADC/SADC=12bit 532us, continuous shunt+bus */
    KIA_RET(wr16(REG_CFG, 0x399F));
    KIA_RET(drv_ina219_set_shunt_mohm(10));

    KIA_RET(rd16(REG_CFG, &cfg));
    ESP_LOGI(TAG, "INA219 OK cfg=0x%04x", cfg);
    S.ready = true;
    return ESP_OK;
}

esp_err_t drv_ina219_read(drv_ina219_sample_t *o)
{
    KIA_RET_IF(!S.ready || !o, ESP_ERR_INVALID_STATE);
    uint16_t bus, shunt, cur, pow;
    KIA_RET(rd16(REG_BUS, &bus));
    KIA_RET(rd16(REG_SHUNT, &shunt));
    KIA_RET(rd16(REG_CURRENT, &cur));
    KIA_RET(rd16(REG_POWER, &pow));

    /* Bus voltage : bits [15:3], 4 mV/LSB */
    o->bus_mv   = (int16_t)((bus >> 3) * 4);
    o->shunt_uv = (int16_t)shunt * 10;
    o->current_ma = ((int16_t)cur * S.lsb_ua) / 1000;
    o->power_mw   = ((int16_t)pow * S.lsb_ua * 20) / 1000;
    return ESP_OK;
}
