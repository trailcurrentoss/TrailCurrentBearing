#include "gnss.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "gnss";

// DFRobot Gravity GNSS I2C address
#define GNSS_I2C_ADDR  0x20

// Data registers (read as a contiguous 30-byte block from register 0x00)
#define REG_YEAR_H     0x00
#define REG_DATA_LEN   30

// Configuration registers
#define REG_GNSS_SET   0x20  // constellation mode
#define REG_SLEEP      0x21  // power control (0x00 = wake, 0x01 = sleep)
#define REG_RGB_MODE   0x22  // RGB LED mode (0x00 = off)

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t gnss_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, -1);
}

static esp_err_t gnss_read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &start_reg, 1, buf, len, -1);
}

esp_err_t gnss_init(int sda_pin, int scl_pin)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GNSS_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GNSS device: %s", esp_err_to_name(err));
        return err;
    }

    // Probe the device with a small read
    uint8_t probe;
    err = gnss_read_regs(0x00, &probe, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GNSS device not found at 0x%02X", GNSS_I2C_ADDR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DFRobot GNSS module found at 0x%02X", GNSS_I2C_ADDR);
    return ESP_OK;
}

esp_err_t gnss_enable_power(void)
{
    return gnss_write_reg(REG_SLEEP, 0x00);
}

esp_err_t gnss_set_mode(uint8_t mode)
{
    ESP_LOGI(TAG, "Setting GNSS constellation mode: %d", mode);
    return gnss_write_reg(REG_GNSS_SET, mode);
}

esp_err_t gnss_set_rgb_off(void)
{
    return gnss_write_reg(REG_RGB_MODE, 0x00);
}

esp_err_t gnss_read(gnss_data_t *data)
{
    uint8_t buf[REG_DATA_LEN];
    esp_err_t err = gnss_read_regs(REG_YEAR_H, buf, REG_DATA_LEN);
    if (err != ESP_OK) {
        return err;
    }

    // Time and date
    data->year   = ((uint16_t)buf[0] << 8) | buf[1];
    data->month  = buf[2];
    data->day    = buf[3];
    data->hour   = buf[4];
    data->minute = buf[5];
    data->second = buf[6];

    // Latitude: degrees(1B) + minutes integer(1B) + minutes fraction(3B) + direction(1B)
    uint8_t lat_dd = buf[7];
    uint8_t lat_mm = buf[8];
    uint32_t lat_frac = ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | buf[11];
    float lat_deg = (float)lat_dd + ((float)lat_mm + (float)lat_frac / 100000.0f) / 60.0f;
    data->latitude = (buf[12] == 'S') ? -lat_deg : lat_deg;

    // Longitude: degrees(1B) + minutes integer(1B) + minutes fraction(3B) + direction(1B)
    uint8_t lon_dd = buf[13];
    uint8_t lon_mm = buf[14];
    uint32_t lon_frac = ((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 8) | buf[17];
    float lon_deg = (float)lon_dd + ((float)lon_mm + (float)lon_frac / 100000.0f) / 60.0f;
    data->longitude = (buf[18] == 'W') ? -lon_deg : lon_deg;

    // Satellites
    data->satellites = buf[19];

    // Altitude: whole(2B) + fractional(1B, /100)
    data->altitude = (double)(((uint16_t)buf[20] << 8) | buf[21]) + (double)buf[22] / 100.0;

    // Speed over ground: whole(2B) + fractional(1B, /100)
    data->speed = (double)(((uint16_t)buf[23] << 8) | buf[24]) + (double)buf[25] / 100.0;

    // Course over ground: whole(2B) + fractional(1B, /100)
    data->course = (double)(((uint16_t)buf[26] << 8) | buf[27]) + (double)buf[28] / 100.0;

    // GNSS mode
    data->gnss_mode = buf[29];

    return ESP_OK;
}
