#include "gnss.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "gnss";

// DFRobot Gravity GNSS I2C address
#define GNSS_I2C_ADDR  0x20

// Data register map (from DFRobot_GNSS library)
//
//  Reg  Name           Description
//  ---  ----           -----------
//   0   I2C_YEAR_H     Year high byte
//   1   I2C_YEAR_L     Year low byte
//   2   I2C_MONTH      Month
//   3   I2C_DATE       Day
//   4   I2C_HOUR       Hour (UTC)
//   5   I2C_MINUTE     Minute
//   6   I2C_SECOND     Second
//   7   I2C_LAT_1      Latitude degrees
//   8   I2C_LAT_2      Latitude whole minutes
//   9   I2C_LAT_X_24   Latitude fractional minutes [23:16]
//  10   I2C_LAT_X_16   Latitude fractional minutes [15:8]
//  11   I2C_LAT_X_8    Latitude fractional minutes [7:0]
//  12   I2C_LON_DIS    ** Longitude direction ** (ASCII 'E'/'W')
//  13   I2C_LON_1      Longitude degrees
//  14   I2C_LON_2      Longitude whole minutes
//  15   I2C_LON_X_24   Longitude fractional minutes [23:16]
//  16   I2C_LON_X_16   Longitude fractional minutes [15:8]
//  17   I2C_LON_X_8    Longitude fractional minutes [7:0]
//  18   I2C_LAT_DIS    ** Latitude direction ** (ASCII 'N'/'S')
//  19   I2C_USE_STAR   Number of satellites used
//  20   I2C_ALT_H      Altitude high byte (bit 7 = sign flag)
//  21   I2C_ALT_L      Altitude low byte
//  22   I2C_ALT_X      Altitude fractional (hundredths)
//  23   I2C_SOG_H      Speed over ground high (bit 7 = sign flag)
//  24   I2C_SOG_L      Speed over ground low
//  25   I2C_SOG_X      SOG fractional (hundredths)
//  26   I2C_COG_H      Course over ground high (bit 7 = sign flag)
//  27   I2C_COG_L      COG low
//  28   I2C_COG_X      COG fractional (hundredths)
//
// Note: direction registers are NOT adjacent to their coordinate data.
// Reg 12 is LONGITUDE direction, reg 18 is LATITUDE direction.

#define REG_YEAR_H     0
#define REG_LAT_1      7
#define REG_LON_DIS    12   // longitude direction (ASCII 'E' or 'W')
#define REG_LON_1      13
#define REG_LAT_DIS    18   // latitude direction (ASCII 'N' or 'S')
#define REG_USE_STAR   19
#define REG_ALT_H      20
#define REG_SOG_H      23
#define REG_COG_H      26
#define REG_DATA_LEN   29   // registers 0-28

// Configuration registers
#define REG_GNSS_MODE  34   // constellation mode (1-7)
#define REG_SLEEP_MODE 35   // power control (0x00 = enable, 0x01 = disable)
#define REG_RGB_MODE   36   // RGB LED (0x05 = on, 0x02 = off)

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
    return gnss_write_reg(REG_SLEEP_MODE, 0x00);
}

esp_err_t gnss_set_mode(uint8_t mode)
{
    ESP_LOGI(TAG, "Setting GNSS constellation mode: %d", mode);
    return gnss_write_reg(REG_GNSS_MODE, mode);
}

esp_err_t gnss_set_rgb_off(void)
{
    return gnss_write_reg(REG_RGB_MODE, 0x02);
}

esp_err_t gnss_read(gnss_data_t *data)
{
    uint8_t buf[REG_DATA_LEN];
    esp_err_t err = gnss_read_regs(REG_YEAR_H, buf, REG_DATA_LEN);
    if (err != ESP_OK) {
        return err;
    }

    // Time and date (regs 0-6)
    data->year   = ((uint16_t)buf[0] << 8) | buf[1];
    data->month  = buf[2];
    data->day    = buf[3];
    data->hour   = buf[4];
    data->minute = buf[5];
    data->second = buf[6];

    // Latitude data (regs 7-11), direction at reg 18
    uint8_t lat_dd = buf[7];
    uint8_t lat_mm = buf[8];
    uint32_t lat_frac = ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | buf[11];
    float lat_deg = (float)lat_dd + ((float)lat_mm + (float)lat_frac / 100000.0f) / 60.0f;
    data->latitude = (buf[REG_LAT_DIS] == 'S') ? -lat_deg : lat_deg;

    // Longitude data (regs 13-17), direction at reg 12
    uint8_t lon_dd = buf[13];
    uint8_t lon_mm = buf[14];
    uint32_t lon_frac = ((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 8) | buf[17];
    float lon_deg = (float)lon_dd + ((float)lon_mm + (float)lon_frac / 100000.0f) / 60.0f;
    data->longitude = (buf[REG_LON_DIS] == 'W') ? -lon_deg : lon_deg;

    // Satellites (reg 19)
    data->satellites = buf[19];

    // Altitude (regs 20-22): bit 7 of high byte is sign flag, mask it off
    data->altitude = (double)((((uint16_t)buf[20] & 0x7F) << 8) | buf[21]) + (double)buf[22] / 100.0;

    // Speed over ground (regs 23-25): bit 7 of high byte is sign flag
    data->speed = (double)((((uint16_t)buf[23] & 0x7F) << 8) | buf[24]) + (double)buf[25] / 100.0;

    // Course over ground (regs 26-28): bit 7 of high byte is sign flag
    data->course = (double)((((uint16_t)buf[26] & 0x7F) << 8) | buf[27]) + (double)buf[28] / 100.0;

    return ESP_OK;
}
