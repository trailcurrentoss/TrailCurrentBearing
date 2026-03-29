#pragma once

#include <stdint.h>
#include "esp_err.h"

// DFRobot Gravity GNSS constellation modes
#define GNSS_MODE_GPS                1
#define GNSS_MODE_BEIDOU             2
#define GNSS_MODE_GPS_BEIDOU         3
#define GNSS_MODE_GLONASS            4
#define GNSS_MODE_GPS_GLONASS        5
#define GNSS_MODE_BEIDOU_GLONASS     6
#define GNSS_MODE_GPS_BEIDOU_GLONASS 7

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    float    latitude;    // decimal degrees, negative = south
    float    longitude;   // decimal degrees, negative = west
    double   altitude;    // meters
    uint8_t  satellites;  // number of satellites in use
    double   speed;       // speed over ground (knots)
    double   course;      // course over ground (degrees)
    uint8_t  gnss_mode;   // active constellation mode
} gnss_data_t;

/**
 * Initialize I2C bus and probe for the DFRobot GNSS module.
 * Returns ESP_OK on success, ESP_FAIL if the device is not found.
 */
esp_err_t gnss_init(int sda_pin, int scl_pin);

/**
 * Wake the GNSS module from sleep (enable power).
 */
esp_err_t gnss_enable_power(void);

/**
 * Select the satellite constellation(s) to use.
 */
esp_err_t gnss_set_mode(uint8_t mode);

/**
 * Turn off the on-board RGB LED to save power.
 */
esp_err_t gnss_set_rgb_on(void);
esp_err_t gnss_set_rgb_off(void);

/**
 * Read all GNSS data from the module via I2C.
 * Populates the gnss_data_t structure with the latest fix.
 */
esp_err_t gnss_read(gnss_data_t *data);
