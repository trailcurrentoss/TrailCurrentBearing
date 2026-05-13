#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_common.h"
#include "wifi_config.h"
#include "ota.h"
#include "discovery.h"
#include "gnss.h"

static const char *TAG = "bearing";

// Waveshare ESP32-S3-RS485-CAN pin assignments
#define CAN_TX_PIN   15
#define CAN_RX_PIN   16
#define I2C_SDA_PIN  1
#define I2C_SCL_PIN  2

// CAN message identifiers (TX — GNSS data)
#define CAN_ID_DATETIME   0x06
#define CAN_ID_SAT_SPEED  0x07
#define CAN_ID_ALTITUDE   0x08
#define CAN_ID_LATLON     0x09

// CAN message identifiers (RX — control triggers)
#define CAN_ID_OTA              0x00
#define CAN_ID_WIFI_CONFIG      0x01
#define CAN_ID_DISCOVERY        0x02

// CAN transmit timing
#define CAN_STATUS_PERIOD_MS   33    // ~30 Hz normal rate
#define TX_PROBE_INTERVAL_MS   2000  // slow probe when no peers

// GNSS poll interval
#define GNSS_POLL_MS  100

// ---------------------------------------------------------------------------
// Shared GNSS data (written by main task, read by CAN task)
// volatile prevents single-access reordering but does NOT make the multi-field
// snapshot atomic — a spinlock guards the whole group so the CAN task never
// captures torn values across a GNSS update (e.g. mixed minute/second on the
// rollover that made the time appear to "jump").
// ---------------------------------------------------------------------------

static portMUX_TYPE g_gnss_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_year;
static uint8_t  g_month, g_day, g_hour, g_minute, g_second;
static float    g_latitude, g_longitude;
static double   g_altitude, g_speed, g_course;
static uint8_t  g_satellites, g_gnss_mode;

// ---------------------------------------------------------------------------
// CAN data encoding helpers
// ---------------------------------------------------------------------------

static void encode_lat_lon(float value, uint8_t out[4])
{
    out[0] = (value < 0) ? 1 : 0;
    if (value < 0) value = -value;
    uint32_t scaled = (uint32_t)(value * 10000.0f + 0.5f);
    out[1] = (scaled >> 16) & 0xFF;
    out[2] = (scaled >> 8) & 0xFF;
    out[3] = scaled & 0xFF;
}

// ---------------------------------------------------------------------------
// TWAI (CAN) task — runs independently from I2C polling
// ---------------------------------------------------------------------------


static void twai_task(void *arg)
{
    // Configure alerts BEFORE any bus activity so no error transitions are missed.
    twai_reconfigure_alerts(CAN_COMMON_ALERTS, NULL);

    // Alerts armed — version broadcast TX failure is caught by the state machine.
    can_common_version_broadcast();

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = CAN_STATUS_PERIOD_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));

        // Bus-off recovery
        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            // No continue — fall through so RX_DATA in the same poll is still processed.
        }

        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
        }

        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive (no peers ACKing?)");
        }

        // TX failure tracking — graceful backoff when no peers on bus
        if (triggered & TWAI_ALERT_TX_FAILED) {
            if (tx_state == TX_ACTIVE) {
                tx_fail_count++;
                if (tx_fail_count >= TX_FAIL_THRESHOLD) {
                    tx_state = TX_PROBING;
                    ESP_LOGW(TAG, "TWAI no peers detected, entering slow probe");
                }
            }
        }

        // TX success — peer detected, resume normal rate
        if (triggered & TWAI_ALERT_TX_SUCCESS) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI probe ACK'd, peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        // Drain received messages and dispatch
        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;

                switch (msg.identifier) {
                case CAN_ID_OTA:
                    ota_handle_trigger(msg.data, msg.data_length_code);
                    break;
                case CAN_ID_WIFI_CONFIG:
                    wifi_config_handle_can(msg.data, msg.data_length_code);
                    break;
                case CAN_ID_DISCOVERY:
                    discovery_handle_trigger();
                    break;
                default:
                    break;
                }
            }
        }

        // Check wifi config timeout
        wifi_config_check_timeout();

        // Periodic transmit (skip if bus is down)
        int64_t now_us = esp_timer_get_time();
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now_us - last_tx_us >= effective_period)) {
            last_tx_us = now_us;

            // Snapshot shared GNSS data atomically — without the spinlock the
            // CAN task could read e.g. minute from one sample and second from
            // the next, producing jumps on every rollover boundary.
            uint16_t year;
            uint8_t month, day, hour, minute, second;
            float lat, lon;
            double alt, spd, crs;
            uint8_t sats, mode;
            taskENTER_CRITICAL(&g_gnss_mux);
            year = g_year;
            month = g_month; day = g_day;
            hour = g_hour; minute = g_minute; second = g_second;
            lat = g_latitude; lon = g_longitude;
            alt = g_altitude; spd = g_speed; crs = g_course;
            sats = g_satellites; mode = g_gnss_mode;
            taskEXIT_CRITICAL(&g_gnss_mux);

            // 0x06: DateTime [year_h, year_l, month, day, hour, minute, second]
            twai_message_t m_dt = {
                .identifier = CAN_ID_DATETIME,
                .data_length_code = 7,
                .data = {
                    (year >> 8) & 0xFF, year & 0xFF,
                    month, day, hour, minute, second
                }
            };

            // 0x07: [satellites, speed_h, speed_l, course_h, course_l, gnss_mode]
            uint16_t speed_scaled = (uint16_t)(spd * 100.0);
            uint16_t course_scaled = (uint16_t)(crs * 10.0 + 0.5);
            twai_message_t m_nav = {
                .identifier = CAN_ID_SAT_SPEED,
                .data_length_code = 6,
                .data = {
                    sats,
                    (speed_scaled >> 8) & 0xFF, speed_scaled & 0xFF,
                    (course_scaled >> 8) & 0xFF, course_scaled & 0xFF,
                    mode
                }
            };

            // 0x08: [alt_3, alt_2, alt_1, alt_0] (altitude * 100)
            uint32_t alt_scaled = (uint32_t)(alt * 100.0);
            twai_message_t m_alt = {
                .identifier = CAN_ID_ALTITUDE,
                .data_length_code = 4,
                .data = {
                    (alt_scaled >> 24) & 0xFF,
                    (alt_scaled >> 16) & 0xFF,
                    (alt_scaled >> 8) & 0xFF,
                    alt_scaled & 0xFF
                }
            };

            // 0x09: [lat_sign, lat_2, lat_1, lat_0, lon_sign, lon_2, lon_1, lon_0]
            uint8_t lat_enc[4], lon_enc[4];
            encode_lat_lon(lat, lat_enc);
            encode_lat_lon(lon, lon_enc);
            twai_message_t m_pos = {
                .identifier = CAN_ID_LATLON,
                .data_length_code = 8,
                .data = {
                    lat_enc[0], lat_enc[1], lat_enc[2], lat_enc[3],
                    lon_enc[0], lon_enc[1], lon_enc[2], lon_enc[3]
                }
            };

            twai_transmit(&m_dt, 0);
            twai_transmit(&m_nav, 0);
            twai_transmit(&m_alt, 0);
            twai_transmit(&m_pos, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// Main application
// ---------------------------------------------------------------------------

void app_main(void)
{
    // Initialize WiFi config (NVS, hostname, credentials)
    wifi_config_init();
    char ssid[33], password[64];
    wifi_config_load(ssid, sizeof(ssid), password, sizeof(password));

    ota_init();
    discovery_init();

    ESP_LOGI(TAG, "=== TrailCurrent Bearing ===");
    ESP_LOGI(TAG, "Hostname: %s", wifi_config_get_hostname());

    // CAN runs in its own task — start before GNSS so discovery/OTA work
    // even while waiting for the GNSS module
    ESP_ERROR_CHECK(can_common_init(CAN_TX_PIN, CAN_RX_PIN));
    xTaskCreatePinnedToCore(twai_task, "twai", 4096, NULL, 5, NULL, 1);

    // Initialize DFRobot GNSS module via I2C
    while (gnss_init(I2C_SDA_PIN, I2C_SCL_PIN) != ESP_OK) {
        ESP_LOGW(TAG, "GNSS module not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    gnss_enable_power();
    gnss_set_mode(GNSS_MODE_GPS_BEIDOU_GLONASS);
    gnss_set_rgb_on();

    // Main task: poll GNSS data via I2C
    uint32_t log_counter = 0;
    while (1) {
        gnss_data_t data;
        if (gnss_read(&data) == ESP_OK) {
            bool date_valid =
                data.year  >= 2025 && data.year  <= 2099 &&
                data.month >= 1    && data.month <= 12 &&
                data.day   >= 1    && data.day   <= 31 &&
                data.hour  <= 23 &&
                data.minute <= 59 &&
                data.second <= 60;

            if (++log_counter >= (1000 / GNSS_POLL_MS)) {
                log_counter = 0;
                ESP_LOGI(TAG, "GNSS raw: %04u-%02u-%02u %02u:%02u:%02u  sats=%u  date_valid=%d",
                         data.year, data.month, data.day,
                         data.hour, data.minute, data.second,
                         data.satellites, date_valid);
            }
            taskENTER_CRITICAL(&g_gnss_mux);
            if (date_valid) {
                g_year      = data.year;
                g_month     = data.month;
                g_day       = data.day;
                g_hour      = data.hour;
                g_minute    = data.minute;
                g_second    = data.second;
            }
            g_latitude  = data.latitude;
            g_longitude = data.longitude;
            g_altitude  = data.altitude;
            g_satellites = data.satellites;
            g_speed     = data.speed;
            g_course    = data.course;
            g_gnss_mode = data.gnss_mode;
            taskEXIT_CRITICAL(&g_gnss_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_MS));
    }
}
