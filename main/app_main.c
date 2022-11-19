/*
 * Copyright (c) 2022 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "shell.h"
#include "wifi.h"
#include "golioth.h"

#if CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
#include <driver/gpio.h>
#endif
#include "lc709203f.h"

#define TAG "Frog"

static void on_client_event(golioth_client_t client, golioth_client_event_t event, void* arg) {
    ESP_LOGI(
            TAG,
            "Golioth client %s",
            event == GOLIOTH_CLIENT_EVENT_CONNECTED ? "connected" : "disconnected");
}

static esp_err_t initialize_lc709203f(i2c_dev_t *lc)
{
    if (!lc)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(lc709203f_set_power_mode(lc, LC709203F_POWER_MODE_OPERATIONAL));
    // Using 2500mAh LiPo battery. Check Datasheet graph for APA values by battery type & mAh
    ESP_ERROR_CHECK(lc709203f_set_apa(lc, 0x32));
    ESP_ERROR_CHECK(lc709203f_set_battery_profile(lc, LC709203F_BATTERY_PROFILE_1));
    ESP_ERROR_CHECK(lc709203f_initial_rsoc(lc));
    ESP_ERROR_CHECK(lc709203f_set_temp_mode(lc, LC709203F_TEMP_MODE_I2C));
    ESP_ERROR_CHECK(lc709203f_set_cell_temperature_celsius(lc, 20));

    uint16_t value = 0;
    ESP_ERROR_CHECK(lc709203f_get_power_mode(lc, (lc709203f_power_mode_t *)&value));
    ESP_LOGI(TAG, "Power Mode (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_apa(lc, (uint8_t *)&value));
    ESP_LOGI(TAG, "APA (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_battery_profile(lc, (lc709203f_battery_profile_t *)&value));
    ESP_LOGI(TAG, "Battery Profile (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_temp_mode(lc, (lc709203f_temp_mode_t *)&value));
    ESP_LOGI(TAG, "Temp Mode (lc709203f): 0x%X", value);

    return ESP_OK;
}

void init_sensors(void *pvParameters){
    i2c_dev_t lc;

    // lc709203f
    uint16_t voltage = 0, rsoc = 0, ite = 0;
    float bat_temp = -274;

    memset(&lc, 0, sizeof(lc));
    ESP_ERROR_CHECK(lc709203f_init_desc(&lc, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));
    initialize_lc709203f(&lc);

    while (1)
    {
        ESP_ERROR_CHECK(lc709203f_get_cell_voltage(&lc, &voltage));
        ESP_ERROR_CHECK(lc709203f_get_rsoc(&lc, &rsoc));
        ESP_ERROR_CHECK(lc709203f_get_cell_ite(&lc, &ite));
        // Temperature in I2C mode. Temperature should be the same as configured.
        ESP_ERROR_CHECK(lc709203f_get_cell_temperature_celsius(&lc, &bat_temp));
        ESP_LOGI(TAG, "Temp (lc709203f): %.1f", bat_temp);
        ESP_LOGI(TAG, "Voltage (lc709203f): %.2f", voltage / 1000.0);
        ESP_LOGI(TAG, "RSOC (lc709203f): %d%%", rsoc);
        ESP_LOGI(TAG, "ITE (lc709203f): %.1f%%", ite / 10.0);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void) {
#if CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
    /// Adafruit Feather esp32ss/s3 needs to set GPIO7 as HIGH level output to enable onboard I2C pull ups
    /// We needn't internal pull ups.
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1 << CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT, CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT_LEVEL);
#endif

    nvs_init();
    shell_start();

    if (!nvs_credentials_are_set()) {
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGW(TAG, "WiFi and golioth credentials are not set");
            ESP_LOGW(TAG, "Use the shell settings commands to set them, then restart");
            vTaskDelay(portMAX_DELAY);
        }
    }

    wifi_init(nvs_read_wifi_ssid(), nvs_read_wifi_password());
    wifi_wait_for_connected();

    const char* psk_id = nvs_read_golioth_psk_id();
    const char* psk = nvs_read_golioth_psk();

    golioth_client_config_t config = {
            .credentials = {
                    .auth_type = GOLIOTH_TLS_AUTH_TYPE_PSK,
                    .psk = {
                            .psk_id = psk_id,
                            .psk_id_len = strlen(psk_id),
                            .psk = psk,
                            .psk_len = strlen(psk),
                    }}};
    golioth_client_t client = golioth_client_create(&config);
    assert(client);

    golioth_client_register_event_callback(client, on_client_event, NULL);

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(init_sensors, "init_sensors", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);

    int32_t iteration = 0;
    while (1) {
        golioth_lightdb_set_int_async(client, "iteration", iteration, NULL, NULL);
        iteration++;
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    };
}
