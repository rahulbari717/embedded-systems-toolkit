/**
 * @file config.c
 * @brief Kconfig-based firmware configuration functions.
 * @author Rahul B.
 * @version 1.0
 * @date August 2025
 */

#include "esp_config.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "menu_config";

/**
 * @brief Initializes and logs all Kconfig menu settings.
 * * This function reads various configuration values set via `idf.py menuconfig`
 * and logs them to the console for verification.
 */
void menu_config(void)
{
    esp_log_level_set("LOG", ESP_LOG_DEBUG);
    // ESP_LOGI(TAG, "MY_INT %d", CONFIG_MY_INT);
    // ESP_LOGI(TAG, "MY_STRING %s", CONFIG_MY_STRING);

    //     bool my_bool = false;
    // #ifdef CONFIG_MY_BOOL
    //     my_bool = true;
    // #else
    //     my_bool = false;
    // #endif
    //     // ESP_LOGI(TAG, "MY_BOOL %s", my_bool ? "yes" : "no");
    //     int option = 0;
    // #ifdef CONFIG_OPTION_1
    //     option = 1;
    // #elif CONFIG_OPTION_2
    //     option = 2;
    // #else
    //     option = 3;
    // #endif
    // ESP_LOGI(TAG, "MY_OPTION %d", option);

    ESP_LOGD(TAG, "WiFi SSID: %s", CONFIG_WIFI_SSID);
    ESP_LOGD(TAG, "WiFi Password: %s", CONFIG_WIFI_PASSWORD);
    // ESP_LOGI(TAG, "LED Pins: %d, %d, %d", CONFIG_LED_PIN_1, CONFIG_LED_PIN_2, CONFIG_LED_PIN_3);
    // ESP_LOGI(TAG, "Button Pins: %d, %d", CONFIG_BUTTON_PIN_1, CONFIG_BUTTON_PIN_2);
}
