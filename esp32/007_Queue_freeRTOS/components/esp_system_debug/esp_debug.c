/**
 * @file esp_debug.c
 * @brief Firmware debugging and logging functions.
 * * @author Rahul B.
 * @version 1.0
 * @date August 2025
 */

#include "esp_debub.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include <stdlib.h>
#include <string.h>

// for idf V5
#include "esp_mac.h"

static const char *TAG = "esp_debug";

void print_mac_address(void)
{
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA); // Get MAC for Wi-Fi Station

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read MAC address, error: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Firmware shutdown handler.
 * * This function is registered to be called before the device reboots.
 * It provides a snapshot of the running tasks and a shutdown message,
 * which is useful for debugging and firmware stability analysis.
 */
void FirmwareShutdownHandler(void)
{
    printf("............ In the shutdown handler ...................\n");
    static char buf[1024];
    TaskHandle_t currentTaskHandle = NULL;
    currentTaskHandle = xTaskGetCurrentTaskHandle();
    printf(">>> CurrentTaskHandle: %s <<<\n", pcTaskGetName(currentTaskHandle));
    vTaskList(buf);
    printf(">>>>>>>>>>>>>> Vtasklist Info <<<<<<<<<<<<<\n");
    printf("  Name        State   Priority  Stack   Num\n");
    printf("_____________________________________________\n");
    printf("%s\n", buf);
    printf("******************* Shutting down **************************\n");
}

/**
 * @brief Logs the reset reason of the ESP32 and registers a shutdown handler.
 * * This provides critical information for debugging unexpected resets,
 * such as a crash or a watchdog timer event.
 */
void ResetReason(void)
{

    esp_reset_reason_t resetReason;
    const char *swResetReasons[] = {"ESP_RST_UNKNOWN",
                                    "ESP_RST_POWERON",
                                    "ESP_RST_EXT",
                                    "ESP_RST_SW",
                                    "ESP_RST_PANIC",
                                    "ESP_RST_INT_WDT",
                                    "ESP_RST_TASK_WDT",
                                    "ESP_RST_WDT",
                                    "ESP_RST_DEEPSLEEP",
                                    "ESP_RST_BROWNOUT",
                                    "ESP_RST_SDIO"};

    shutdown_handler_t handler = FirmwareShutdownHandler;
    esp_err_t ret = esp_register_shutdown_handler(handler);
    printf("Shutdown Handler result : %s\n", esp_err_to_name(ret));
    resetReason = esp_reset_reason();
    printf("Reset Reason: %s\n", swResetReasons[resetReason]);

    printf("Software Start\n");
    print_mac_address();
}
