#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <time.h>
#include "esp_sntp.h"

#define TAG "NTP TIME"
#define TAG_DEBUG "NTP_DEBUG"

SemaphoreHandle_t got_time_semaphore;

void print_time()
{
    esp_log_level_set(TAG_DEBUG, ESP_LOG_DEBUG);
    time_t now = 0;
    time(&now);
    struct tm *time_info = localtime(&now);

    char time_buffer[50];
    strftime(time_buffer, sizeof(time_buffer), "%c", time_info);
    ESP_LOGW(TAG, "************ %s ***********", time_buffer);
}

void on_got_time(struct timeval *tv)
{
    // printf("on got callback %lld\n", tv->tv_sec);
    // print_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(got_time_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void sntp(void)
{

    // ADD THESE LINES AT THE START:
    if (esp_sntp_enabled())
    {
        esp_sntp_stop();
        vTaskDelay(pdMS_TO_TICKS(200)); // Small delay to ensure cleanup
    }

    esp_log_level_set(TAG_DEBUG, ESP_LOG_DEBUG);
    got_time_semaphore = xSemaphoreCreateBinary();

    // Set Indian Standard Time (UTC+5:30)
    setenv("TZ", "IST-5:30", 1);
    tzset();

    ESP_LOGI(TAG, "Before NTP sync (local time may be wrong)");
    // print_time();

    // Configure SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(on_got_time);
    esp_sntp_init();

    // Wait until we actually get time from server
    if (xSemaphoreTake(got_time_semaphore, pdMS_TO_TICKS(20000)) == pdTRUE)
    {
        ESP_LOGI(TAG, "SNTP sync successful!");
    }
    else
    {
        ESP_LOGE(TAG, "SNTP sync timed out!");
    }

    // Print a few samples
    for (int i = 0; i < 3; i++)
    {
        print_time();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vSemaphoreDelete(got_time_semaphore);
    got_time_semaphore = NULL;

    esp_sntp_stop();
}