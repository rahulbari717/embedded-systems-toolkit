/**
 * @file main.c
 * @brief Main application entry point for the Smart Multi-Sensor Hub.
 * @author Rahul B.
 * @version 1.0
 * @date December 2025
 */

#include "esp_debub.h"

#include "mqtt_client.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

#include "main.h"
#include "esp_config.h"
#include "my_nvs_storage.h"
#include "wifi_connect.h"

static const char *TAG = "MAIN";

// Global Definitions
TaskHandle_t handle_cmd_task;
TaskHandle_t handle_menu_task;
TaskHandle_t handle_print_task;
TaskHandle_t handle_led_task;
TaskHandle_t handle_rtc_task;

QueueHandle_t q_data;
QueueHandle_t q_print;
SemaphoreHandle_t print_mutex;

TimerHandle_t handle_led_timer[4];
state_t curr_state = sMainMenu;

/**
 * @brief The main application entry point.
 * * This function initializes the device, configures settings,
 * initializes and uses the NVS storage, and then runs the main application loop.
 */
void app_main()
{

	initialize_and_use_nvs();
    menu_config();

     // 1. Create Mutex
    print_mutex = xSemaphoreCreateMutex();
    configASSERT(print_mutex != NULL);

    init_gpio();
    init_uart();
    ResetReason();

    // 3. Create Queues
    q_data = xQueueCreate(20, sizeof(char));      // Char stream from UART
    q_print = xQueueCreate(10, sizeof(char*));    // Pointers to strings

    configASSERT(q_data != NULL);
    configASSERT(q_print != NULL);

    // Tasks
    xTaskCreate(uart_rx_task,     "uart_rx",   4096, NULL, 10, NULL); 
    xTaskCreate(cmd_handler_task, "cmd_hdlr",  4096, NULL, 9,  &handle_cmd_task);
    xTaskCreate(print_task,       "printer",   4096, NULL, 8,  &handle_print_task); // High Priority
   
    xTaskCreate(menu_task,        "menu",      4096, NULL, 5,  &handle_menu_task);
    xTaskCreate(led_task,         "led_logic", 4096, NULL, 5,  &handle_led_task);
    xTaskCreate(rtc_task,         "rtc_logic", 8192, NULL, 5,  &handle_rtc_task);

    // Timers
    for(int i = 0 ; i < 4 ; i++) {
        handle_led_timer[i] = xTimerCreate("led_timer", pdMS_TO_TICKS(500), pdTRUE, (void*)(i+1), led_effect_callback);    
    }

    // Start
    xTaskNotify(handle_menu_task, 0, eNoAction);
}
