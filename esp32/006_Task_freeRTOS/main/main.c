#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 1. Updated Prototypes: Added (void *pvParameters)
static void task1(void *pvParameters); 
static void task2(void *pvParameters);

BaseType_t status; 

TaskHandle_t task1_handle;
TaskHandle_t task2_handle;

// 2. Updated Definitions
static void task1(void *pvParameters){
    for(;;){
        printf("temperature reading \n"); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void task2(void *pvParameters){
    for(;;){
        printf("humidity reading \n"); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    // The 4th argument (the string) is what gets passed to pvParameters
    status = xTaskCreate(task1, "temp_task", 2048, "Hello from Task-1", 2, &task1_handle);

    configASSERT(status == pdPASS);

    status = xTaskCreate(task2, "hum_task", 2048, "Hello from Task-2", 2, &task2_handle);

    configASSERT(status == pdPASS);

}