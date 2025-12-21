#ifndef __MAIN_H
#define __MAIN_H

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h" 

#include "driver/gpio.h"
#include "driver/uart.h"
#include <stdarg.h>

// Define your LED Pins here (Change these to match your board)
#define LED1_PIN    GPIO_NUM_2
#define LED2_PIN    GPIO_NUM_4
#define LED3_PIN    GPIO_NUM_15
#define LED4_PIN    GPIO_NUM_27

#define UART_NUM    UART_NUM_0
#define BUF_SIZE    1024

// --- Data Structures ---
typedef struct {
    uint8_t payload[20]; // Increased size for commands
    uint32_t len;
} command_t;

typedef enum {
    sMainMenu = 0,
    sLedEffect,
    sRtcReport,
} state_t;

// Global Handles
extern TaskHandle_t handle_cmd_task;
extern TaskHandle_t handle_menu_task;
extern TaskHandle_t handle_print_task;
extern TaskHandle_t handle_led_task;
extern TaskHandle_t handle_rtc_task;

extern QueueHandle_t q_data;
extern QueueHandle_t q_print;
extern SemaphoreHandle_t print_mutex;

extern state_t curr_state;
extern TimerHandle_t handle_led_timer[4];

void thread_safe_print(const char *msg);
int system_log_wrapper(const char *fmt, va_list args);

// --- Function Prototypes ---

// Hardware / Effects
void init_gpio(void);
void init_uart(void);
void led_effect_stop(void);
void led_effect(int n);
void LED_effect1(void);
void LED_effect2(void);
void LED_effect3(void);
void LED_effect4(void);
void led_effect_callback(TimerHandle_t xTimer);

// Tasks
void menu_task(void *param);
void led_task(void *param);
void rtc_task(void *param);
void print_task(void *param);
void cmd_handler_task(void *param);
void uart_rx_task(void *param);

#endif /* __MAIN_H */