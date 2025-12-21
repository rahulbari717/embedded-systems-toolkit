/**
 * @file led_effect.c
 * @author Rahul B.
 * @version 1.0
 * @date December 2025
 */

#include "main.h"

// --- Hardware Init ---
void init_gpio(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Set bits for Pins 2, 4, 5, 18
    io_conf.pin_bit_mask = (1ULL<<LED1_PIN) | (1ULL<<LED2_PIN) | (1ULL<<LED3_PIN) | (1ULL<<LED4_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
}

void set_led(int pin, int level) {
    gpio_set_level(pin, level);
}

// --- Effects Logic ---
void turn_off_all_leds(void) {
    set_led(LED1_PIN, 0); set_led(LED2_PIN, 0);
    set_led(LED3_PIN, 0); set_led(LED4_PIN, 0);
}

void turn_on_all_leds(void) {
    set_led(LED1_PIN, 1); set_led(LED2_PIN, 1);
    set_led(LED3_PIN, 1); set_led(LED4_PIN, 1);
}

void turn_on_odd_leds(void) {
    set_led(LED1_PIN, 1); set_led(LED2_PIN, 0);
    set_led(LED3_PIN, 1); set_led(LED4_PIN, 0);
}

void turn_on_even_leds(void) {
    set_led(LED1_PIN, 0); set_led(LED2_PIN, 1);
    set_led(LED3_PIN, 0); set_led(LED4_PIN, 1);
}

int get_pin_from_index(int index) {
    switch(index) {
        case 0: return LED1_PIN;
        case 1: return LED2_PIN;
        case 2: return LED3_PIN;
        case 3: return LED4_PIN;
        default: return LED1_PIN;
    }
}

void LED_control(int value) {
    for(int i = 0 ; i < 4 ; i++) {
        set_led(get_pin_from_index(i), (value >> i) & 0x1);
    }
}

void LED_effect1(void) {
    static int flag = 1;
    (flag ^= 1) ? turn_off_all_leds() : turn_on_all_leds();
}

void LED_effect2(void) {
    static int flag = 1;
    (flag ^= 1) ? turn_on_even_leds() : turn_on_odd_leds();
}

void LED_effect3(void) {
    static int i = 0;
    LED_control(0x1 << (i++ % 4));
}

void LED_effect4(void) {
    static int i = 0;
    LED_control(0x08 >> (i++ % 4));
}

void led_effect_stop(void) {
    for(int i = 0 ; i < 4 ; i++)
        xTimerStop(handle_led_timer[i], portMAX_DELAY);
    turn_off_all_leds();
}

void led_effect(int n) {
    led_effect_stop();
    xTimerStart(handle_led_timer[n-1], portMAX_DELAY);
}

// This callback is used by the timers created in main.c
void led_effect_callback(TimerHandle_t xTimer) {
     int id = (uint32_t)pvTimerGetTimerID(xTimer);
     switch(id) {
         case 1: LED_effect1(); break;
         case 2: LED_effect2(); break;
         case 3: LED_effect3(); break;
         case 4: LED_effect4(); break;
     }
}