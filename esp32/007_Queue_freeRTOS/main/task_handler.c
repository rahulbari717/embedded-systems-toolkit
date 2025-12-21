/**
 * @file led_effect.c
 * @author Rahul B.
 * @version 1.0
 * @date December 2025
 */

#include "main.h"
#include "esp_log.h"
#include "wifi_connect.h" // Your wifi header
#include "sntp_time.h"    // Your sntp header
#include "esp_system.h"
#include <stdarg.h>   

void thread_safe_print(const char *msg);
int system_log_wrapper(const char *fmt, va_list args);
void time_sntp(void);

// Static strings to avoid stack scope issues in queues
const char *msg_menu = "\n\n=== MAIN MENU ===\n"
                       "1. LED Effect\n"
                       "2. Date & Time Report\n"
                       "3. Exit Application\n"
                       "Enter Selection: ";

const char *msg_led_menu = "\n--- LED MENU ---\n"
                           "Type: e1, e2, e3, e4, none\n"
                           "Type 'back' to return\n"
                           "Selection: ";

const char *msg_inv = "Invalid Command.\n";
const char *msg_wifi_start = "Starting WiFi & SNTP...\n";
const char *msg_exit = "Exiting Application... (System Halted)\n";

static const char *TAG_TASK = "TASK";


// =============================================================
// ⭐ ADD FUNCTION DEFINITIONS HERE
// =============================================================

/**
 * @brief Mutex-protected UART print function
 */
void thread_safe_print(const char *msg) {
    if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
        uart_write_bytes(UART_NUM, msg, strlen(msg));
        xSemaphoreGive(print_mutex);
    }
}

/**
 * @brief System log wrapper that uses mutex
 */
int system_log_wrapper(const char *fmt, va_list args) {
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    thread_safe_print(buffer);
    return 0; 
}

/**
 * @brief Wrapper for SNTP time sync
 */
void time_sntp(void) {
    sntp(); 
}

// --- Helpers ---
int extract_command(command_t *cmd) {
    uint8_t item;
    if(uxQueueMessagesWaiting(q_data) == 0) return -1;
    
    uint8_t i = 0;
    do {
        if(xQueueReceive(q_data, &item, 0) == pdTRUE) {
            cmd->payload[i++] = item;
        } else break;
    } while(item != '\n' && i < sizeof(cmd->payload) - 1);

    cmd->payload[i-1] = '\0'; // Remove newline, null terminate
    cmd->len = i-1;
    return 0;
}

// --- TASKS ---

void menu_task(void *param) {
    uint32_t cmd_addr;
    command_t *cmd;

    while(1) {
        // 1. Wait to be activated
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        curr_state = sMainMenu;

        // 2. Show Menu
        xQueueSend(q_print, &msg_menu, portMAX_DELAY);

        // 3. Wait for Command Handler to send processed input
        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

        // 4. Logic
        if(cmd->len > 0) {
            switch(cmd->payload[0]) {
                case '1':
                    curr_state = sLedEffect;
                    xTaskNotify(handle_led_task, 0, eNoAction);
                    break;
                case '2':
                    curr_state = sRtcReport;
                    xTaskNotify(handle_rtc_task, 0, eNoAction);
                    break;
                case '3':
                    xQueueSend(q_print, &msg_exit, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Let print finish
                    esp_restart(); // Or vTaskSuspendAll();
                    break;
                default:
                    xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                    xTaskNotify(handle_menu_task, 0, eNoAction); // Loop back
            }
        }
    }
}
void led_task(void *param) {
    uint32_t cmd_addr;
    command_t *cmd;

    while(1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        // ⭐ BUG FIX #3: Set state BEFORE printing menu
        curr_state = sLedEffect;
        
        xQueueSend(q_print, &msg_led_menu, portMAX_DELAY);

        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

        if (strcmp((char*)cmd->payload, "back") == 0) {
            // ⭐ BUG FIX #3: Change state immediately
            curr_state = sMainMenu;
            xTaskNotify(handle_menu_task, 0, eNoAction);
        }
        else if (strcmp((char*)cmd->payload, "none") == 0) {
            led_effect_stop();
            xTaskNotify(handle_led_task, 0, eNoAction);  // Stay in LED menu
        }
        else if (strcmp((char*)cmd->payload, "e1") == 0) {
            led_effect(1);
            xTaskNotify(handle_led_task, 0, eNoAction);
        }
        else if (strcmp((char*)cmd->payload, "e2") == 0) {
            led_effect(2);
            xTaskNotify(handle_led_task, 0, eNoAction);
        }
        else if (strcmp((char*)cmd->payload, "e3") == 0) {
            led_effect(3);
            xTaskNotify(handle_led_task, 0, eNoAction);
        }
        else if (strcmp((char*)cmd->payload, "e4") == 0) {
            led_effect(4);
            xTaskNotify(handle_led_task, 0, eNoAction);
        }
        else {
            xQueueSend(q_print, &msg_inv, portMAX_DELAY);
            xTaskNotify(handle_led_task, 0, eNoAction);  // Stay in LED menu
        }
    }
}

// =============================================================
// TASK: RTC Report
// =============================================================
void rtc_task(void *param) {
    while(1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        // ⭐ MOVE LOG SUPPRESSION HERE (before WiFi starts)
        esp_log_level_set("wifi", ESP_LOG_ERROR);
        esp_log_level_set("wifi_init", ESP_LOG_ERROR);
        esp_log_level_set("phy_init", ESP_LOG_ERROR);
        esp_log_level_set("WIFI CONNECT", ESP_LOG_WARN);
        
        // Install the wrapper so WiFi logs use the Mutex
        esp_log_set_vprintf(system_log_wrapper);

        // Start WiFi (Logs will now be suppressed)
        start_wifi(); 
        time_sntp();

        const char *msg_done = "Report Done. Returning to menu...\n";
        xQueueSend(q_print, &msg_done, portMAX_DELAY);

        xTaskNotify(handle_menu_task, 0, eNoAction);
    }
}

void cmd_handler_task(void *param) {
    command_t cmd;
    while(1) {
        if(xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE) {
            // ⭐ BUG FIX #4: Check both return value AND length
            if(extract_command(&cmd) == 0 && cmd.len > 0) {
                switch(curr_state) {
                    case sMainMenu: xTaskNotify(handle_menu_task, (uint32_t)&cmd, eSetValueWithOverwrite); break;
                    case sLedEffect: xTaskNotify(handle_led_task, (uint32_t)&cmd, eSetValueWithOverwrite); break;
                    case sRtcReport: break;
                }
            }
        }
    }
}

// =============================================================
// TASK: Print Task
// =============================================================
void print_task(void *param) {
    char *msg;
    while(1) {
        if (xQueueReceive(q_print, &msg, portMAX_DELAY) == pdTRUE) {
            thread_safe_print(msg);
        }
    }
}

void uart_rx_task(void *param) {
    uint8_t data;
    while(1) {
        int len = uart_read_bytes(UART_NUM, &data, 1, portMAX_DELAY);
        if(len > 0) {
            // ⭐ BUG FIX #4: Convert \r to \n
            if(data == '\r') data = '\n';
            
            // ⭐ BUG FIX #1: Echo using mutex (create string properly)
            char echo[2] = {data, '\0'};
            thread_safe_print(echo);
            
            xQueueSend(q_data, &data, 0);
            if(data == '\n') xTaskNotify(handle_cmd_task, 1, eNoAction);
        }
    }
}

