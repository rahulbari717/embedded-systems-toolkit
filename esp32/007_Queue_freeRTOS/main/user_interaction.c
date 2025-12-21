/**
 * @file user_interaction.c
 * @author Rahul B.
 * @version 1.0
 * @date December 2025
 */

#include "main.h"
#include "esp_log.h"
#include "wifi_connect.h" 
#include "sntp_time.h"    

// Static Messages
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
const char *msg_exit = "Exiting Application...\n";

// =============================================================
//  THE PROTECTED FUNCTION (MUTEX LOGIC)
// =============================================================
void thread_safe_print(const char *msg) {
    // 1. Try to take the token (Wait forever if busy)
    if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
        
        // 2. Critical Section: Access the Shared Resource (UART)
        uart_write_bytes(UART_NUM, msg, strlen(msg));
        
        // 3. Give the token back
        xSemaphoreGive(print_mutex);
    }
}

// =============================================================
//  SYSTEM LOG WRAPPER (Catches WiFi/NTP logs)
// =============================================================
int system_log_wrapper(const char *fmt, va_list args) {
    char buffer[256];
    
    // Format the log message into a string
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    
    // Use our protected function to print it!
    thread_safe_print(buffer);
    
    return 0; 
}

// =============================================================
//  TASK: Print Task (Queue Consumer)
// =============================================================
void print_task(void *param) {
    char *msg;
    while(1) {
        // 1. Dequeue
        if (xQueueReceive(q_print, &msg, portMAX_DELAY) == pdTRUE) {
            
            // 2. Call the Protected Function
            thread_safe_print(msg);
        }
    }
}

// =============================================================
//  TASK: RTC Report
// =============================================================
// void rtc_task(void *param) {
//     while(1) {
//         xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
//         // 1. Install the wrapper so WiFi logs use the Mutex
//         esp_log_set_vprintf(system_log_wrapper);

//         // 2. Start WiFi (Logs will now wait for Mutex)
//         start_wifi(); 
        
//         // 3. Print Time (Logs will wait for Mutex)
//         time_sntp(); 

//         // 4. Return to Menu
//         xTaskNotify(handle_menu_task, 0, eNoAction);
//     }
// }

int extract_command(command_t *cmd) {
    uint8_t item;
    if(uxQueueMessagesWaiting(q_data) == 0) return -1;
    
    uint8_t i = 0;
    do {
        if(xQueueReceive(q_data, &item, 0) == pdTRUE) {
            // ⭐ BUG FIX #4: Skip \r completely
            if(item == '\r') continue;
            
            cmd->payload[i++] = item;
        } else break;
    } while(item != '\n' && i < sizeof(cmd->payload) - 1);

    // Remove newline and null terminate
    if(i > 0 && cmd->payload[i-1] == '\n') {
        cmd->payload[i-1] = '\0';
        cmd->len = i-1;
    } else {
        cmd->payload[i] = '\0';
        cmd->len = i;
    }
    
    // ⭐ BUG FIX #4: Reject empty commands
    if(cmd->len == 0) return -1;
    
    return 0;
}

// --- TASK: Menu ---
void menu_task(void *param) {
    uint32_t cmd_addr;
    command_t *cmd;

    while(1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        curr_state = sMainMenu;

        xQueueSend(q_print, &msg_menu, portMAX_DELAY);

        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

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
                    vTaskDelay(100); abort(); 
                    break;
                default:
                    xQueueSend(q_print, &msg_inv, portMAX_DELAY);
                    xTaskNotify(handle_menu_task, 0, eNoAction);
            }
        }
    }
}

// --- TASK: LED ---
void led_task(void *param) {
    uint32_t cmd_addr;
    command_t *cmd;

    while(1) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        xQueueSend(q_print, &msg_led_menu, portMAX_DELAY);

        xTaskNotifyWait(0, 0, &cmd_addr, portMAX_DELAY);
        cmd = (command_t*)cmd_addr;

        if (strcmp((char*)cmd->payload, "back") == 0) {
            xTaskNotify(handle_menu_task, 0, eNoAction);
        }
        else if (strcmp((char*)cmd->payload, "none") == 0) {
            // led_effect_stop(); // Call functions from led_effect.c
        }
        // ... (Add other LED calls here) ...
        
        if(curr_state == sLedEffect) xTaskNotify(handle_led_task, 0, eNoAction);
    }
}

// --- TASK: Command Handler ---
void cmd_handler_task(void *param) {
    command_t cmd;
    while(1) {
        if(xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE) {
            if(extract_command(&cmd) != -1) {
                switch(curr_state) {
                    case sMainMenu: xTaskNotify(handle_menu_task, (uint32_t)&cmd, eSetValueWithOverwrite); break;
                    case sLedEffect: xTaskNotify(handle_led_task, (uint32_t)&cmd, eSetValueWithOverwrite); break;
                    case sRtcReport: break;
                }
            }
        }
    }
}

// --- TASK: UART RX ---
void uart_rx_task(void *param) {
    uint8_t data;
    while(1) {
        int len = uart_read_bytes(UART_NUM, &data, 1, portMAX_DELAY);
        if(len > 0) {
            if(data == '\r') data = '\n';
            
            // ECHO: We also protect the echo to ensure clean typing!
            thread_safe_print((char*)&data); 
            
            xQueueSend(q_data, &data, 0);
            if(data == '\n') xTaskNotify(handle_cmd_task, 1, eNoAction);
        }
    }
}