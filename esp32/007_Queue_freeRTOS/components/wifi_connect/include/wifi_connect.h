#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "my_nvs_storage.h"
#include <netdb.h>

#include "sdkconfig.h"

#define WIFI_TIMEOUT_MS (15000) // 15 seconds default timeout
// Configuration constants
#define MAX_RETRY_ATTEMPTS 5
#define RETRY_DELAY_MS 5000
#define RECONNECT_DELAY_MS 2000

// WiFi state management
typedef struct
{
    esp_netif_t *netif_sta;
    esp_netif_t *netif_ap;
    EventGroupHandle_t wifi_events;
    bool is_connected;
    int retry_count;
    bool auto_reconnect;
} wifi_context_t;

//  @brief Categorizes disconnection reasons and determines if reconnection should be attempted

typedef enum
{
    DISCONNECT_CATEGORY_RECOVERABLE, // Can retry connection
    DISCONNECT_CATEGORY_AUTH_FAILED, // Authentication/password issues
    DISCONNECT_CATEGORY_AP_ISSUES,   // AP-side problems
    DISCONNECT_CATEGORY_FATAL        // Should not retry automatically
} disconnect_category_t;

// Event bits
static const int CONNECTED = BIT0;
static const int DISCONNECTED = BIT1;
static const int FAILED = BIT2;

extern wifi_context_t wifi_ctx; // declaration only
extern esp_netif_t *esp_netif;

// Core functions
void wifi_connect_init(void);
esp_err_t wifi_connect_sta(const char *ssid, const char *pass, int timeout_ms);
void wifi_connect_ap(const char *ssid, const char *pass);
void wifi_disconnect(void);
esp_err_t start_wifi(void);

// Status functions
bool wifi_is_connected(void);
void wifi_enable_auto_reconnect(bool enable);
int wifi_get_retry_count(void);
#endif