/**
 * @file wifi_connect.c
 * @brief Handles Wi-Fi connections with STA, AP Captive Portal, and Bluetooth provisioning.
 * @author Rahul
 * @date 1st Sept 2025
 */
#include "wifi_connect.h"
#include "sntp_time.h"
#include "esp_http_server.h"
#include "esp_mac.h"

#define SSID (CONFIG_WIFI_SSID)
#define PASSWORD (CONFIG_WIFI_PASSWORD)

static char *TAG = "WIFI CONNECT";
wifi_context_t wifi_ctx = {0}; // actual definition
esp_netif_t *esp_netif = NULL;

char *get_wifi_disconnection_string(wifi_err_reason_t wifi_err_reason);
int disconnection_err_count = 0;

static disconnect_category_t categorize_disconnect_reason(wifi_err_reason_t reason)
{
    switch (reason)
    {
    // Recoverable - network temporary issues
    case WIFI_REASON_BEACON_TIMEOUT:
    case WIFI_REASON_NO_AP_FOUND:
    case WIFI_REASON_ASSOC_TOOMANY:
        return DISCONNECT_CATEGORY_RECOVERABLE;

    // Authentication/Security issues - likely wrong credentials
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
    case WIFI_REASON_MIC_FAILURE:
    case WIFI_REASON_802_1X_AUTH_FAILED:
        return DISCONNECT_CATEGORY_AUTH_FAILED;

    // AP-side issues
    case WIFI_REASON_ASSOC_EXPIRE:
    case WIFI_REASON_ASSOC_FAIL:
    case WIFI_REASON_NOT_AUTHED:
    case WIFI_REASON_NOT_ASSOCED:
    case WIFI_REASON_AP_INITIATED:
        return DISCONNECT_CATEGORY_AP_ISSUES;

    // Fatal - should not retry
    case WIFI_REASON_CIPHER_SUITE_REJECTED:
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
    case WIFI_REASON_INVALID_RSN_IE_CAP:
        return DISCONNECT_CATEGORY_FATAL;

    // Default to recoverable for unknown reasons
    default:
        return DISCONNECT_CATEGORY_RECOVERABLE;
    }
}

/**
 * @brief Enhanced event handler with proper disconnection categorization
 */
void event_handler(void *event_handler_arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi AP started");
            wifi_ctx.is_connected = true; // AP is active
            break;

        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "WiFi AP stopped");
            wifi_ctx.is_connected = false;
            break;

        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGW(TAG, "Station " MACSTR " left, AID=%d, reason=%d",
                     MAC2STR(event->mac), event->aid, event->reason);
            break;
        }

        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi station started");
            wifi_ctx.retry_count = 0;
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
        {
            wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
            ESP_LOGI(TAG, "Connected to AP: SSID=%s, Channel=%d",
                     event->ssid, event->channel);
            wifi_ctx.retry_count = 0; // Reset retry count on successful connection
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            const char *reason_str = get_wifi_disconnection_string(event->reason);
            disconnect_category_t category = categorize_disconnect_reason(event->reason);

            // Enhanced logging with categorization
            switch (category)
            {
            case DISCONNECT_CATEGORY_RECOVERABLE:
                ESP_LOGW(TAG, "WiFi disconnected (RECOVERABLE): %s (%d) - Will retry",
                         reason_str, event->reason);
                break;
            case DISCONNECT_CATEGORY_AUTH_FAILED:
                ESP_LOGE(TAG, "WiFi disconnected (AUTH_FAILED): %s (%d) - Check credentials!",
                         reason_str, event->reason);
                break;
            case DISCONNECT_CATEGORY_AP_ISSUES:
                ESP_LOGW(TAG, "WiFi disconnected (AP_ISSUES): %s (%d) - AP-side problem",
                         reason_str, event->reason);
                break;
            case DISCONNECT_CATEGORY_FATAL:
                ESP_LOGE(TAG, "WiFi disconnected (FATAL): %s (%d) - Configuration error!",
                         reason_str, event->reason);
                break;
            }

            wifi_ctx.is_connected = false;

            // Reconnection logic based on category and retry count
            if (wifi_ctx.auto_reconnect && wifi_ctx.retry_count < MAX_RETRY_ATTEMPTS)
            {
                bool should_retry = false;
                int delay_ms = RECONNECT_DELAY_MS;

                switch (category)
                {
                case DISCONNECT_CATEGORY_RECOVERABLE:
                    should_retry = true;
                    delay_ms = RECONNECT_DELAY_MS;
                    break;
                case DISCONNECT_CATEGORY_AP_ISSUES:
                    should_retry = true;
                    delay_ms = RETRY_DELAY_MS; // Longer delay for AP issues
                    break;
                case DISCONNECT_CATEGORY_AUTH_FAILED:
                    if (wifi_ctx.retry_count < 2)
                    { // Only retry auth issues twice
                        should_retry = true;
                        delay_ms = RETRY_DELAY_MS;
                    }
                    break;
                case DISCONNECT_CATEGORY_FATAL:
                    should_retry = false;
                    ESP_LOGE(TAG, "Fatal WiFi error - automatic reconnection disabled");
                    break;
                }

                if (should_retry)
                {
                    wifi_ctx.retry_count++;
                    ESP_LOGI(TAG, "Scheduling reconnection attempt %d/%d in %d ms",
                             wifi_ctx.retry_count, MAX_RETRY_ATTEMPTS, delay_ms);

                    // Use a task delay instead of blocking here
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    esp_wifi_connect();
                    return; // Don't set DISCONNECTED bit yet
                }
            }

            if (wifi_ctx.retry_count >= MAX_RETRY_ATTEMPTS)
            {
                ESP_LOGE(TAG, "Max retry attempts (%d) reached - giving up", MAX_RETRY_ATTEMPTS);
                wifi_ctx.auto_reconnect = false;
            }

            xEventGroupSetBits(wifi_ctx.wifi_events, DISCONNECTED);
            break;
        }

        default:
            ESP_LOGI(TAG, "Unhandled WiFi event: %ld", event_id);
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));

        wifi_ctx.is_connected = true;
        wifi_ctx.retry_count = 0; // Reset retry count on successful IP assignment

        xEventGroupSetBits(wifi_ctx.wifi_events, CONNECTED);

        // Start SNTP time synchronization
        // sntp();
    }
}

void wifi_connect_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t ret = esp_event_loop_create_default();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Event loop already created");
    } else {
        ESP_ERROR_CHECK(ret);
    }

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_LOGI(TAG, "WiFi initialization completed");
}

esp_err_t wifi_connect_sta(const char *ssid, const char *pass, int timeout)
{

    if (!ssid || !pass)
    {
        ESP_LOGE(TAG, "Invalid SSID or password");
        return ESP_ERR_INVALID_ARG;
    }

    // Clean up any existing connection
    if (wifi_ctx.netif_sta)
    {
        esp_netif_destroy(wifi_ctx.netif_sta);
        wifi_ctx.netif_sta = NULL;
    }

    if (wifi_ctx.wifi_events)
    {
        vEventGroupDelete(wifi_ctx.wifi_events);
    }

    // Create event group and network interface
    wifi_ctx.wifi_events = xEventGroupCreate();
    if (!wifi_ctx.wifi_events)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    wifi_ctx.netif_sta = esp_netif_create_default_wifi_sta();
    if (!wifi_ctx.netif_sta)
    {
        ESP_LOGE(TAG, "Failed to create network interface");
        vEventGroupDelete(wifi_ctx.wifi_events);
        return ESP_ERR_NO_MEM;
    }

    // Set WiFi mode and configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));

    // Enable automatic reconnection
    wifi_ctx.auto_reconnect = true;
    wifi_ctx.retry_count = 0;
    wifi_ctx.is_connected = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP_LOGI(TAG, "Connecting to WiFi SSID: %s (timeout: %d ms)", ssid, timeout_ms);
    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s (timeout: %d ms)", ssid, timeout);

    // Wait for connection result
    EventBits_t bits = xEventGroupWaitBits(
        wifi_ctx.wifi_events,
        CONNECTED | DISCONNECTED,
        pdTRUE,  // Clear bits on exit
        pdFALSE, // Wait for any bit
        pdMS_TO_TICKS(timeout));

    if (bits & CONNECTED)
    {
        ESP_LOGI(TAG, "WiFi connection successful");
        return ESP_OK;
    }
    else if (bits & DISCONNECTED)
    {
        ESP_LOGE(TAG, "WiFi connection failed - disconnected");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGE(TAG, "WiFi connection timeout after %d ms", timeout);

        return ESP_ERR_TIMEOUT;
    }
}

void wifi_connect_ap(const char *ssid, const char *pass)
{

    wifi_connect_init();
    esp_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password) - 1);

    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.beacon_interval = 100;
    wifi_config.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting WiFi...");

    wifi_ctx.auto_reconnect = false;
    wifi_ctx.is_connected = false;

    esp_wifi_stop();

    if (wifi_ctx.netif_sta)
    {
        esp_netif_destroy(wifi_ctx.netif_sta);
        wifi_ctx.netif_sta = NULL;
    }

    if (wifi_ctx.netif_ap)
    {
        esp_netif_destroy(wifi_ctx.netif_ap);
        wifi_ctx.netif_ap = NULL;
    }

    if (wifi_ctx.wifi_events)
    {
        vEventGroupDelete(wifi_ctx.wifi_events);
        wifi_ctx.wifi_events = NULL;
    }
}

esp_err_t start_wifi(void)
{
    ESP_LOGI(TAG, "Starting WiFi connection...");

    wifi_connect_init();

    esp_err_t ret = wifi_connect_sta(SSID, PASSWORD, WIFI_TIMEOUT_MS);
    // esp_err_t ret = wifi_connect_sta("Rahul", "rahul8459", WIFI_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi connection failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi started successfully");
    return ESP_OK;
}

bool wifi_is_connected(void)
{
    return wifi_ctx.is_connected;
}

void wifi_enable_auto_reconnect(bool enable)
{
    wifi_ctx.auto_reconnect = enable;
    ESP_LOGI(TAG, "Auto-reconnect %s", enable ? "enabled" : "disabled");
}

int wifi_get_retry_count()
{
    return wifi_ctx.retry_count;
}