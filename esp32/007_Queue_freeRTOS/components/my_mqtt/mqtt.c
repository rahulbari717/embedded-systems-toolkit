// ========================================
// FILE: mqtt.c
// ========================================
#include <stdio.h>
#include "mqtt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static char *TAG = "MQTT";
static esp_mqtt_client_handle_t client;

// Internal function prototypes
static void handle_mqtt_connected(void);
static void handle_mqtt_disconnected(void);
static void handle_mqtt_subscribed(void);
static void handle_mqtt_unsubscribed(void);
static void handle_mqtt_published(void);
static void handle_mqtt_data(esp_mqtt_event_handle_t event);
static void handle_mqtt_error(esp_mqtt_event_handle_t event);

static void mqtt_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        handle_mqtt_connected();
        break;
    case MQTT_EVENT_DISCONNECTED:
        handle_mqtt_disconnected();
        break;
    case MQTT_EVENT_SUBSCRIBED:
        handle_mqtt_subscribed();
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        handle_mqtt_unsubscribed();
        break;
    case MQTT_EVENT_PUBLISHED:
        handle_mqtt_published();
        break;
    case MQTT_EVENT_DATA:
        handle_mqtt_data(event);
        break;
    case MQTT_EVENT_ERROR:
        handle_mqtt_error(event);
        break;
    default:
        ESP_LOGW(TAG, "Unhandled MQTT event ID: %d", event_id);
        break;
    }
}

// Handler functions for each MQTT event
static void handle_mqtt_connected(void)
{
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    esp_mqtt_client_subscribe(client, "animal/mammal/cat/felix", 1);
    esp_mqtt_client_subscribe(client, "animal/reptiles/+/slither", 1);
    esp_mqtt_client_subscribe(client, "animal/fish/#", 1);
    esp_mqtt_client_subscribe(client, "home/rahul/#", 1);
}

static void handle_mqtt_disconnected(void)
{
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
}

static void handle_mqtt_subscribed(void)
{
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
}

static void handle_mqtt_unsubscribed(void)
{
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
}

static void handle_mqtt_published(void)
{
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
}

static void handle_mqtt_data(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("topic: %.*s\n", event->topic_len, event->topic);
    printf("message: %.*s\n", event->data_len, event->data);
}

static void handle_mqtt_error(esp_mqtt_event_handle_t event)
{
    ESP_LOGE(TAG, "ERROR %s", strerror(event->error_handle->esp_transport_sock_errno));
}

// Public functions
void mqtt_init(void)
{
    esp_mqtt_client_config_t esp_mqtt_client_config = {
        .broker.address.uri = "mqtt://test.mosquitto.org:1883"};
    client = esp_mqtt_client_init(&esp_mqtt_client_config);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
}

void mqtt_start(void)
{
    esp_mqtt_client_start(client);
}

int mqtt_send(const char *topic, const char *payload)
{
    return esp_mqtt_client_publish(client, topic, payload, strlen(payload), 1, 0);
}

void test_send_messages(void *param)
{
    int count = 0;
    char message[50];
    while (true)
    {
        sprintf(message, "hello rahul ... %d", count++);
        ESP_LOGW(TAG, "hello rahul ... %d", count);
        mqtt_send("animal/message/from/esp32", message);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void mqtt_main(void)
{
    ESP_LOGI(TAG, "MQTT main started");
    //  Initialize and start MQTT
    mqtt_init();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds to ensure WiFi is connected
    mqtt_start();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds to ensure WiFi is connected

    // Create task for sending messages
    xTaskCreate(test_send_messages, "test send messages", 1024 * 2, NULL, 5, NULL);
}