// ========================================
// FILE: include/mqtt.h
// ========================================
#ifndef MQTT_H
#define MQTT_H
#include "mqtt_client.h"

// Public function declarations
void mqtt_init(void);
void mqtt_start(void);
int mqtt_send(const char *topic, const char *payload);
void test_send_messages(void *param);

void mqtt_main(void);

#endif // MQTT_H