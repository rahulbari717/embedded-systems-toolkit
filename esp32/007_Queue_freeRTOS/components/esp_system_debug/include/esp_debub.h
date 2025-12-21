#ifndef ESP_DEBUB_H
#define ESP_DEBUB_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void FirmwareShutdownHandler(void);
void ResetReason(void);

#endif // ESP_DEBUB_H