#ifndef MY_NVS_STORAGE_H
#define MY_NVS_STORAGE_H

#include "esp_err.h"
#include <stddef.h>
#include "sdkconfig.h" // Needed for CONFIG_WIFI_SSID etc.

#ifdef __cplusplus
extern "C"
{
#endif

    // Function prototypes for NVS operations.
    esp_err_t nvs_init_storage(void);
    esp_err_t nvs_save_wifi_credentials(const char *ssid, const char *password);
    esp_err_t nvs_get_wifi_credentials(char *ssid, size_t *ssid_size, char *password, size_t *password_size);
    esp_err_t nvs_save_calibrated_value(float value);
    esp_err_t nvs_get_calibrated_value(float *value);
    void nvs_read_and_print_all_values(void);
    void initialize_and_use_nvs(void);

#ifdef __cplusplus
}
#endif

#endif // MY_NVS_STORAGE_H
