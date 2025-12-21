/**
 * @file nvs.c
 * @brief Functions for handling Non-Volatile Storage (NVS) operations.
 * @author Rahul B.
 * @version 1.0
 * @date August 2025
 */

#include "my_nvs_storage.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdlib.h>    // For atof()
#include "sdkconfig.h" // For CONFIG_WIFI_SSID etc.

static const char *TAG = "NVS_STORAGE";
/**
 * @brief Initializes the NVS flash partition.
 * * If the partition is corrupted or has an old version, this function
 * will automatically erase and re-initialize it. This is a common
 * practice for robust applications.
 * * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t nvs_init_storage(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition seems invalid. Erasing and re-initializing.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    return ret;
}

/**
 * @brief Saves Wi-Fi credentials to NVS in the "wifi_creds" namespace.
 * * @param ssid The Wi-Fi SSID to save.
 * @param password The Wi-Fi password to save.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t nvs_save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("wifi_creds", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, "ssid", ssid);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(err));
    }

    err = nvs_set_str(handle, "pass", password);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Wi-Fi credentials saved successfully.");
    return err;
}

/**
 * @brief Retrieves Wi-Fi credentials from NVS.
 * * @param ssid Pointer to a buffer where the SSID will be copied.
 * @param ssid_size Pointer to the size of the SSID buffer.
 * @param password Pointer to a buffer where the password will be copied.
 * @param password_size Pointer to the size of the password buffer.
 * @return ESP_OK on success, or an error code if not found or on error.
 */
esp_err_t nvs_get_wifi_credentials(char *ssid, size_t *ssid_size, char *password, size_t *password_size)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("wifi_creds", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_str(handle, "ssid", ssid, ssid_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SSID: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_get_str(handle, "pass", password, password_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get password: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Wi-Fi credentials retrieved.");
    return ESP_OK;
}

/**
 * @brief Saves a calibrated float value to NVS in the "sensor_data" namespace.
 * * @param value The float value to save.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t nvs_save_calibrated_value(float value)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("sensor_data", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // Convert the float to a string for storage.
    char value_str[20];
    snprintf(value_str, sizeof(value_str), "%.2f", value);

    err = nvs_set_str(handle, "cal_val", value_str);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save calibrated value: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Calibrated value saved.");
    return err;
}

/**
 * @brief Retrieves a calibrated float value from NVS.
 * * @param value Pointer to a float variable where the retrieved value will be stored.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t nvs_get_calibrated_value(float *value)
{
    nvs_handle_t handle;
    esp_err_t err;
    char value_str[20];
    size_t required_size = sizeof(value_str);

    err = nvs_open("sensor_data", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_str(handle, "cal_val", value_str, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get calibrated value: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    *value = atof(value_str);
    nvs_close(handle);
    ESP_LOGI(TAG, "Calibrated value retrieved.");
    return ESP_OK;
}

/**
 * @brief Reads and prints all stored NVS values for verification.
 * * This function opens the relevant NVS namespaces and retrieves
 * both the Wi-Fi credentials and the calibrated sensor value,
 * printing them to the console.
 */
void nvs_read_and_print_all_values(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Read and print WiFi credentials
    err = nvs_open("wifi_creds", NVS_READONLY, &handle);
    if (err == ESP_OK)
    {
        char ssid[32] = {0};
        char password[64] = {0};
        size_t ssid_size = sizeof(ssid);
        size_t password_size = sizeof(password);

        if (nvs_get_str(handle, "ssid", ssid, &ssid_size) == ESP_OK)
        {
            ESP_LOGI(TAG, "Read from NVS -> SSID: %s", ssid);
        }
        else
        {
            ESP_LOGW(TAG, "SSID not found in NVS.");
        }

        if (nvs_get_str(handle, "pass", password, &password_size) == ESP_OK)
        {
            ESP_LOGI(TAG, "Read from NVS -> Password: %s", password);
        }
        else
        {
            ESP_LOGW(TAG, "Password not found in NVS.");
        }
        nvs_close(handle);
    }
    else
    {
        ESP_LOGE(TAG, "Error opening 'wifi_creds' namespace: %s", esp_err_to_name(err));
    }

    // Read and print calibrated value
    err = nvs_open("sensor_data", NVS_READONLY, &handle);
    if (err == ESP_OK)
    {
        char cal_val_str[20] = {0};
        size_t required_size = sizeof(cal_val_str);

        if (nvs_get_str(handle, "cal_val", cal_val_str, &required_size) == ESP_OK)
        {
            ESP_LOGI(TAG, "Read from NVS -> Calibrated Value: %s", cal_val_str);
        }
        else
        {
            ESP_LOGW(TAG, "Calibrated value not found in NVS.");
        }
        nvs_close(handle);
    }
    else
    {
        ESP_LOGE(TAG, "Error opening 'sensor_data' namespace: %s", esp_err_to_name(err));
    }
}

/**
 * @brief This function encapsulates all NVS-related logic.
 *
 * It initializes the NVS flash, saves the Wi-Fi credentials from Kconfig,
 * saves a calibrated sensor value, and then retrieves and prints all the
 * stored data to verify the operations. This keeps app_main() clean.
 */
void initialize_and_use_nvs(void)
{
    // Initialize NVS storage at the beginning of your application.
    // This must be done before any NVS read/write operations.
    ESP_ERROR_CHECK(nvs_init_storage());

    // Save the credentials and a calibrated value to NVS.
    // The credentials come from the Kconfig menu.
    ESP_LOGI(TAG, "Saving data to NVS from Kconfig and local variable...");
    ESP_ERROR_CHECK(nvs_save_wifi_credentials(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD));

    // Example of saving a float value.
    float calibrated_temp = 23.45;
    ESP_ERROR_CHECK(nvs_save_calibrated_value(calibrated_temp));

    // Read and print all values from NVS to verify they were saved correctly.
    ESP_LOGI(TAG, "Verifying and printing all data from NVS...");
    nvs_read_and_print_all_values();
}
