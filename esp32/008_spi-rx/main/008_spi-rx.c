/*
 * SPI Slave RX Only for ESP32 (ESP-IDF 5.5)
 * Receives data from STM32 Master
 * 
 * PIN CONFIGURATION:
 * ==================
 * STM32 (Master)          ESP32 Pico (Slave)
 * ------------------------------------------
 * PB12 (NSS/CS)    --->   GPIO5  (CS)
 * PB13 (SCLK)      --->   GPIO18 (SCLK)
 * PB15 (MOSI)      --->   GPIO23 (MOSI)
 * GND              --->   GND
 * 
 * SPI SETTINGS (Must Match):
 * ==========================
 * Mode: 0 (CPOL=0, CPHA=0)
 * Clock: 2 MHz
 * Bit Order: MSB First
 * Data Frame: 8-bit
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

// SPI Pin Definitions
// Option 1: Using VSPI pins (recommended - try this first)
#define GPIO_CS      5   // VSPICS0 - Right side
#define GPIO_SCLK    19  // VSPIQ - Right side
#define GPIO_MOSI    21  // VSPIHD - Right side
#define GPIO_MISO    22  // VSPIWP - Not connected but required

// Buffer size
#define RX_BUFFER_SIZE 128

static const char *TAG = "SPI_SLAVE";

// Reception buffer
DMA_ATTR uint8_t rx_buffer[RX_BUFFER_SIZE];
DMA_ATTR uint8_t tx_buffer[RX_BUFFER_SIZE];  // Dummy TX buffer

esp_err_t ret;

void spi_init(void){
    ESP_LOGI(TAG, "Initializing SPI Slave...");
    
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = RX_BUFFER_SIZE,
    };
    
    // Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,              // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };
    
    // Enable pull-ups on SPI lines
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);
    
    // Initialize SPI slave interface
    ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "SPI Slave initialized successfully");
}

void app_main(void)
{
    spi_init(); 
    ESP_LOGI(TAG, "Waiting for data from STM32 Master...");
    
    // Main reception loop
    while (1) {
        // Clear buffers
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        memset(tx_buffer, 0, RX_BUFFER_SIZE);
        
        // Set up a transaction
        spi_slave_transaction_t t = {
            .length = RX_BUFFER_SIZE * 8,  // Length in bits
            .trans_len = 0,
            .tx_buffer = tx_buffer,
            .rx_buffer = rx_buffer
        };
        
        // Wait for transaction from master
        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            // Calculate actual received bytes
            size_t received_bytes = t.trans_len / 8;
            
            if (received_bytes > 0) {
                ESP_LOGI(TAG, "=== Data Received ===");
                ESP_LOGI(TAG, "Length byte: %d", rx_buffer[0]);
                
                // First byte is length, rest is actual data
                uint8_t data_length = rx_buffer[0];
                
                if (data_length > 0 && data_length < RX_BUFFER_SIZE) {
                    // Null terminate the string
                    rx_buffer[data_length + 1] = '\0';
                    
                    // Print received data (skip the length byte)
                    ESP_LOGI(TAG, "Data: %s", (char*)&rx_buffer[1]);
                    ESP_LOGI(TAG, "Total bytes received: %d", received_bytes);
                } else {
                    // Print raw data
                    ESP_LOGI(TAG, "Raw data (%d bytes):", received_bytes);
                    ESP_LOG_BUFFER_HEX(TAG, rx_buffer, received_bytes);
                }
                
                ESP_LOGI(TAG, "====================\n");
            }
        } else {
            ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        }
        
        // Small delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}