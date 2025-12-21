idf_component_register(
    SRCS 
        "wifi_connect.c"
        "wifi_connect_err.c"

    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_netif
        esp_wifi
        log
        nvs_flash
        sntp_time
        esp_http_server 
        my_nvs_storage
        bt
)
