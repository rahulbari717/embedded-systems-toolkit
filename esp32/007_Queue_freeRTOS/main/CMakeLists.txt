idf_component_register(SRCS 
                        "main.c" 
                        "led_effect.c" 
                        "task_handler.c"
                    INCLUDE_DIRS 
                        "."
                    REQUIRES 
                        # --- Standard Components ---
                        esp_driver_gpio
                        esp_driver_uart
                        nvs_flash
                        esp_timer
                        driver        

                        # --- Your Custom Components ---
                        esp_system_debug 
                        menu_config 
                        my_nvs_storage             
                        my_mqtt
                        wifi_connect 
                        sntp_time
                    )