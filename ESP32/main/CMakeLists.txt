idf_component_register(SRCS "main.c" "sht40.c" "ssd1306.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_i2c esp_wifi nvs_flash esp_event esp_netif esp_mqtt)
