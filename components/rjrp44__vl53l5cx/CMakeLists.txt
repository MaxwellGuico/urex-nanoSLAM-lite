cmake_minimum_required(VERSION 3.5)

idf_component_register(SRC_DIRS     src
                       INCLUDE_DIRS "include"
                       REQUIRES     "driver" "esp_driver_i2c" "esp_driver_gpio")
