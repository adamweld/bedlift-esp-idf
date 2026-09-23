idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES cybergear canbus sensors state buttons motion safety ui
                             LovyanGFX esp_driver_gpio esp_timer)
