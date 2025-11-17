idf_component_register(SRCS "colour-sensor.c"
                    PRIV_REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_pcnt
                    INCLUDE_DIRS ".")

