idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES led_strip tinyusb nvs_flash toy_connector)