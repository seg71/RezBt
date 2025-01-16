idf_component_register(SRCS "toy_connector.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos
                       PRIV_REQUIRES bt)
