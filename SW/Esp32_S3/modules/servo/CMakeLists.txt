idf_component_register(
    SRCS "servo.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc
    PRIV_REQUIRES config
)