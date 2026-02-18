idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES config ble H-Bridge_TB6612FNG servo
)
