idf_component_register(SRCS "micMain.c" "stateMachine.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_adc esp_driver_gpio)