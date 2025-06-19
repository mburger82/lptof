idf_component_register(SRCS ./src/i2smanager.c
                        INCLUDE_DIRS .
                        REQUIRES esp_driver_i2s driver audioplayerconfig)