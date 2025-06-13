idf_component_register(SRCS 
                            # ./src/tmf8828_app.cpp
                            ./src/tmf882x_calib.c
                            ./src/tmf882x_image.c
                            ./src/tmf8828_calib.c
                            ./src/tmf8828_image.c
                            ./src/tmf8828_shim.c
                            ./src/tmf8828.c
                            ./src/tmf8828_app.c
                            
                        INCLUDE_DIRS .
                        REQUIRES gpi2c)