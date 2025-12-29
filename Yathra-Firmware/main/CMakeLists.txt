idf_component_register  ( SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES 
                            driver 
                            i2c
                            util_i2c_scanner
                            sens_vl53l0x
                    )


