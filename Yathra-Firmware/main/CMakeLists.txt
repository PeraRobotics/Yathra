idf_component_register  ( SRCS  "main.cpp" 
                                "src/imu.cpp"
                                "src/telemetry.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES 
                            driver 
                            util_i2c_scanner
                            util_ahrs
                            sens_mpu9250
                            sens_qmc5883l
                            hx711                            
                    )


