idf_component_register  ( SRCS  "main.cpp" 
                                "src/imu.cpp"
                                "src/telemetry.cpp"
                                "src/baro.cpp"
                                "src/thruster.cpp"
                                "src/control.cpp"
                                "src/mission.cpp"
                        INCLUDE_DIRS "." "src"
                        REQUIRES 
                            driver 
                            util_i2c
                            util_ahrs
                            sens_mpu9250
                            sens_qmc5883l
                            hx711                            
                    )


