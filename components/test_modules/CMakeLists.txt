idf_component_register(SRCS "kalman_test.cpp" "neo_m8n_test.cpp" "bno085_test.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES bno085 neo_m8n kalman_filter esp_timer esp-dsp)
