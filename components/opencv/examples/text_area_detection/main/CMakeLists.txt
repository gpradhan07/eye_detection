idf_component_register(SRCS "main.c" "cv_text_detection.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES opencv  esp32-camera esp_timer)
