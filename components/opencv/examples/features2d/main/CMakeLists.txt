idf_component_register(SRCS "main.c" "cv_opencv_features2d.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES opencv  esp32-camera esp_timer)
