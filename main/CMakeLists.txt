idf_component_register(
    SRCS 
        esp32S3_knet_demo.cpp
        app_camera.c
        ./face_detection/facedetectcnn-data.cpp
        ./face_detection/facedetectcnn-model.cpp
        ./face_detection/facedetectcnn.cpp
        dms_spi_api.c
        dms_timer_api.c
        wifi_connection1.c
    INCLUDE_DIRS 
        .
        include
        face_detection
    
    REQUIRES opencv esp32-camera esp_timer nvs_flash esp_psram esp_wifi  esp_http_client
    )
