#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>      
#undef EPS
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#define EPS 192
#include "app_camera.h"
#include "esp_timer.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
//#include <face1.hpp>
#include "nvs_flash.h"
#include "utils.h"  
// face detection 
#include <facedetectcnn.h>

#define BUZZER_PIN GPIO_NUM_15
#define LED_PIN GPIO_NUM_16


// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define DETECT_BUFFER_SIZE 0x20000

using namespace cv;
static const char *TAG = "knet";

size_t _jpg_buf_len;
uint8_t * _jpg_buf;

extern "C" {
void app_main(void);
void init_spi(void);
esp_err_t spi_send_data_chunks(uint8_t *data, int len);
void dms_create_timer(void);
void dms_start_gpio_timer_led(void);
void dms_start_gpio_timer_buzzer(void);
void wifi_init_sta(void);
}

float chebyshev_distance(float x1, float y1, float x2, float y2) {
    return std::max(std::abs(x1 - x2), std::abs(y1 - y2));
}

std::tuple<float, float, float> line_coefficients(float x1, float y1, float x2, float y2) {
    float A = y2 - y1;
    float B = x1 - x2;
    float C = x2 * y1 - x1 * y2;
    return std::make_tuple(A, B, C);
}

float distance_between_points(float x1, float y1, float x2, float y2) {
    float distance = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return distance;
}

float perpendicular_distance(float x0, float y0, float x1, float y1, float x2, float y2) {
    auto [A, B, C] = line_coefficients(x1, y1, x2, y2);
    float numerator = std::abs(A * x0 + B * y0 + C);
    float denominator = std::sqrt(A * A + B * B);
    float distance = numerator / denominator;
    return distance;
}

bool distraction_state(int faceX, int faceY, int faceW, int faceH, int x_eye_left, int y_eye_left, int x_eye_right, int y_eye_right,int x_nose, int y_nose,int x_mouth_left,int y_mouth_left, int x_mouth_right, int y_mouth_right){
    float eye_distance = distance_between_points(x_eye_left, y_eye_left, x_eye_right, y_eye_right);

    // Case 1
    float d_eye_right = chebyshev_distance(x_nose, y_nose, x_eye_left, y_eye_left);
    float d_eye_left = chebyshev_distance(x_nose, y_nose, x_eye_right, y_eye_right);
    float d_mouth_left = chebyshev_distance(x_nose, y_nose, x_mouth_left, y_mouth_left);
    float d_mouth_right = chebyshev_distance(x_nose, y_nose, x_mouth_right, y_mouth_right);

    float dist_index = d_eye_left * (0.3) + d_eye_right * (0.3) + d_mouth_left * (0.2) + d_mouth_right * (0.2);
    float dist_para1 = ((dist_index / 4) / eye_distance) * 100;

    // case 2
    // float d_nose_top_perp = perpendicular_distance(x_nose, y_nose, x_eye_left, y_eye_left, x_eye_right, y_eye_right);
    // float d_nose_bottom_perp = perpendicular_distance(x_nose, y_nose, x_mouth_left, y_mouth_left, x_mouth_right, y_mouth_right);

    // float nose_top_perp_norm = (d_nose_top_perp / eye_distance) * 100;
    // float nose_bottom_perp_norm = (d_nose_bottom_perp / eye_distance) * 100;
    // //float dist_para2 = (nose_top_perp_norm / nose_bottom_perp_norm) * 10;
    // float dist_para2 = nose_bottom_perp_norm;

    //case 3
    float d_nose_face_top = perpendicular_distance(x_nose, y_nose, faceX, faceY , faceX + faceW, faceY);
    float d_nose_face_bottom = perpendicular_distance(x_nose, y_nose, faceX, faceY + faceH , faceX + faceW, faceY + faceH);
    
    float nose_face_top_perp_norm = (d_nose_face_top / eye_distance) * 100;
    float nose_face_bottom_perp_norm = (d_nose_face_bottom / eye_distance) * 100;

    float dist_para2 = nose_face_bottom_perp_norm;

    printf("dist_param_1 : %f , dist_param_2 : %f \n",dist_para1,dist_para2);

    // if (dist_para1 > 14.0 || dist_para1 < 11.5) {
    //     printf("distracted !\n");
    //     return true;
    // }
    //   if (dist_para2 < 11.5 || dist_para2 > 14.0) {
    //     printf("distracted !\n");
    //         return true;
    // }
        if (dist_para1 >= 18.50 ) {
        printf("looking left right distracted !\n");
        return true;
    }
    if (dist_para2 <=101.0 ) {
        printf("looking down distracted !\n");
        return true;
    }
        return false;
}

void app_main(void)
{
    //int variables and buffers
    int * pResults = NULL;
    unsigned char * pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
    int dist_counter = 0;

// Configure the built-in LED pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = ((1ULL<<LED_PIN)|(1ULL<<BUZZER_PIN)|(1ULL<<46)),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    //camera init 
    app_camera_init();
    //init spi
    init_spi();
    dms_create_timer();

    while(true){

        
        //get frame 
        ESP_LOGI(TAG, "Taking picture... \n");
        uint64_t start = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGI(TAG, "Camera capture failed\n");
            continue ;
        }
        
        ESP_LOGI(TAG, "Picture taken. \n");
        // ESP_LOGI(TAG, "Capture Frame Buffer : \n");
        // //print frame
        // for(int i=0;i<(fb->len);i++)
        // {
        //     printf("%02X",fb->buf[i]);
        //     if(i%1000 == 0)
        //     {
        //         vTaskDelay(10 / portTICK_RATE_MS);
        //     }
        // }
        // printf("\n");

        // // create capture frame buffer to mat image 
        Mat image(fb->height, fb->width, CV_8UC1, fb->buf);
        // rgb to bgr
       
        //cv::cvtColor(image, image, COLOR_RGB2GRAY);
        // // resize to 320X240
        cv::Mat resized_image;
        cv::resize(image, resized_image, Size(160, 120), INTER_LINEAR);
        //cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_MAKETYPE(CV_8U, 3), const_cast<uint8_t*>(IMAGE_ELEMENT));
        // ESP_LOGI(TAG, "Resize Frame Buffer : \n");
        //     // display cropp image face
        // uchar * arr_1 = image.isContinuous()? image.data: image.clone().data;
        // uint length_1 = image.total()*image.channels();
        // bool convertion = fmt2jpg(arr_1,length_1,image.cols,image.rows,PIXFORMAT_GRAYSCALE,90,&_jpg_buf,&_jpg_buf_len);
        // for (int i = 0; i < _jpg_buf_len; ++i) {
        //     if(i%1000 == 0){
        //         vTaskDelay(pdMS_TO_TICKS(10));
        //     }
        //     printf("%02X",_jpg_buf[i]);
        // }
        // }
        // printf("\n");
        
        //face detection inference
        std::vector<cv::Mat> channels = {resized_image, resized_image, resized_image};
        cv::Mat stackedImage;
        cv::merge(channels, stackedImage);
        faceroi_t faceroi = {1,1,1,1,1,1,1,1,1,1,1,1,1,1};
        pResults = facedetect_cnn(pBuffer, (unsigned char*)(stackedImage.ptr(0)), stackedImage.cols, stackedImage.rows, (int)stackedImage.step,&faceroi);
        
        if(pResults[0]>=1){
            if(faceroi.conf>70){
                double xfactor = 3.0;
            // new eye crop
                int cx = faceroi.eye_l_x + 2;
                int cy = faceroi.eye_l_y + 2;

                double x1 = 0.965 * cx;
                double x2 = 1.034 * cx;
                double w = (x2 - x1) * 2.1;
                double h = (w * 22 / 48);

                double margin_x = w / 2;
                double margin_y = h / 2;

                int min_x = static_cast<int>(cx - margin_x);
                int min_y = static_cast<int>(cy - margin_y);
                int max_x = static_cast<int>(cx + margin_x);
                int max_y = static_cast<int>(cy + margin_y);

                int rx1 = min_x;
                int ry1 = min_y;
                int rx2 = max_x;
                int ry2 = max_y;

                // new eye crop

                // old eye crop
                //  int rectangle_width = static_cast<int>(faceroi.w * 0.085 * xfactor);
                //  int rectangle_height = static_cast<int>(faceroi.h * 0.055 * xfactor);
                //  // int rectangle_width = 10;
                //  // int rectangle_height = 8;
                //  int rx1 = faceroi.eye_r_x - rectangle_width / 2;
                //  int ry1 = faceroi.eye_r_y - rectangle_height / 2;
                //  int rx2 = faceroi.eye_r_x  + rectangle_width / 2;
                //  int ry2 = faceroi.eye_r_y + rectangle_height / 2;

                // int lx1 = faceroi.eye_l_x - rectangle_length / 2;
                // int ly1 = faceroi.eye_l_y - rectangle_width / 2;
                // int lx2 = faceroi.eye_l_x + rectangle_length / 2;
                // int ly2 = faceroi.eye_l_y + rectangle_width / 2;

                // crop left eye
                //  cv::Mat eye_left = image(Range(ly1,ly2),Range(lx1,lx2)).clone();
                //  cv::resize(eye_left, eye_left, Size(48, 22), INTER_LINEAR);
                //  uchar * arr_1 = eye_left.isContinuous()? eye_left.data: eye_left.clone().data;
                //  uint length_1 = eye_left.total()*eye_left.channels();

                // crop right eye
                // cv::Mat eye_right = resized_image(Range(ry1,ry2),Range(rx1,rx2)).clone();
                cv::Mat eye_right = image(Range(ry1 * 4, ry2 * 4), Range(rx1 * 4, rx2 * 4)).clone();
                //printf("cx :%d , cy :%d \n",cx*4,cy*4);
                // Point center(cx*4, cy*4);
                // Scalar line_Color(0, 0, 255);
                // cv::circle(image, center,2, line_Color, 2);
                // uchar * arr_2 = image.isContinuous()? image.data: image.clone().data;
                // uint length_2 = image.total()*image.channels();
                // bool convertion1 = fmt2jpg(arr_2,length_2,image.cols,image.rows,PIXFORMAT_GRAYSCALE,90,&_jpg_buf,&_jpg_buf_len);
                // for (int i = 0; i < _jpg_buf_len; ++i) {
                //     if(i%1000 == 0){
                //         vTaskDelay(pdMS_TO_TICKS(10));
                //     }
                //     printf("%02X",_jpg_buf[i]);
                // }
                // printf("\n");
                cv::resize(eye_right, eye_right, Size(48, 22), INTER_LINEAR);
                // cv::Mat result;
                // cv::matchTemplate(eye_right, eye_right, result, cv::TM_CCOEFF_NORMED);
                //cv::equalizeHist(eye_right,eye_right);
                uchar *arr_1 = eye_right.isContinuous() ? eye_right.data : eye_right.clone().data;
                uint length_1 = eye_right.total() * eye_right.channels();

                // printf("eye hex : \n");
                // for (int i = 0; i < length_1; ++i) {
                //     if(i%1000 == 0){
                //         vTaskDelay(pdMS_TO_TICKS(10));
                //     }

                //     printf("%02X",arr_1[i]);
                //     }
                //     printf("\n");
                // printf("lenght eye : %d \n", length_1 );
                // // printf("eye hex  after conversion : \n");
                // bool convertion = fmt2jpg(arr_1, length_1, eye_right.cols, eye_right.rows, PIXFORMAT_GRAYSCALE, 90, &_jpg_buf, &_jpg_buf_len);
                // for (int i = 0; i < _jpg_buf_len; ++i)
                // {
                //     if (i % 1000 == 0)
                //     {
                //         vTaskDelay(pdMS_TO_TICKS(10));
                //         }

                //     printf("%02X", _jpg_buf[i]);
                // } 
                // printf("\n");
            // printf("lenght eye after conversion : %d \n", _jpg_buf_len );
            bool status =  distraction_state(faceroi.x,faceroi.y,faceroi.w,faceroi.h,faceroi.eye_l_x,faceroi.eye_l_y,faceroi.eye_r_x,faceroi.eye_r_y,faceroi.nose_x,faceroi.nose_y,faceroi.mouth_l_x,faceroi.mouth_l_y,faceroi.mouth_r_x,faceroi.mouth_r_y);
            // vTaskDelay(pdMS_TO_TICKS(500));

           
            if(status){
               dms_start_gpio_timer_led();
               dist_counter+=1;
               if(dist_counter>=2){
                    // dms_start_gpio_timer_buzzer();
                    dist_counter=0;
               }
            }
            else{
                 dist_counter=0;
                if(faceroi.conf>85){
                spi_send_data_chunks(arr_1, length_1);
            }     
                
            }
            uint64_t end = esp_timer_get_time();
            printf("esp timer %llu \n", (end - start)/1000);

            }
            
        }
        else{
            //  dms_start_gpio_timer_buzzer();
            vTaskDelay(500 / portTICK_RATE_MS);
        }
        //cv::Mat croppedImage = image(Range(pResults[2],pResults[2]+pResults[4]),Range(pResults[1],pResults[1]+pResults[3])); 
        //pResults = facedetect_cnn(pBuffer, (unsigned char*)(fb), fb->width, fb->height, 960);
        //pResults = facedetect_cnn(pBuffer, (unsigned char*)(IMAGE_ELEMENT), IMAGE_WIDTH, IMAGE_HEIGHT, 300);
        //free cam buf
        esp_camera_fb_return(fb);
        
          
        //watchdog delay
        vTaskDelay(10 / portTICK_RATE_MS);
        if(_jpg_buf!= NULL){
            printf("jpg_buf freed\r\n");
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

    }

}
