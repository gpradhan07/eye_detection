
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
// #include <stdlib.h>
#include "esp_psram.h"
#include "esp_chip_info.h"
// #include "esp_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
static const char *TAG = "knet";
TimerHandle_t timer_gpio_led;
TimerHandle_t timer_gpio_buzzer;

#define BUZZER_PIN GPIO_NUM_15
#define LED_PIN GPIO_NUM_16


void gpio_timer_callback_led(TimerHandle_t gpio_timer)
{
        ESP_LOGI(TAG, "timer on ... \n");
       
        gpio_set_level(LED_PIN, 0);

}

void gpio_timer_callback_buzzer(TimerHandle_t gpio_timer)
{
        ESP_LOGI(TAG, "timer on ... \n");
        
        gpio_set_level(BUZZER_PIN, 0);

}
void dms_create_timer(void)
{
    timer_gpio_led = xTimerCreate("gpio_timer", pdMS_TO_TICKS(300), pdFALSE,NULL,gpio_timer_callback_led);
    if(timer_gpio_led){
            ESP_LOGI(TAG, "Timer created successfully added\r\n");
    }
    timer_gpio_buzzer = xTimerCreate("gpio_timer", pdMS_TO_TICKS(500), pdFALSE,NULL,gpio_timer_callback_buzzer);
    if(timer_gpio_buzzer){
            ESP_LOGI(TAG, "Timer created successfully added\r\n");
    }
}

void dms_start_gpio_timer_led(void)
{
     gpio_set_level(LED_PIN, 1);
    xTimerStart(timer_gpio_led,0);

}
void dms_start_gpio_timer_buzzer(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    xTimerStart(timer_gpio_buzzer,0);

}