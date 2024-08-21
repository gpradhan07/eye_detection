 
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
// #include "esp_flash_spi_init.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define GPIO_HANDSHAKE 46
#define GPIO_MOSI 11
#define GPIO_MISO 12
#define GPIO_SCLK 9
#define GPIO_CS 10
#define RCV_HOST  SPI2_HOST
#define GPIO_HOOTER 17

#define SPI_RECV_SIZE 1500

spi_slave_transaction_t t;

DMA_ATTR char recvbuf[129]="";
DMA_ATTR char sendbuf[1500]="";
static const char *TAG = "knet";
//GPIO hooter configuration
void hotter_level_beep(void) 
{
    printf("Hooter high...\r\n");
    gpio_set_level(GPIO_HOOTER, 1);
    vTaskDelay(1000 / portTICK_RATE_MS);
    gpio_set_level(GPIO_HOOTER, 0);
    printf("Hooter low...\r\n");
}

void configure_hooter_gpio(void)
{
    gpio_config_t hooter_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<GPIO_HOOTER)
    };   
    gpio_config(&hooter_conf);
}

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 0);
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

void init_spi(void)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    //Configuration for the handshake line
    // gpio_config_t io_conf={
    //     .intr_type=GPIO_INTR_DISABLE,
    //     .mode=GPIO_MODE_OUTPUT,
    //     .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    // };

    //Configure handshake line as output
    // gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if(ret!=ESP_OK){

        printf("failed spi_slave_initialize\r\n");
    }
    memset(recvbuf, 0, 33);

    memset(&t, 0, sizeof(t));


}

esp_err_t spi_send_data_chunks(uint8_t *data, int len)
{
    int spi_recv = 0,send_len = 0;
    int i,n = 0;
    send_len = len;
    esp_err_t ret;
    memset(sendbuf,0x00,sizeof(sendbuf));
    memset(recvbuf,0x00,sizeof(recvbuf));
    do{
        if(send_len > SPI_RECV_SIZE){
            if(n == 0)
            {
                memcpy(sendbuf,"###",3);
                spi_recv = SPI_RECV_SIZE ;
                memcpy(sendbuf+3,data,spi_recv - 3);

            } else {
                spi_recv = SPI_RECV_SIZE;
                memcpy(sendbuf,(data+(n*SPI_RECV_SIZE)),spi_recv);
            }
            send_len = send_len - spi_recv + 3;

        } else {
            if(n == 0)
            {
                memcpy(sendbuf,"###",3);
                spi_recv = send_len;
                memcpy(sendbuf+3,data,spi_recv);
                memcpy(sendbuf+spi_recv+3,"$#@",3);
                spi_recv = spi_recv + 6;
            } else {
                spi_recv = send_len; 
                memcpy(sendbuf,(data+((n*SPI_RECV_SIZE)-3)),spi_recv);
                memcpy(sendbuf+spi_recv,"$#@",3);
                spi_recv = spi_recv + 3;
            }
        send_len = 0;
                
        }

        t.length=spi_recv*8;
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;

        // use pic->buf to access the image
        ret = spi_slave_transmit(RCV_HOST, &t, pdMS_TO_TICKS(5000));
        if(ret!=ESP_OK){

            printf("failed spi_slave_transmit\r\n");
        }
        // ESP_LOGI(TAG, "%d---%d--%d\r\n", spi_recv,send_len,n);
        // for(i=0;i<(spi_recv);i++)
        // {
        //    printf("%02X",sendbuf[i]);
        // }
        printf("\n\r");
        printf("send data len : %d\r\n",spi_recv);

        memset(sendbuf,0x00,sizeof(sendbuf));
        memset(recvbuf,0x00,sizeof(recvbuf));
        n++;

    }while(send_len != 0);
    return 0;
}
