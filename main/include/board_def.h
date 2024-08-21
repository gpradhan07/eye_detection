

#define SPI_MISO    22
#define SPI_MOSI    19
#define SPI_SCLK    21

#define SD_CS       0
#define SD_MOSI     SPI_MOSI
#define SD_MISO     SPI_MISO
#define SD_CLK      SPI_SCLK

#define TFT_MISO    SPI_MISO
#define TFT_MOSI    SPI_MOSI
#define TFT_SCLK    SPI_SCLK
#define TFT_CS      12      // Chip select control pin
#define TFT_DC      15      // Data Command control pin
#define TFT_BK      2       // TFT backlight  pin
#define TFT_RST     GPIO_NUM_MAX    //No use

#define TFT_WITDH   240
#define TFT_HEIGHT  240

#define I2C_SDA     18
#define I2C_SCL     23

#define IIS_SCLK    14
#define IIS_LCLK    32
#define IIS_DSIN    -1
#define IIS_DOUT    33

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       26
#define Y3_GPIO_NUM       13
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM    5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25
#define XCLK_FREQ       20000000

// ai thinker board

#ifdef AI_THINKER
// #define CAM_PIN_PWDN 32
// #define CAM_PIN_RESET -1 //software reset will be performed
// #define CAM_PIN_XCLK 0
// #define CAM_PIN_SIOD 26
// #define CAM_PIN_SIOC 27

// #define CAM_PIN_D7 35
// #define CAM_PIN_D6 34
// #define CAM_PIN_D5 39
// #define CAM_PIN_D4 36
// #define CAM_PIN_D3 21
// #define CAM_PIN_D2 19
// #define CAM_PIN_D1 18
// #define CAM_PIN_D0 5
// #define CAM_PIN_VSYNC 25
// #define CAM_PIN_HREF 23
// #define CAM_PIN_PCLK 22

// s3 ov2640
#else 
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK -1
#define CAM_PIN_SIOD 2
#define CAM_PIN_SIOC 1
 
#define CAM_PIN_D7 13
#define CAM_PIN_D6 47
#define CAM_PIN_D5 14
#define CAM_PIN_D4 45
#define CAM_PIN_D3 48
#define CAM_PIN_D2 39
#define CAM_PIN_D1 38
#define CAM_PIN_D0 40
#define CAM_PIN_VSYNC 41
#define CAM_PIN_HREF 42
#define CAM_PIN_PCLK 21


#endif

