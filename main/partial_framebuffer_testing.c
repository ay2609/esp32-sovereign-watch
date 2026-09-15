#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>

//#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "esp_sleep.h"

#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "esp_netif.h"
#include "esp_timer.h"

#include "icm42670.h" // imu
#include "mcp342x.h" // ADC
#include "ds3231.h" // clock

#include "st7789.h"
#include "fontx.h"
#include "fontbdf.h"

#define MCP3427_I2C_ADDRESS         0x6E

#define GAIN  MCP342X_GAIN1       // +-2.048
#define CHANNEL MCP342X_CHANNEL1
#define RESOLUTION MCP342X_RES_16 // 16-bit, 15 sps

#define BUTTON_1    GPIO_NUM_18 // right side
#define BUTTON_2    GPIO_NUM_36 // 36 doesnt work for external interrupt? // what is external interrupt vs the 32 per core?
#define BUTTON_3    GPIO_NUM_35
#define BUTTON_4    GPIO_NUM_4 // left side
#define BUTTON_5    GPIO_NUM_5

#define COLOR rgb565(159,171,162)

//#define IMU_WOM GPIO_NUM_15 // wake on motion

#define ICM42670_I2C_ADDRESS        0x69 // Other addresses
#define PORT 0

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

#define ENABLE_MAIN 1

#include "driver/ledc.h"
#include "esp_err.h"

#define BACKLIGHT_PIN GPIO_NUM_7  // Replace with your backlight GPIO pin
#define PWM_FREQUENCY 5000         // 5 kHz PWM frequency
#define PWM_CHANNEL LEDC_CHANNEL_0 // Select a channel for PWM
#define PWM_TIMER LEDC_TIMER_0     // Use timer 0

#define rgb16bit(r, g, b) ((r << 11) | (g << 5) | b)

volatile int currentScreen = 0;

static void SPIFFS_Directory(char * path) {
    DIR* dir = opendir(path);
    assert(dir != NULL);
    while (true) {
        struct dirent*pe = readdir(dir);
        if (!pe) break;
        ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
    }
    closedir(dir);
}

// ---

typedef struct {
    uint16_t color;
    uint16_t bg;
} scheme;

scheme shell = {rgb16bit(29, 63, 29),
                rgb16bit(31, 0, 0) };

scheme gravity = {rgb16bit(24, 12, 12),
                  rgb16bit(4, 10, 5)};

volatile scheme *theme;

void ST7789(void *pvParameters)
{

    uint8_t ascii[40];
    uint8_t ascii2[40];
    sprintf((char *)ascii, "00");

    theme = &shell;

    TFT_t dev;

    // init LCD
    spi_clock_speed(80000000); // 80 MHz
    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    // load characters into memory
    bdf_t file_data;

    file_data.filename = "/spiffs/MPFW32_C.bdf";
    file_data.chars = heap_caps_calloc(2600, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data);

    bdf_t file_data_small;

    file_data_small.filename = "/spiffs/MPFW12_C.bdf";
    file_data_small.chars = heap_caps_calloc(1400, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_small.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_small.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data_small);

    bdf_t file_data_big;

    file_data_big.filename = "/spiffs/MPFW52_C.bdf";
    file_data_big.chars = heap_caps_calloc(3700, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_big.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_big.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data_big);

    const char * mpfw32 = "/spiffs/MPFW32_C.bdf";
    BDFChar bdfChar32;
    loadBDFChar(mpfw32, 50, &bdfChar32);

    const char * mpfw12 = "/spiffs/MPFW12_C.bdf";
    BDFChar bdfChar12;
    loadBDFChar(mpfw12, 50, &bdfChar12);

    const char * mpfw52 = "/spiffs/MPFW52_C.bdf";
    BDFChar bdfChar52;
    loadBDFChar(mpfw52, 50, &bdfChar52);

    lcdFillScreen(&dev, theme->bg);
    lcdDrawFinish(&dev);

    static struct tm lastSecond = {0};
    static struct tm lastMinute = {0};
    static struct tm lastHour = {0};

    int prevScreen = currentScreen;
    bool needs_update = true;

    uint16_t xpos;
    uint16_t base_ypos = 19;
    uint16_t ypos;

    uint16_t centerX = CONFIG_WIDTH - 40;
    uint16_t centerY = CONFIG_HEIGHT - 40;

    uint16_t centerX2 = CONFIG_WIDTH - 130;
    uint16_t centerY2 = CONFIG_HEIGHT - 80;

    uint16_t centerX3 = CONFIG_WIDTH - 30;
    uint16_t centerY3 = 45;

//    uint16_t color = WHITE;

    int num_screens = 1;

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(10));

        if (prevScreen != currentScreen) { // changed in some other thread
            lcdFillScreen(&dev, theme->bg);
            prevScreen = currentScreen;
        }

        if (gpio_get_level(BUTTON_3) && gpio_get_level(BUTTON_4)) {
            needs_update = true;
            currentScreen++;
            currentScreen %= num_screens;
            prevScreen = currentScreen;
            lcdFillScreen(&dev, theme->bg);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_4)) { // up a screen
            needs_update = true;
            currentScreen += 1;
            currentScreen %= num_screens;
            prevScreen = currentScreen;
            lcdFillScreen(&dev, theme->bg);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_5)) { // down a screen
            needs_update = true;
            currentScreen -= 1;
            currentScreen = (currentScreen == -1) ? num_screens - 1 : currentScreen;
            prevScreen = currentScreen;
            lcdFillScreen(&dev, theme->bg);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (currentScreen == 0) {


        }


    }

}

static const char *TAG2 = "ST7789";

void app_main(void)
{
    gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
    gpio_pulldown_dis(BUTTON_1); // External pull-down already present
    gpio_pullup_dis(BUTTON_1);
    gpio_set_direction(BUTTON_2, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_2);
    gpio_pullup_dis(BUTTON_2);
    gpio_set_direction(BUTTON_3, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_3);
    gpio_pullup_dis(BUTTON_3);
    gpio_set_direction(BUTTON_4, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_4);
    gpio_pullup_dis(BUTTON_4);
    gpio_set_direction(BUTTON_5, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_5);
    gpio_pullup_dis(BUTTON_5);

    ESP_LOGI(TAG2, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
            .base_path = "/spiffs",
            .partition_label = NULL,
            .max_files = 12,
            .format_if_mount_failed =true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG2, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG2, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG2, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total,&used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG2,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG2,"Partition size: total: %d, used: %d", total, used);
    }

    SPIFFS_Directory("/spiffs/");
    xTaskCreate(ST7789, "ST7789", 1024*6, NULL, 2, NULL);
}
