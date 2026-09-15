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

void init_backlight_pwm()
{
    // Configure the PWM timer
    ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = PWM_TIMER,
            .duty_resolution  = LEDC_TIMER_13_BIT,  // Set duty resolution (13-bit for 0-8191)
            .freq_hz          = PWM_FREQUENCY,
            .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure the PWM channel
    ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = PWM_CHANNEL,
            .timer_sel      = PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = BACKLIGHT_PIN,
            .duty           = 0,  // Start with backlight off (0 duty)
            .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_backlight_brightness(uint16_t brightness)
{
    // Adjust the duty cycle to change brightness
    // brightness should be between 0 (off) and 8191 (full brightness)
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, brightness));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL));
}

static float convert_voltage_to_percentage(float voltage)
{
    float min_voltage = 0.94; // 0.94 actual
    float max_voltage = 1.165; // 1.7 actual

    // Convert the voltage to a percentage
    float percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;

    if (percentage > 100)
        percentage = 100;
    else if (percentage < 1)
        percentage = 1;

    return percentage;
}

//static const char *TAG = "ST7789";
volatile int currentScreen = 0;
volatile int backlightOn = 0;

typedef struct {
    float temperature;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm42670_data_t;

icm42670_data_t imuData;

static QueueHandle_t timeQueue = NULL;
static QueueHandle_t ICMQueue = NULL;
static QueueHandle_t MCPQueue = NULL;


TaskHandle_t getDS3231Handle = NULL;
TaskHandle_t getICM42670Handle = NULL;
TaskHandle_t getMCP3427Handle = NULL;

SemaphoreHandle_t i2cMutex;
static mcp342x_t adc;


static double convert_to_degrees(int value)
{
    return ((value * 90.0) / 8192.0) + 0.5; // + 90
}

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

static void parse_buffer(char* buffer, int* percents) {

    char *line = strtok(buffer, "\n");

    while (line != NULL) {
        if (strstr(line, "IDLE0")) {
            // Extract the percentage for IDLE 0
            sscanf(line, "IDLE0 %*d %d", &percents[0]);
        }
        if (strstr(line, "IDLE1")) {
            // Extract the percentage for IDLE 1
            sscanf(line, "IDLE1 %*d %d", &percents[1]);
        }

        line = strtok(NULL, "\n");
    }

}

static const char *TAG = "wifi station";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init(i2c_dev_t* dev) {

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    const wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t sta_config_1 = {
        .sta = {
                .ssid = "YOUR_WIFI_SSID_1",
                .password = "YOUR_WIFI_PASSWORD_1",
        }
    };

    wifi_config_t sta_config_2 = {
        .sta = {
                .ssid = "YOUR_WIFI_SSID_2",
                .password = "YOUR_WIFI_PASSWORD_2",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config_1));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 "YOUR_WIFI_SSID_1", "YOUR_WIFI_PASSWORD_1");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 "YOUR_WIFI_SSID_1", "YOUR_WIFI_PASSWORD_1");

        ESP_LOGI(TAG, "Primary AP failed – trying secondary");
        // 1) clear out old bits
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        // 2) tear down any half-connect
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        // 3) set new config
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config_2));
        // 4) initiate a fresh connect
        ESP_ERROR_CHECK(esp_wifi_connect());
        // 5) wait again
        bits = xEventGroupWaitBits(s_wifi_event_group,
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                   pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                     "YOUR_WIFI_SSID_2", "...");
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                     "YOUR_WIFI_SSID_2", "...");
        }

    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&cfg);

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGI("WiFi", "Failed to update system time within 10s timeout");
    }

    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    // Example structure to hold RTC time components

    setenv("TZ", "EST+5", 1);

    struct tm* timeinfo = localtime(&current_time.tv_sec);

    timeinfo->tm_hour++;

    ds3231_set_time(dev, timeinfo);
}

static void enter_light_sleep()
{
    gpio_set_direction(BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BACKLIGHT_PIN, 0);
    gpio_hold_en(BACKLIGHT_PIN);

    uint64_t wakeup_pins = (1ULL << BUTTON_1);
    esp_sleep_enable_ext1_wakeup(wakeup_pins, ESP_EXT1_WAKEUP_ANY_HIGH); // Wake up on any high level

    // Enter light sleep
    esp_light_sleep_start();

    // after wakeup: exit light sleep and resume execution here
    gpio_hold_dis(BACKLIGHT_PIN);
    gpio_set_level(BACKLIGHT_PIN, 1);
}

static void getMCP3427(void *arg)
{
    // Clear device descriptor
    memset(&adc, 0, sizeof(adc));

    ESP_ERROR_CHECK(mcp342x_init_desc(&adc, MCP3427_I2C_ADDRESS, 0, 13, 14));

    adc.channel = CHANNEL;
    adc.gain = GAIN;
    adc.resolution = RESOLUTION;
    adc.mode = MCP342X_CONTINUOUS;

    uint32_t wait_time;
    ESP_ERROR_CHECK(mcp342x_get_sample_time_us(&adc, &wait_time)); // microseconds
    wait_time = wait_time / 1000 + 1; // milliseconds

    // start first conversion
    ESP_ERROR_CHECK(mcp342x_start_conversion(&adc));

    while (1)
    {
        uint32_t ulNotificationValue;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
        {
            if (ulNotificationValue == 1)
            {
                if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
                {
                    // Wait for conversion
                    vTaskDelay(pdMS_TO_TICKS(wait_time));

                    // Read data
                    float volts;
                    ESP_ERROR_CHECK(mcp342x_get_voltage(&adc, &volts, NULL));
                    ESP_LOGI("MCP", "Channel: %d, voltage: %0.4f\n", adc.channel, volts);

                    // Send the structured data to the queue
                    if (xQueueSend(MCPQueue, &volts, portMAX_DELAY) != pdPASS)
                    {
                        ESP_LOGI("MCP3427", "Failed to send data to queue");
                    }
                    xSemaphoreGive(i2cMutex);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

void ds3231(void *pvParameters) {
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));

    wifi_init(&dev);
    ESP_ERROR_CHECK(esp_wifi_stop()); // don't need after initializing time, for now.

    // arbitrary, doesn't affect anything bc we're not setting ds3231 with it
    struct tm time = {
            .tm_year = 124,
            .tm_mon = 10,
            .tm_mday = 20,
            .tm_hour = 10,
            .tm_min = 30,
            .tm_sec = 45,
    };

    currentScreen = 1;

    while (1)
    {
        float temp;

        uint32_t ulNotificationValue;
        if(xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == true) {

            if(ulNotificationValue == 1) {
                vTaskDelay(pdMS_TO_TICKS(100));

                if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

                    if (ds3231_get_time(&dev, &time) != ESP_OK) {
                        printf("Could not get time\n");
                        continue;
                    }

                    xQueueSend(timeQueue, &time, 0);
                    xSemaphoreGive(i2cMutex);
                }
            }
        }
    }

}

void icm42670(void *pvParameters) {
    icm42670_t dev = {0};

    // THERE EXISTS A SLEEP MODE DARE I SAY YIPPEE HOORAY WOO HOO, use it when implementing sleep and deep sleep or wtv

    ESP_ERROR_CHECK(icm42670_init_desc(&dev, ICM42670_I2C_ADDRESS, PORT, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* optional shit */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_50HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_50HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_4G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_250DPS));

    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI("ICM42670", "Initial temperature: %.2f°C", temperature);

    while (1)
    {
        uint32_t ulNotificationValue;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
        {
            if (ulNotificationValue == 1)
            {
                if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
                {
                    // DO NOT ADD DELAY AFTER TAKING MUTEX

                    // Read the interrupt status to clear the interrupt
                    uint8_t int_status;
                    ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_INT_STATUS, (int16_t*)&int_status));

                    // Read the data
                    icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_X1, &imuData.accel_x);
                    icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Y1, &imuData.accel_y);
                    icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Z1, &imuData.accel_z);
                    icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_X1, &imuData.gyro_x);
                    icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Y1, &imuData.gyro_y);
                    icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Z1, &imuData.gyro_z);

                    xQueueSend(ICMQueue, &imuData, portMAX_DELAY);
                    xSemaphoreGive(i2cMutex);
                }
            }
        }

//        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

float random_float(void) {
    return (float) esp_random() / UINT32_MAX;
}



typedef struct {
    uint16_t color;
    uint16_t bg;
} scheme;

scheme shell = {rgb16bit(29, 63, 29),
                rgb16bit(31, 0, 0) };

scheme gravity = {rgb16bit(24, 12, 12),
                  rgb16bit(4, 10, 5)};

void ST7789(void *pvParameters)
{
    int mem_check_count = 0;
    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

    uint8_t ascii[40];
    uint8_t ascii2[40];
    sprintf((char *)ascii, "00");

    scheme *theme = &shell;

    TFT_t dev;

    // init LCD
    spi_clock_speed(80000000); // 80 MHz
    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
    lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

    // load characters into memory

    bdf_t file_data;

    file_data.filename = "/spiffs/MPFW32_C.bdf";
    file_data.chars = heap_caps_calloc(2600, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data);

    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

    bdf_t file_data_small;

    file_data_small.filename = "/spiffs/MPFW12_C.bdf";
    file_data_small.chars = heap_caps_calloc(1400, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_small.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_small.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data_small);

    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

    bdf_t file_data_big;

    file_data_big.filename = "/spiffs/MPFW52_C.bdf";
    file_data_big.chars = heap_caps_calloc(3700, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_big.index = heap_caps_calloc(125, sizeof(uint32_t), MALLOC_CAP_32BIT);
    file_data_big.lengths = heap_caps_calloc(125, sizeof(uint8_t), MALLOC_CAP_8BIT);

    loadFileDynamic(&file_data_big);

    ESP_LOGI("MEM", "CHECK #%d", mem_check_count);
    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Largest block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEM", "Minimum ever free heap: %lu", esp_get_minimum_free_heap_size());
    mem_check_count++;

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


#if 0 // fps
    uint64_t frame_start_time;
    uint64_t frame_end_time;
    float frame_time;
    float fps = 0.0;
    uint16_t xpos = (CONFIG_WIDTH - strlen((char *)ascii) * bdfChar32.width) / 2;
    uint16_t ypos = (CONFIG_HEIGHT - bdfChar32.height) / 2;

    while(1) {


        // timer
        frame_start_time = esp_timer_get_time();

        // draw call
        memset(ascii, 0, sizeof(ascii));
        sprintf((char *)ascii, "%.2f", fps);
        lcdFillScreen(&dev, theme->bg);
//        lcdDrawFillRect(&dev, xpos - 1, ypos - 1, xpos + (5 * bdfChar32.width) + 1, ypos + bdfChar32.height + 1, theme->bg);
        lcdSetFontDirection(&dev, 0);
        lcdDrawString2(&dev, &file_data, xpos, ypos, ascii, WHITE);
        lcdDrawPixel(&dev, xpos, ypos, WHITE);

//        lcdDrawBlock(&dev, xpos - 1, ypos - 1, xpos + (5 * bdfChar32.width) + 1, ypos + bdfChar32.height + 1);
        lcdDrawFinish(&dev);
        ESP_LOGI(TAG, "fps: %.2f", fps);

        // calculate new fps
        frame_end_time = esp_timer_get_time();
        frame_time = (frame_end_time - frame_start_time) / 1e6;
        fps = 1.0 / frame_time;
    }
#endif

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

    char buffer[512];
    int percents[2];

    int i = 0;

//    init_backlight_pwm();
//    set_backlight_brightness(7000);

    int num_screens = 2;
    bool finished_opening_sequence = false;

    float ax = 0, ay = 0, az = 0;

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

        if (gpio_get_level(BUTTON_2)) {
            lcdFillScreen(&dev, theme->bg);
            lcdDrawFinish(&dev);
            vTaskDelay(pdMS_TO_TICKS(200));
            enter_light_sleep();
            needs_update = true;
        }

        if (currentScreen == 0) {

            // run the opening sequence

            if (!finished_opening_sequence) {
                lcdFillScreen(&dev, theme->bg);

                memset(ascii, 0, sizeof(ascii));
                sprintf((char *) ascii, "%s", "OS");
                memset(ascii2, 0, sizeof(ascii2));
                sprintf((char *) ascii2, "%s", "sovereign");
                int textlen1 = strlen((char *) ascii) * bdfChar52.width;
                int textlen2 = strlen((char *) ascii2) * bdfChar32.width;

                lcdDrawString2(&dev, &file_data_big, &bdfChar52, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + 2,
                               ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 2, ascii, theme->color);


                lcdDrawString2(&dev, &file_data, &bdfChar32, ((CONFIG_WIDTH - (textlen2)) / 2),
                               ((CONFIG_HEIGHT - bdfChar32.height) / 2), ascii2, theme->color);

                lcdDrawFinish(&dev);
                vTaskDelay(pdMS_TO_TICKS(100));

                // add ring
                lcdDrawRoundRect(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                                 ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                                 ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                                 ((CONFIG_HEIGHT + bdfChar32.height) / 2) + bdfChar52.height + 4, 5, theme->color);
                lcdDrawLine(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2), ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 6, theme->color);
                lcdDrawLine(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                            ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 6, theme->color);

                lcdDrawRoundRect(&dev, (CONFIG_WIDTH - textlen2) / 2, (CONFIG_HEIGHT - bdfChar32.height) / 2,
                                 ((CONFIG_WIDTH + textlen2) / 2) + 2, (CONFIG_HEIGHT + bdfChar32.height) / 2, 5, theme->color);

                lcdDrawFinish(&dev);
                vTaskDelay(pdMS_TO_TICKS(150));

                // fill it white
                lcdDrawFillRect(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                                ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                                ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                                ((CONFIG_HEIGHT + bdfChar32.height) / 2) + bdfChar52.height + 6, theme->color);
                lcdDrawFillRect(&dev, (CONFIG_WIDTH - textlen2) / 2, (CONFIG_HEIGHT - bdfChar32.height) / 2,
                                ((CONFIG_WIDTH + textlen2) / 2) + 2, (CONFIG_HEIGHT + bdfChar32.height) / 2, theme->color);


                lcdDrawFinish(&dev);
                vTaskDelay(pdMS_TO_TICKS(150));
                lcdFillScreen(&dev, theme->bg);

                // reset and then add text + ring

                lcdDrawString2(&dev, &file_data_big, &bdfChar52, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + 2,
                               ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 2, ascii, theme->color);

                lcdDrawString2(&dev, &file_data, &bdfChar32, ((CONFIG_WIDTH - (textlen2)) / 2),
                               ((CONFIG_HEIGHT - bdfChar32.height) / 2), ascii2, theme->color);

                lcdDrawRoundRect(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                                 ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                                 ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                                 ((CONFIG_HEIGHT + bdfChar32.height) / 2) + bdfChar52.height + 4, 5, theme->color);
                lcdDrawLine(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2), ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2),
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 6, theme->color);
                lcdDrawLine(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                            ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4,
                            ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 6, theme->color);

                lcdDrawRoundRect(&dev, (CONFIG_WIDTH - textlen2) / 2, (CONFIG_HEIGHT - bdfChar32.height) / 2,
                                 ((CONFIG_WIDTH + textlen2) / 2) + 2, (CONFIG_HEIGHT + bdfChar32.height) / 2, 5, theme->color);

                lcdDrawFinish(&dev);
                vTaskDelay(pdMS_TO_TICKS(150));
                lcdFillScreen(&dev, theme->bg);

                // just text
                lcdDrawString2(&dev, &file_data_big, &bdfChar52, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + 2,
                               ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 2, ascii, theme->color);

                lcdDrawString2(&dev, &file_data, &bdfChar32, ((CONFIG_WIDTH - (textlen2)) / 2),
                               ((CONFIG_HEIGHT - bdfChar32.height) / 2), ascii2, theme->color);

                lcdDrawFinish(&dev);

                finished_opening_sequence = true;
            } else {
                memset(ascii, 0, sizeof(ascii));
                sprintf((char *) ascii, "%s", "OS");
                int textlen1 = strlen((char *) ascii) * bdfChar52.width;
                memset(ascii2, 0, sizeof(ascii2));
                sprintf((char *) ascii2, "%s", "sovereign");
                int textlen2 = strlen((char *) ascii2) * bdfChar32.width;

                // clear
                // SECOND is sovereign (dumbass)
                lcdDrawFillRect(&dev, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) - 10,
                                ((CONFIG_HEIGHT + bdfChar32.height) / 2),
                                ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + textlen1 + 4 + 10,
                                ((CONFIG_HEIGHT + bdfChar32.height) / 2) + bdfChar52.height + 6, theme->bg);
                lcdDrawFillRect(&dev, ((CONFIG_WIDTH - textlen2) / 2) - 10, (CONFIG_HEIGHT - bdfChar32.height) / 2,
                                ((CONFIG_WIDTH + textlen2) / 2) + 2 + 10, ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 10 , theme->bg);

                int rand_shift_sov = (int) (10 * random_float());
//                int rand_shift_os = (int) (10 * random_float());

                // fill bottom half
                lcdDrawString2(&dev, &file_data, &bdfChar32, ((CONFIG_WIDTH - (textlen2)) / 2) - rand_shift_sov,
                               ((CONFIG_HEIGHT - bdfChar32.height) / 2), ascii2, theme->color);
                lcdDrawFillRect(&dev, ((CONFIG_WIDTH - textlen2) / 2) - 10,
                                (CONFIG_HEIGHT - bdfChar32.height) / 2,
                                ((CONFIG_WIDTH + textlen2) / 2) + 2 + 10,
                                ((CONFIG_HEIGHT - bdfChar32.height) / 2) + 15 , theme->bg); // 10 here for 10 pixel cutoff


                lcdDrawString2(&dev, &file_data_big, &bdfChar52, ((CONFIG_WIDTH - (9 * bdfChar32.width)) / 2) + 2,
                               ((CONFIG_HEIGHT + bdfChar32.height) / 2) + 2, ascii, theme->color);
                lcdDrawString3(&dev, &file_data, &bdfChar32, ((CONFIG_WIDTH - (textlen2)) / 2) + rand_shift_sov,
                               ((CONFIG_HEIGHT - bdfChar32.height) / 2), ascii2, theme->color);

                lcdDrawFinish(&dev);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        else if (currentScreen == 1) {

//            set_backlight_brightness(4096);

            if (0) {
                vTaskGetRunTimeStats(buffer);
                ESP_LOGI("Stats", "Stats: \n%s", buffer);

                parse_buffer(buffer, percents);
                ESP_LOGI("Stats", "Stats - IDLE0: %d | IDLE1: %d", percents[0], percents[1]);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
            xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
            xTaskNotify(getDS3231Handle, 1, eSetValueWithOverwrite);
            xTaskNotify(getMCP3427Handle, 1, eSetValueWithOverwrite);

            struct tm currentTime;

            i += 1;
            i = i % 720;

            // CIRCLE 1

            lcdDrawFillCircle(&dev, centerX, centerY, 110, theme->bg);

            // cool lines
            for (int x = -40; x <= 40; x += 20) {
                lcdDrawLineAngle(&dev, -50, x, 50, x, centerX, centerY, 45 + (-i / 2), theme->color);
                lcdDrawLineAngle(&dev, -50, x, 50, x, centerX, centerY, -45 + (-i / 2), theme->color);
            }

            for (int x = 0; x < 360; x += 45) {
                lcdDrawFillRectAngle(&dev, centerX, centerY, 40, 26, 0, 50 + 12, x + (-i / 2), theme->bg);
                lcdDrawLineAngle(&dev, 75, 0, 106, 0, centerX, centerY, x + i, theme->color);
            }


            for (int x = 0; x < 360; x += 45) {
                lcdDrawLineAngle(&dev, 0, 0, 71, 0, centerX, centerY, x + (-i / 2), theme->color);
                lcdDrawLineAngle(&dev, 0, 0, 71, 0, centerX, centerY, x + (-i / 2), theme->color);
            }

            // pointers

            // outer
            lcdDrawRectAngle(&dev, centerX, centerY, 150, 150, i, theme->color);
            lcdDrawRectAngle(&dev, centerX, centerY, 150, 150, i + 45, theme->color);
            lcdDrawRegularPolygon(&dev, centerX, centerY, 8, 106, i, theme->color);

            // middle
            lcdDrawRectAngle(&dev, centerX, centerY, 100, 100, -i / 2, theme->color);
            lcdDrawRectAngle(&dev, centerX, centerY, 100, 100, (-i / 2) + 45, theme->color);
            lcdDrawRegularPolygon(&dev, centerX, centerY, 8, 71, -i / 2, theme->color);

            lcdDrawTriangleAtAngle(&dev, 0, 106, -11, 75, 11, 75, centerX, centerY, i, WHITE);
            lcdDrawTriangleAtAngle(&dev, 0, 71, -11, 50, 11, 50, centerX, centerY, -i / 2, WHITE);

            // CIRCLE 2
            lcdDrawFillCircle(&dev, centerX3, centerY3, 75, theme->bg);

            lcdDrawRegularPolygon(&dev, centerX3, centerY3, 3, 45, -i - 90, theme->color);
            lcdDrawRegularPolygon(&dev, centerX3, centerY3, 3, 45, -i + 90, theme->color);

            lcdDrawCircle(&dev, centerX3, centerY3, 45, theme->color);
            lcdDrawCircle(&dev, centerX3, centerY3, 21, theme->color);

            lcdDrawRegularPolygon(&dev, centerX3, centerY3, 3, 70, -i + 90, theme->color);
            lcdDrawCircle(&dev, centerX3, centerY3, 70, theme->color);


            if (xQueueReceive(timeQueue, &currentTime, 0)) {

                xpos = 8;

                if (currentTime.tm_hour != lastHour.tm_hour || needs_update) {
                    ypos = base_ypos;
                    // Clear BG

                    memset(ascii, 0, sizeof(ascii));

                    int hour = currentTime.tm_hour;
                    bool am;

                    ESP_LOGI("THE FUCKING HOUR", "%d", hour);

                    am = (hour >= 12) ? false : true;

                    if (hour == 0) {
                        hour = 12;
                        am = true;
                    } else if (hour < 12) {
                        am = true;
                    } else if (hour == 12) {
                        hour = 12;
                        am = false;
                    } else {
                        hour -= 12;
                        am = false;
                    }

                    if (hour < 10) {
                        sprintf((char *) ascii, "0%d", hour);
                    } else {
                        sprintf((char *) ascii, "%d", hour);
                    }

                    lcdDrawFillRect(&dev, xpos - 1, ypos - 1, xpos + (strlen((char *) ascii) * bdfChar52.width) + 1,
                                    ypos + bdfChar52.height + 2, theme->bg);
                    lcdSetFontDirection(&dev, 0);
                    lcdDrawString2(&dev, &file_data_big, &bdfChar52, xpos, ypos, ascii, theme->color);

                    memset(ascii, 0, sizeof(ascii));
                    sprintf((char *) ascii, "%s", "H");
                    lcdDrawString2(&dev, &file_data_small, &bdfChar12, xpos + (2 * bdfChar52.width) + 5,
                                   ypos + bdfChar52.height - bdfChar12.height, ascii, theme->color);

                    memset(ascii, 0, sizeof(ascii));
                    if (am)
                        sprintf((char *) ascii, "%s", "AM");
                    else
                        sprintf((char *) ascii, "%s", "PM");
                    lcdDrawFillRect(&dev, xpos, ypos + (3 * bdfChar52.height) + 10, xpos + (2 * bdfChar32.width),
                                    ypos + (4 * bdfChar52.height) + 12, theme->bg);
                    lcdDrawString2(&dev, &file_data, &bdfChar32, xpos, ypos + (3 * bdfChar52.height) + 10, ascii,
                                   theme->color);

                    memcpy(&lastHour, &currentTime,
                           sizeof(struct tm)); // Update lastTime with the current time after updating the display
                }

                if (currentTime.tm_min != lastMinute.tm_min || needs_update) {
                    ypos = base_ypos + (1 * (bdfChar52.height + 2));

                    memset(ascii, 0, sizeof(ascii));

                    if (currentTime.tm_min < 10) {
                        sprintf((char *) ascii, "0%d", currentTime.tm_min);
                    } else {
                        sprintf((char *) ascii, "%d", currentTime.tm_min);
                    }

                    // Clear BG
                    lcdDrawFillRect(&dev, xpos - 1, ypos - 1, xpos + (strlen((char *) ascii) * bdfChar52.width) + 1,
                                    ypos + bdfChar52.height + 2, theme->bg);
                    lcdSetFontDirection(&dev, 0);
                    lcdDrawString2(&dev, &file_data_big, &bdfChar52, xpos, ypos, ascii, theme->color);

                    memset(ascii, 0, sizeof(ascii));
                    sprintf((char *) ascii, "%s", "M");

//                lcdDrawFillRect(&dev, xpos + (2 * bdfChar32.width) + 3,
//                                ypos + bdfChar32.height - bdfChar12.height,
//                                xpos + (2 * bdfChar32.width) + bdfChar12.width + 6,
//                                ypos + bdfChar32.height + 2, theme->bg);
                    lcdDrawString2(&dev, &file_data_small, &bdfChar12, xpos + (2 * bdfChar52.width) + 5,
                                   ypos + bdfChar52.height - bdfChar12.height, ascii, theme->color);


                    memcpy(&lastMinute, &currentTime,
                           sizeof(struct tm)); // Update lastTime with the current time after updating the display
                }

                if (lastSecond.tm_sec != currentTime.tm_sec || needs_update) {
                    ypos = base_ypos + (2 * (bdfChar52.height + 2));

                    memset(ascii, 0, sizeof(ascii));

                    if (currentTime.tm_sec < 10) {
                        sprintf((char *) ascii, "0%d", currentTime.tm_sec);
                    } else {
                        sprintf((char *) ascii, "%d", currentTime.tm_sec);
                    }

                    // Clear BG
                    lcdDrawFillRect(&dev, xpos - 1, ypos - 1, xpos + (strlen((char *) ascii) * bdfChar52.width) + 1,
                                    ypos + bdfChar52.height + 2, theme->bg);
                    lcdSetFontDirection(&dev, 0);
                    lcdDrawString2(&dev, &file_data_big, &bdfChar52, xpos, ypos, ascii, theme->color);

                    memset(ascii, 0, sizeof(ascii));
                    sprintf((char *) ascii, "%s", "S");

                    lcdDrawString2(&dev, &file_data_small, &bdfChar12, xpos + (2 * bdfChar52.width) + 5,
                                   ypos + bdfChar52.height - bdfChar12.height, ascii, theme->color);

                    memcpy(&lastSecond, &currentTime, sizeof(struct tm));
                }

                if (needs_update) {
                    needs_update = false;
                }
            }

            icm42670_data_t recImuData;

            if (xQueueReceive(ICMQueue, &recImuData, 0)) {


                // GYROSCOPE DATA TELLS YOU ANGULAR MOMENTUM (WHAT DIRECTION YOU ROTATE IN)

                uint16_t angX = ((int) (convert_to_degrees(recImuData.accel_x) / 1.5)) + 180;
                uint16_t angY = ((int) (convert_to_degrees(recImuData.accel_y) * 1.25)) + 180;


                lcdDrawFillCircle(&dev, centerX2, centerY2, 45, theme->bg);

                lcdDrawFillRectAngle(&dev, centerX2, centerY2, 60, 60, 0, 0, angX + 45, theme->color);
                lcdDrawFillRectAngle(&dev, centerX2, centerY2, 60, 60, 0, 0, angX, theme->color);
                lcdDrawFillCircle(&dev, centerX2, centerY2, 34, theme->bg);
                lcdDrawCircle(&dev, centerX2, centerY2, 34, theme->color);
                lcdDrawRectAngle(&dev, centerX2, centerY2, 48, 48, angX, theme->color);
                lcdDrawRectAngle(&dev, centerX2, centerY2, 48, 48, angX + 45, theme->color);
                lcdDrawRegularPolygon(&dev, centerX2, centerY2, 8, 42, angX, theme->color);

                lcdDrawTriangleAtAngle(&dev, 42, 0, 0, -15, 0, 15, centerX2, centerY2, angX, WHITE);
                lcdDrawTriangleAtAngle(&dev, -42, 0, 0, -15, 0, 15, centerX2, centerY2, angX, WHITE);


                // inner
                lcdDrawFillCircle(&dev, centerX, centerY, 37, theme->bg);

                lcdDrawRectAngle(&dev, centerX, centerY, 45, 45, angY, theme->color);
                lcdDrawRectAngle(&dev, centerX, centerY, 45, 45, angY + 45, theme->color);
                lcdDrawRegularPolygon(&dev, centerX, centerY, 8, 32, angY, theme->color);

                lcdDrawTriangleAtAngle(&dev, 32, 0, 0, -10, 0, 10, centerX, centerY, angY + 90, WHITE);
                lcdDrawTriangleAtAngle(&dev, -32, 0, 0, -10, 0, 10, centerX, centerY, angY + 90, WHITE);
            }

            float volts;
            if (xQueueReceive(MCPQueue, &volts, 0)) {
                float percentage = convert_voltage_to_percentage(volts);
                int x_bat = 24;
                int y_bat = 232;

                memset(ascii, 0, sizeof(ascii));
                sprintf((char *) ascii, "%.1f%%", percentage);
//                ESP_LOGI("Mainloop MCP", "percent %f", percentage);

                lcdDrawFillRect(&dev, x_bat, y_bat - 3, x_bat + (4 * bdfChar12.width) + 14,
                                y_bat + (1 * bdfChar12.height) + 10, theme->bg);
                lcdDrawString2(&dev, &file_data_small, &bdfChar12, x_bat, y_bat - 3, ascii, theme->color);

                // redraw background of bar
                lcdDrawFillRect(&dev, x_bat - 16, y_bat - 40, x_bat - 4, y_bat + bdfChar12.height, theme->bg);

                // outer shell of bar
                lcdDrawRoundRect(&dev, x_bat - 4, y_bat + bdfChar12.height, x_bat - 16, y_bat - 40, 3, theme->color);

                // bar (fills 36 + bdf char height pixels pixels)
                lcdDrawFillRect(&dev, x_bat - 14,
                                (y_bat + bdfChar12.height - 2) - (int) ((36 + bdfChar12.height) * (percentage / 100)),
                                x_bat - 6, y_bat + bdfChar12.height - 2, theme->color);



                // wow this fucking blows btw, you have to change both 2nd and 3rd params to correctly modify font size
                lcdDrawFinish(&dev);
            }
            lcdDrawFinish(&dev);

        }

    }

    // never reach here
    while (1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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

#if ENABLE_MAIN
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        ESP_LOGE("main", "Failed to create mutex");
        return; // Handle error appropriately
    }

    timeQueue = xQueueCreate(5, sizeof(struct tm));
    ICMQueue = xQueueCreate(10, sizeof(icm42670_data_t));
    MCPQueue = xQueueCreate(5, sizeof(float));

    ESP_ERROR_CHECK(i2cdev_init()); // Initialize I2C



    // I SWITCHED THE ORDER BETWEEN THESE 2 AND ST7789, MIGHT CAUSE PROBLEMS
    xTaskCreate(ds3231, "getDS3231", configMINIMAL_STACK_SIZE * 6, NULL, 5, &getDS3231Handle);
    xTaskCreatePinnedToCore(icm42670, "getICM42670", configMINIMAL_STACK_SIZE * 16, NULL, 5, &getICM42670Handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(getMCP3427, "getMCP3427", configMINIMAL_STACK_SIZE * 8, NULL, 5, &getMCP3427Handle, APP_CPU_NUM);
#endif

    xTaskCreate(ST7789, "ST7789", 1024*6, NULL, 2, NULL);
}
