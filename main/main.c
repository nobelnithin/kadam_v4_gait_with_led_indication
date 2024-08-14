#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <icm42670.h>
#include "esp_timer.h"

static const char *TAG = "icm42670";

#define INT_SOURCE1 0x2C
#define RGB_RED 15

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif
#define ALPHA 0.8 

bool t_detect_flag = true;
static bool fire_on = false;

/* Find gpio definitions in sdkconfig */
float lin_accel_y=0.0;
static esp_timer_handle_t fire_timer;

bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}

void fire_off_callback(void* arg) {
    printf("-----Fire OFF (Timer)----\n");
    gpio_set_level(RGB_RED, 1);
    t_detect_flag = true;
    vTaskDelay(75 / portTICK_PERIOD_MS);
}

void icm42670_test(void *pvParameters)
{
    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(
        icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG, "Temperature reading: %f", temperature);

    int16_t raw_reading;
    uint8_t data_register1;
    uint8_t data_register2;
    uint8_t data_register3;
    uint8_t data_register4;
    uint8_t data_register5;
    uint8_t data_register6;

    /* select which acceleration or gyro value should be read: */
    data_register1 = ICM42670_REG_ACCEL_DATA_X1;
    data_register2= ICM42670_REG_ACCEL_DATA_Y1;
    data_register3= ICM42670_REG_ACCEL_DATA_Z1;
    data_register4= ICM42670_REG_GYRO_DATA_X1;
    data_register5 = ICM42670_REG_GYRO_DATA_Y1;
    data_register6= ICM42670_REG_GYRO_DATA_Z1;
    // ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, INT_SOURCE1, &raw_reading));
    // printf("%d\n", raw_reading);
    int g=0;
    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register4, &raw_reading));

        //ESP_LOGI(TAG, "Raw accelerometer / gyro reading: %d", raw_reading);
        // printf("%d %d %d\n",5000,raw_reading,-5000);
        //float accel_y = raw_reading/2048.0;
        g=0.9*g+0.1*raw_reading;
        lin_accel_y = raw_reading-g;
        // printf("%d %.2f %d\n",2000,lin_accel_y,-2000);
        vTaskDelay(pdMS_TO_TICKS(75));
    }
}
void set_fire(void *params)
{
    vTaskDelay(pdMS_TO_TICKS(10000));//initialization delay :)
    float prev_value = 0.0;
	float curr_value = 0.0;
    float fire_value = 0;

    while(1)
    {
        prev_value = curr_value;
		curr_value = lin_accel_y;
        if(trough_detected(curr_value, prev_value) && lin_accel_y<-500 && t_detect_flag )
        {
           printf("--->->->->->Fire on<-<-<-<-<----\n");
           gpio_set_direction(RGB_RED, GPIO_MODE_OUTPUT);
           gpio_set_level(RGB_RED, 0);
           t_detect_flag = false;
           fire_on = true;
           fire_value = prev_value;
           esp_timer_start_once(fire_timer, 750000);
        }

        if(lin_accel_y > (fire_value+1000) && fire_on)
        {
            printf("-----Fire OFF----\n");
            gpio_set_level(RGB_RED, 1);
            fire_on = false;
            t_detect_flag = true;
            esp_timer_stop(fire_timer);
        }
    

    
    vTaskDelay(pdMS_TO_TICKS(75));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    const esp_timer_create_args_t fire_timer_args = {
        .callback = &fire_off_callback,
        .name = "fire_timer"
    };

    if (esp_timer_create(&fire_timer_args, &fire_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fire timer");
        return;
    }

    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(set_fire, "set fire", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}