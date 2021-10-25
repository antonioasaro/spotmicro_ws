#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include "esp_log.h"
#include "esp_system.h"
#include <pca9685.h>

#include "i2cpwm_board/msg/servo.h"
#include "i2cpwm_controller/i2cpwm_controller.h"

#define ADDR PCA9685_ADDR_BASE
#define SDA_GPIO gpio_num_t(4)
#define SCL_GPIO gpio_num_t(5)
#define PWM_FREQ_HZ 50
#define SERVO_MIN 200 // (20ms / 4096 ticks) == 4.88 us * 200
#define SERVO_MAX 400 // 4.88 us * 400

static i2c_dev_t dev;
static i2c_port_t I2CPORT = 1;
static const char *TAG = "i2cpwm_controller";

void i2cpwm_controller_init()
{
    uint16_t freq;
    uint16_t val = SERVO_MIN;

    ESP_LOGI(TAG, "i2cpwm_controller_init()");
    memset(&dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, I2CPORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&dev));
    ESP_ERROR_CHECK(pca9685_restart(&dev));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));
    for (uint32_t i = 0; i < 16; i++)
    {
        if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", i);
    }
}

void i2cpwm_controller_servos_absolute()
{
    uint16_t val;

    val = SERVO_MIN;
    ESP_LOGI(TAG, "i2cpwm_controller_servos_absolute()");
    if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value on ch0");
}

void i2cpwm_controller_servos_proportional()
{
    uint16_t val;

    val = SERVO_MAX;
    ESP_LOGI(TAG, "i2cpwm_controller_servos_proportyional()");
    if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value on ch0");
}
