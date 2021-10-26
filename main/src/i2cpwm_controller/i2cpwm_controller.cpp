#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
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
#define RELAY_GPIO gpio_num_t(18)
#define PWM_FREQ_HZ 50
#define SERVO_MIN 200 // (20ms / 4096 ticks) == 4.88 us * 200
#define SERVO_MAX 400 // 4.88 us * 400
#define POSITION_UNDEFINED 0

static const i2c_port_t I2CPORT = 1;
static const char *TAG = "i2cpwm_controller";

i2c_dev_t dev;
servo_config _servo_configs[12];

void i2cpwm_controller_init()
{
    uint16_t freq;
    uint16_t value = SERVO_MIN;

    ESP_LOGI(TAG, "i2cpwm_controller_init()");
    memset(&dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, I2CPORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&dev));
    ESP_ERROR_CHECK(pca9685_restart(&dev));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));
    for (uint32_t i = 0; i < 16; i++)
    {
        if (pca9685_set_pwm_value(&dev, 0, value) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", i);
    }
    usleep(1000);
    gpio_set_level(RELAY_GPIO, 0);
}

void i2cpwm_controller_config_servo(int servo, int center, int range, int direction)
{
    _servo_configs[servo - 1].center = center;
    _servo_configs[servo - 1].range = range;
    _servo_configs[servo - 1].direction = direction;
    _servo_configs[servo - 1].mode_pos = POSITION_UNDEFINED;
    ESP_LOGI(TAG, "Servo #%d configured with center=%d, range=%d, and direction=%d", servo, center, range, direction);
}

void i2cpwm_controller_servos_absolute(i2cpwm_board__msg__ServoArray *msg)
{
    int servo;
    int value;

    //// ESP_LOGI(TAG, "i2cpwm_controller_servos_absolute()");
    for (int i = 0; i < 12; i++)
    {
        servo = msg->servos.data[0].servo;
        value = msg->servos.data[0].value;

        if ((value < 0) || (value > 4096))
        {
            ESP_LOGE(TAG, "Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
            continue;
        }
        if (pca9685_set_pwm_value(&dev, servo, value) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", servo);
    }
}

void i2cpwm_controller_servos_proportional(i2cpwm_board__msg__ServoArray *msg)
{
    int value;

    printf("Antonio - i2cpwm_controller_servos_proportional: %d %d %d %f %d %f\n",
           msg->servos.size, msg->servos.capacity,
           msg->servos.data[0].servo, msg->servos.data[0].value,
           msg->servos.data[1].servo, msg->servos.data[1].value);

    value = SERVO_MAX;
    //// ESP_LOGI(TAG, "i2cpwm_controller_servos_proportyional()");
    if (pca9685_set_pwm_value(&dev, 0, value) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value on ch0");
}
