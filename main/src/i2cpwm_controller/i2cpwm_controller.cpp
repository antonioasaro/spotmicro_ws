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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <pca9685.h>

#include "i2cpwm_board/msg/servo.h"
#include "i2cpwm_controller/i2cpwm_controller.h"

#define ADDR PCA9685_ADDR_BASE
#define SDA_GPIO gpio_num_t(4)
#define SCL_GPIO gpio_num_t(5)
#define RELAY_GPIO gpio_num_t(18)
#define PWM_FREQ_HZ 50 // (20ms / 4096 ticks) == 4.88 us
#define POSITION_UNDEFINED 0

static const i2c_port_t I2CPORT = 1;
static const char *TAG = "i2cpwm_controller";

i2c_dev_t dev;
servo_config _servo_configs[12];
TaskHandle_t xHandle;
bool calibrating = false;

void i2cpwm_controller_init()
{
    uint16_t freq;

    ESP_LOGI(TAG, "Initialize I2C_PORT_1");
    memset(&dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, I2CPORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&dev));
    ESP_ERROR_CHECK(pca9685_restart(&dev));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));
    for (uint32_t i = 0; i < 16; i++)
    {
        if (pca9685_set_pwm_value(&dev, i, 0) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", i);
    }
    usleep(1000);
    gpio_set_level(RELAY_GPIO, 0);
}

#define SERVO_MED 306
#define SERVO_RANGE 360 // 80 or 380
#define SERVO_MIN (SERVO_MED - (SERVO_RANGE / 2))
#define SERVO_MAX (SERVO_MED + (SERVO_RANGE / 2))
void servo_calibration_task(void *pvParameters)
{
    uint16_t chan = 0;
    uint16_t val = SERVO_MED;
    uint16_t dir = 2;
    bool once = false;
    while (calibrating)
    {
        //// ESP_LOGI("servo_calibration_task", "CH%d = %-4d", chan, val);
        if (pca9685_set_pwm_value(&dev, chan, val) != ESP_OK)
            ESP_LOGE("servo_calibration_task", "Could not set PWM value to ch0");
        if (val == SERVO_MED)
        {
            ESP_LOGI(TAG, "Servo calibration ch%d at SERVO_MED", chan);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            if (once)
                chan = (chan + 1) % 3;
            once = !once;
        }
        val = val + dir;
        if (val >= SERVO_MAX)
            dir = -2;
        else
        {
            if (val <= SERVO_MIN)
                dir = 2;
        }
        vTaskDelay(5);
    }
}

void i2cpwm_controller_calibration_enable()
{
    if (calibrating == false)
    {
        ESP_LOGI(TAG, "Enable calibration");
        xTaskCreatePinnedToCore(servo_calibration_task, "servo_calibration", configMINIMAL_STACK_SIZE * 3, NULL, 5, &xHandle, APP_CPU_NUM);
        calibrating = true;
    }
}

void i2cpwm_controller_calibration_disable()
{
        ESP_LOGI(TAG, "Disable calibration");
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
    }
    calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////////
// the public API is ONE based and hardware is ZERO based
///////////////////////////////////////////////////////////////////////////////////
void i2cpwm_controller_config_servo(int servo, int center, int range, int direction)
{
    _servo_configs[servo - 1].center = center;
    _servo_configs[servo - 1].range = range;
    _servo_configs[servo - 1].direction = direction;
    _servo_configs[servo - 1].mode_pos = POSITION_UNDEFINED;
    if (servo == 1)
        ESP_LOGI(TAG, "Servo #%d configured with center=%d, range=%d, and direction=%d", servo, center, range, direction);
}

void i2cpwm_controller_servos_absolute(i2cpwm_board__msg__ServoArray *msg)
{
    int servo;
    int value;

    //// ESP_LOGI(TAG, "i2cpwm_controller_servos_absolute()");
    for (int i = 1; i <= 12; i++)
    {
        servo = msg->servos.data[i - 1].servo;
        value = msg->servos.data[i - 1].value;
        if ((value < 0) || (value > 4096))
        {
            ESP_LOGE(TAG, "Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
            continue;
        }
        if (pca9685_set_pwm_value(&dev, servo - 1, value) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", servo - 1);
        // if (servo == 1)
        //     ESP_LOGI(TAG, "servo[%d] = %d", servo, value);
    }
}

void i2cpwm_controller_servos_proportional(i2cpwm_board__msg__ServoArray *msg)
{

    //// ESP_LOGI(TAG, "i2cpwm_controller_servos_proportyional()");
    int servo;
    float value;

    for (int i = 1; i <= 12; i++)
    {
        servo = msg->servos.data[i - 1].servo;
        value = msg->servos.data[i - 1].value;
        servo_config *configp = &(_servo_configs[servo - 1]);

        int pos = (configp->direction * (((float)(configp->range) / 2) * value)) + configp->center;
        if ((pos < 0) || (pos > 4096))
        {
            ESP_LOGE(TAG, "Invalid computed position servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
            return;
        }
        if (pca9685_set_pwm_value(&dev, servo - 1, pos) != ESP_OK)
            ESP_LOGE(TAG, "Could not set PWM value on ch%d", servo - 1);
        // if (servo == 1)
        //     ESP_LOGI(TAG, "servo[%d] = (direction(%d) * ((range(%d) / 2) * value(%6.4f))) + %d = %d", servo, configp->direction, configp->range, value, configp->center, pos);
    }
}
