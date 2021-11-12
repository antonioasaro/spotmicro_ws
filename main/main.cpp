#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>
#include <pca9685.h>

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "ssd1306.h"
#include "mpu6050.h"
#include "FastLED.h"
#include "FX.h"

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

#define LED_GPIO gpio_num_t(2)
#define RELAY_GPIO gpio_num_t(18)

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

#define ADDR PCA9685_ADDR_BASE
//// #define DEFAULT_PINS
#ifdef DEFAULT_PINS
#define SDA_GPIO 21
#define SCL_GPIO 22
#else
#define SDA_GPIO gpio_num_t(4)
#define SCL_GPIO gpio_num_t(5)
#endif
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "main";
static const i2c_port_t I2CPORT = 1;
SpotMicroMotionCmd *motion;

CRGBPalette16 currentPalette;
TBlendType currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 IRAM_ATTR myRedWhiteBluePalette_p;
#include "palettes.h"

//#define NUM_LEDS 512
#define NUM_LEDS 15
#define DATA_PIN 17
#define LED_TYPE WS2812B

CRGB leds[NUM_LEDS];

#define N_COLORS 17
static const CRGB colors[N_COLORS] = {
	CRGB::Red,
	CRGB::Green,
	CRGB::Blue,
	CRGB::White,
	CRGB::AliceBlue,
	CRGB::ForestGreen,
	CRGB::Lavender,
	CRGB::MistyRose,
	CRGB::DarkOrchid,
	CRGB::DarkOrange,
	CRGB::Black,
	CRGB::Teal,
	CRGB::Violet,
	CRGB::Lime,
	CRGB::Chartreuse,
	CRGB::BlueViolet,
	CRGB::Aqua};

#define N_COLORS_CHASE 5
CRGB colors_chase[N_COLORS_CHASE] = {
	CRGB::Red,
	CRGB::Green,
	CRGB::Blue,
	CRGB::DarkOrange,
	CRGB::White};


void blink_task(void *pvParameters)
{

	while (1)
	{
		for (int j = 0; j < N_COLORS; j++)
		{
			for (int i = 0; i < NUM_LEDS; i++)
			{
				leds[i] = colors[j];
			}
			FastLED.show();
			vTaskDelay(500 / portTICK_PERIOD_MS);
		};
	}
};

void spotmicro_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	static int8_t seq_no = 0;

	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		gpio_set_level(LED_GPIO, seq_no++ & 0x1);
	}
};

void gpios_init(void)
{

	// Initialize GPIOs
	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_reset_pin(RELAY_GPIO);
	gpio_set_level(RELAY_GPIO, 1);
	gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
}

extern "C" void app_main(void)
{

	// Initializing GPIO pins ...
	ESP_LOGI(TAG, "////////////////////");
	ESP_LOGI(TAG, "Initialize GPIO pins");
	gpios_init();

	// Initializing i2cdev
	ESP_LOGI(TAG, "Initialize i2cdev");
	ESP_ERROR_CHECK(i2cdev_init());
	vTaskDelay(500 / portTICK_PERIOD_MS);

	// Initializing i2c_port_0 ...
	ESP_LOGI(TAG, "Initialize I2C_PORT_0");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	vTaskDelay(500 / portTICK_PERIOD_MS);

	// Initializing OLED ...
	ESP_LOGI(TAG, "Initialize OLED");
	ssd1306();
	xTaskCreate((TaskFunction_t)&ssd1306_clear_task, "ssd1306_clear_task", 2048, NULL, 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_display_text", 2048, (void *)"Starting up ... ", 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);

//// #define MPU6050_EN
#ifdef MPU6050_EN
	// Initializing mpu6050
	ESP_LOGI(TAG, "Initialize MPU6050");
	mpu6050();
	xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 6, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

#define WS2812B_EN
#ifdef WS2812B_EN
	// Start LEDs
	ESP_LOGI(TAG, "Initialize WS2812B LED strip");
	FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
	FastLED.setMaxPowerInVoltsAndMilliamps(5, 1000);
	xTaskCreatePinnedToCore(&blink_task, "blink_task", 4000, NULL, 5, NULL, APP_CPU_NUM);
	vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

#ifdef UCLIENT_PROFILE_UDP
	// Start the networking if required
	ESP_ERROR_CHECK(uros_network_interface_initialize());
	vTaskDelay(500 / portTICK_PERIOD_MS);
#endif // UCLIENT_PROFILE_UDP

	// Initialize uros ...
	ESP_LOGI(TAG, "Initialize uros!!");
	xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_display_text", 2048, (void *)"Initialize uros ", 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// Create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "spotmicro_node", "", &support));

	// Create executor
	ESP_LOGI(TAG, "Create spotmicro_executors");
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, SPOT_MICRO_MOTION_CMD_SUBSCRIBERS + 1, &allocator));

	// Initialize spot micro kinematics object of this class
	ESP_LOGI(TAG, "Create SpotMicroMotionCmd object");
	xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_display_text", 2048, (void *)"SpotMicro-ESP32 ", 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	motion = new SpotMicroMotionCmd(node, executor);

	// Create timer
	ESP_LOGI(TAG, "Create spotmicro LED timer");
	rcl_timer_t spotmicro_timer;
	const unsigned int spotmicro_timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(&spotmicro_timer, &support, RCL_MS_TO_NS(spotmicro_timer_timeout), spotmicro_timer_callback));
	RCCHECK(rclc_executor_add_timer(&executor, &spotmicro_timer));

//// #define RATETIME_EN
#ifdef RATETIME_EN
	struct timespec ts_begin;
	struct timespec ts_end;
#endif
	// Only proceed if servo configuration publishing succeeds
	if (motion->publishServoConfiguration())
	{
		float rate = (1.0 / motion->getNodeConfig().dt); // Define the looping rate
		float sleep = (1000 / rate) * 1000 / 20;
		while (1)
		{
#ifdef RATETIME_EN
			clock_gettime(CLOCK_REALTIME, &ts_begin);
#endif
			motion->runOnce();
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
#ifdef RATETIME_EN
			clock_gettime(CLOCK_REALTIME, &ts_end);
			float delta = ((float)(ts_end.tv_nsec - ts_begin.tv_nsec)) / (1000 * 1000);
			if (delta > 33.0)
				printf("Delta time is: %.02fms with sleep of: %.02fms\n", delta, sleep / 1000);
#endif
			usleep(sleep);
		}
	}
}
