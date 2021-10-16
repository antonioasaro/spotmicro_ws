#define TEST_KINEMATICS

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

#ifdef TEST_KINEMATICS
#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
SpotMicroMotionCmd *motion;
#endif

#ifdef OTA_UPDATES
#define FIRMWARE_VERSION 1.61
#define UPDATE_JSON_URL "https://192.168.1.177:8070/spotmicro.json"
#include "https_ota.h"
#endif

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

#define LED_GPIO gpio_num_t(2)
#define BUTTON_GPIO gpio_num_t(35)
#define STRING_BUFFER_LEN 256
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
#define PWM_FREQ_HZ 50
#define SERVO_MIN 200 // (20ms / 4096 ticks) == 4.88 us * 200
#define SERVO_MAX 400 // 4.88 us * 400

rcl_publisher_t button_publisher;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t joy_subscriber;

static const i2c_port_t I2CPORT = 1;
std_msgs__msg__Header outgoing_button;
geometry_msgs__msg__Twist cmd_vel;
sensor_msgs__msg__Joy joy;
int32_t seq_no = 0;
int32_t angle = 0;
char all_stats[256];
uint32_t buttonA;
uint32_t buttonB;
uint32_t buttonX;
uint32_t buttonY;

void publish_stats_task(void *pvParameters)
{

	while (1)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		sprintf(all_stats, "Current stats:  \n\n  mpu_x = %hu\n  mpu_y = %hu\n  mpu_z = %hu\n\n A=%u B=%u X=%u Y=%u\n",
				accel_x, accel_y, accel_z,
				buttonA, buttonB, buttonX, buttonY);
		xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_text_task", 2048,
					(void *)all_stats, 6, NULL);
	}
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		gpio_set_level(LED_GPIO, seq_no++ & 0x1);
		sprintf(outgoing_button.frame_id.data, ": %d", gpio_get_level(BUTTON_GPIO));
		outgoing_button.frame_id.size = strlen(outgoing_button.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outgoing_button.stamp.sec = ts.tv_sec;
		outgoing_button.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the button message
		rcl_publish(&button_publisher, (const void *)&outgoing_button, NULL);
		////		printf("Button send seq %s\n", outgoing_button.frame_id.data);
	}
};

void cmd_vel_subscription_callback(const void *msgin)
{

	geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)msgin;
	printf("Received keyboard: x = %f y = %f z = %f\n", msg->linear.x, msg->linear.y, msg->linear.z);
	//printf("Received angular: x==%f y==%f z==%f\n", msg->angular.x, msg->angular.y, msg->angular.z);
}

void joy_subscription_callback(const void *msgin)
{

	sensor_msgs__msg__Joy *msg = (sensor_msgs__msg__Joy *)msgin;
	buttonA = (uint32_t)msg->buttons.data[0];
	buttonB = (uint32_t)msg->buttons.data[1];
	buttonX = (uint32_t)msg->buttons.data[2];
	buttonY = (uint32_t)msg->buttons.data[3];
	//printf("Received joystick sec = %d\n", msg->header.stamp.sec);
	//printf("Received joystick button A = %d\n", (uint32_t) msg->buttons.data[0]);
}

void micro_ros_task(void *arg)
{

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

	// Create button publisher
	printf("Create publisher\n");
	RCCHECK(rclc_publisher_init_default(
		&button_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"/button"));

	// Create cmd_vel subscriber
	printf("Create cmd_vel subscriber\n");
	RCCHECK(rclc_subscription_init_default(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel"));

	// Create joy subscriber
	printf("Create joy subscriber\n");
	RCCHECK(rclc_subscription_init_default(
		&joy_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
		"/joy"));

	// Create timer
	printf("Create timer\n");
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Initialize outgoing_button
	printf("Initialize outgoing_button\n");
	char outgoing_button_buffer[STRING_BUFFER_LEN];
	outgoing_button.frame_id.data = outgoing_button_buffer;
	outgoing_button.frame_id.capacity = STRING_BUFFER_LEN;

	// Create executor
	printf("Create executors\n");
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, &cmd_vel_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &joy_subscriber, &joy, &joy_subscription_callback, ON_NEW_DATA));

	float_t joy_axis[4];
	int32_t joy_button[16];
	joy.axes.capacity = 4;
	joy.buttons.capacity = 16;
	joy.axes.data = joy_axis;
	joy.buttons.data = joy_button;

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&button_publisher, &node));
	RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&joy_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

void blink_task(void *pvParameters)
{

	while (1)
	{
		for (int j = 0; j < N_COLORS; j++)
		{
			printf("Blink LEDs\n");

			for (int i = 0; i < NUM_LEDS; i++)
			{
				leds[i] = colors[j];
			}
			FastLED.show();
			vTaskDelay(200 / portTICK_PERIOD_MS);
		};
	}
};

#define N_COLORS_CHASE 5
CRGB colors_chase[N_COLORS_CHASE] = {
	CRGB::Red,
	CRGB::Green,
	CRGB::Blue,
	CRGB::DarkOrange,
	CRGB::White};

void chase_task(void *pvParameters)
{
	int pos = 0;
	int led_color = 0;
	while (1)
	{
		// do it the dumb way - blank the leds
		for (int i = 0; i < NUM_LEDS; i++)
		{
			leds[i] = CRGB::Black;
		}

		// set the one LED to the right color
		leds[pos] = colors_chase[led_color];
		pos = (pos + 1) % NUM_LEDS;

		// use a new color
		if (pos == 0)
		{
			led_color = (led_color + 1) % N_COLORS_CHASE;
		}

		// uint64_t start = esp_timer_get_time();
		FastLED.show();
		// uint64_t end = esp_timer_get_time();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	};
}

void pca9685_task(void *pvParameters)
{
	i2c_dev_t dev;
	uint16_t freq;
	uint16_t val = SERVO_MIN;
	uint16_t dir = 2;

	memset(&dev, 0, sizeof(i2c_dev_t));
	ESP_ERROR_CHECK(pca9685_init_desc(&dev, ADDR, I2CPORT, SDA_GPIO, SCL_GPIO));
	ESP_ERROR_CHECK(pca9685_init(&dev));
	ESP_ERROR_CHECK(pca9685_restart(&dev));
	ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ));
	ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&dev, &freq));
	ESP_LOGI("pca9685_task", "Freq %dHz, real %d", PWM_FREQ_HZ, freq);
	for (uint32_t i = 0; i < 16; i++)
	{
		if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
			ESP_LOGE("pca9685_task", "Could not set PWM value to ch%d", i);
	}
	gpio_set_level(RELAY_GPIO, 0);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	while (1)
	{
		// ESP_LOGI("pca9685_task", "CH0 = %-4d", val);
		if (pca9685_set_pwm_value(&dev, 0, val) != ESP_OK)
			ESP_LOGE("pca9685_task", "Could not set PWM value to ch0");
		if (val == 300)
		{
			vTaskDelay(250 / portTICK_PERIOD_MS);
		}

		val = val + dir;
		if (val >= SERVO_MAX)
		{
			dir = -2;
		}
		else
		{
			if (val <= SERVO_MIN)
			{
				dir = 2;
			}
		}
		vTaskDelay(4);
	}
}

void app_gpios(void)
{

	// Initialize GPIOs
	printf("Initialize GPIOs\n");
	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_reset_pin(RELAY_GPIO);
	gpio_set_level(RELAY_GPIO, 0);
	gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(BUTTON_GPIO);
	gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}

void spotmicro_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		gpio_set_level(LED_GPIO, seq_no++ & 0x1);
		motion->runOnce();
		// node.runOnce();
		// ros::spinOnce();
		// rate.sleep()
	}
};

extern "C" void app_main(void)
{

	app_gpios();

#ifdef UCLIENT_PROFILE_UDP
	// Start the networking if required
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif // UCLIENT_PROFILE_UDP

#ifdef OTA_UPDATES
	// Check for OTA firmware update
	https_ota();
#endif

	// Initializing OLED ...
	printf("Initialize OLED\n");
	ssd1306();
	xTaskCreate(&ssd1306_clear_task, "ssd1306_clear_task", 2048, NULL, 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_display_text", 2048,
				(void *)"Initializing ...", 1, NULL);
	vTaskDelay(500 / portTICK_PERIOD_MS);

#ifdef TEST_KINEMATICS
#define TOTAL_EXECUTORS 2
	printf("Testing Kinematics!!\n");
	xTaskCreate((TaskFunction_t)&ssd1306_text_task, "ssd1306_display_text", 2048,
				(void *)"Kinematics!!!   ", 1, NULL);
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

	// Create timer
	printf("Create spotmicro_timer\n");
	rcl_timer_t spotmicro_timer;
	const unsigned int spotmicro_timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&spotmicro_timer,
		&support,
		RCL_MS_TO_NS(spotmicro_timer_timeout),
		spotmicro_timer_callback));

	// Create executor
	printf("Create spotmicro_executors\n");
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, TOTAL_EXECUTORS, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &spotmicro_timer));

	// Initialize spot micro kinematics object of this class
	motion = new SpotMicroMotionCmd(node, executor);

	// Only proceed if servo configuration publishing succeeds
	if (motion->publishServoConfiguration())
	{
		float rate = (1.0 / motion->getNodeConfig().dt); // Defing the looping rate

		struct timespec ts_begin;
		struct timespec ts_end;
		while (1)
		{
			clock_gettime(CLOCK_REALTIME, &ts_begin);
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			usleep((1000 / rate) * 1000);
			clock_gettime(CLOCK_REALTIME, &ts_end);
			// printf("Delta time is: %.02fms\n", ((float) (ts_end.tv_nsec - ts_begin.tv_nsec)) / (1000 * 1000));
		}
	}
#else

	// Initializing mpu6050
	printf("Initialize MPU6050\n");
	mpu6050();
	xTaskCreate(&publish_stats_task, "publish_stats_task", 2048, NULL, 1, NULL);
	xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 6, NULL);

	// Start LEDs
	vTaskDelay(100 / portTICK_PERIOD_MS);
	FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
	FastLED.setMaxPowerInVoltsAndMilliamps(5, 1000);
	//xTaskCreatePinnedToCore(&blink_task, "blink_task", 4000, NULL, 5, NULL, APP_CPU_NUM);
	xTaskCreatePinnedToCore(&chase_task, "chase_task", 4000, NULL, 5, NULL, APP_CPU_NUM);

	// Start SERVOs
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(i2cdev_init());
	xTaskCreatePinnedToCore(pca9685_task, "pca9685_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);

	// Pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task,
				"micro_task",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);
#endif
}
