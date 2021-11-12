#ifndef I2CPWM_CONTROLLER //used for conditional compiling.
#define I2CPWM_CONTROLLER

#include "i2cpwm_board/msg/servo.h"
#include "i2cpwm_board/msg/servo_array.h"
#include "i2cpwm_board/msg/servo_config.h"

extern void i2cpwm_controller_init(void);
extern void i2cpwm_controller_servos_absolute(i2cpwm_board__msg__ServoArray *msg);
extern void i2cpwm_controller_servos_proportional(i2cpwm_board__msg__ServoArray *msg);
extern void i2cpwm_controller_config_servo(int servo, int center, int range, int direction);
extern void i2cpwm_controller_calibration_enable();
extern void i2cpwm_controller_calibration_disable();

struct ServoArray
{
    i2cpwm_board__msg__Servo servos[12];
};

typedef struct _servo_config
{
    int center;
    int range;
    int direction;
    int mode_pos;
} servo_config;

#endif
