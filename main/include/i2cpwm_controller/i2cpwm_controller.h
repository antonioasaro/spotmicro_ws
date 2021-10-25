#ifndef I2CPWM_CONTROLLER //used for conditional compiling.
#define I2CPWM_CONTROLLER

extern void i2cpwm_controller(void);
#include "i2cpwm_board/msg/servo.h"
#include "i2cpwm_board/msg/servo_array.h"
#include "i2cpwm_board/msg/servo_config.h"

struct ServoArray
{
    i2cpwm_board__msg__Servo servos[12];
};

typedef struct
{
    i2cpwm_board__msg__ServoConfig servos[12];

} ServosConfigRequest;

struct ServosConfig
{
    ServosConfigRequest request;
    uint16_t response;
};

#endif
