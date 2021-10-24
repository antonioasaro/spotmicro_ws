#ifndef I2CPWM_CONTROLLER //used for conditional compiling.
#define I2CPWM_CONTROLLER

extern void i2cpwm_controller(void);
#include "i2cpwm_board/msg/servo.h"
#include "i2cpwm_board/msg/servo_config.h"

// typedef struct _servo_config
// {
//     int center;
//     int range;
//     int direction;
//     int mode_pos;
// } servo_config;

// struct Servo
// {
//     int16_t servo;
//     float value;
// };

struct ServoArray
{
    i2cpwm_board__msg__Servo servos[12];
};

// struct ServoConfig
// {
//     int16_t servo;
//     int16_t center;
//     int16_t range;
//     int16_t direction;
// };

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
