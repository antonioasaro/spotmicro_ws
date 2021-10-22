extern void i2cpwm_controller(void);

typedef struct _servo_config {
    int center;
    int range;
    int direction;
    int mode_pos;
} servo_config;
