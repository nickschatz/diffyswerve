#pragma once

#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

typedef struct {
    int gpio_dir;
    int unit;
    int timer;
    int operator;
} Motor;

void init_unit(int unit, int timer);
Motor* init_motor(int gpio_pwm, int gpio_dir);
void set_motor_speed(Motor* motor, float speed);