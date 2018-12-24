#pragma once

#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

typedef struct {
    int gpio_dir;
    int unit;
    int timer;
    int op;
} Motor;

void initUnit(int unit, int timer);
Motor* createMotor(int gpio_pwm, int gpio_dir);
void setMotorSpeed(Motor* motor, float speed);