#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "motor_control.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define PRIORITY_WHEELPOD 5
#define PERIOD_WHEELPOD 10 // Wheelpod control loop period in ticks (1 tick = 1 ms)
#define kP_WHEELPOD 1 // RPM / degree error

#define kV_MOTOR 2250 // RPM / V
#define G1_WHEELPOD 6/80
#define G2_WHEELPOD 17/26 // todo check this
#define WHEEL_DIA 2.375
#define SYSTEM_VOLTAGE 7.2

#define ANGLE_MAX (M_PI)

typedef struct {
    Motor* motor1;
    Motor* motor2;
    int gpio_sensor;
    TaskHandle_t control_task_handle;
    QueueHandle_t message_queue;
} Wheelpod;

typedef struct {
    float wheelSpeed;
    float wheelAngle;
} WheelpodCommand;

Wheelpod* createWheelpod(Motor* motor1, Motor* motor2, int gpio_sensor);

void vRunWheelpod(void* args);

float signAngle(float sign, float angle);

bool within90(float angle1, float angle2);