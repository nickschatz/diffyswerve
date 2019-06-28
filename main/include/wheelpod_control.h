#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "motor_control.h"
#include "encoder_i2c.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define PRIORITY_WHEELPOD 5
#define PERIOD_WHEELPOD 10 // Wheelpod control loop period in ticks (1 tick = 1 ms)
#define kP_WHEELPOD 1800 // RPM / rad error
#define kI_WHEELPOD 12 // RPM / rad*ms error
#define IZONE_WHEELPOD 20

#define kV_MOTOR 2250 // RPM / V
#define G1_WHEELPOD (6./80.)
#define G2_WHEELPOD (17./26.)
#define WHEEL_DIA 2.375
#define SYSTEM_VOLTAGE 7.2

#define ANGLE_MAX (M_PI)

typedef struct {
    Motor* motor1;
    Motor* motor2;
    TaskHandle_t control_task_handle;
    QueueHandle_t message_queue;
    float xpos;
    float ypos;
    Encoder encoder;
    float angle_offset;
    float angle;
} Wheelpod;

typedef struct {
    float wheelSpeed;
    float wheelAngle;
} WheelpodCommand;

Wheelpod* createWheelpod(Motor* motor1, Motor* motor2, Encoder encoder, float angle_offset);

void vRunWheelpod(void* args);

float signAngle(float sign, float angle);

bool within90(float angle1, float angle2);

float getAngle(Wheelpod* wheelpod);

void setWheelpod(Wheelpod* wheelpod, float speed, float angle);