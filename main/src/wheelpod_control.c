#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "wheelpod_control.h"
#include "motor_control.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// Stop VSCode from complaining
#ifndef pdFALSE
#define pdFALSE 0
#endif

Wheelpod* createWheelpod(Motor* motor1, Motor* motor2, int gpio_sensor) {
    Wheelpod* wheelpod = (Wheelpod*) malloc(sizeof(Wheelpod));
    wheelpod->motor1 = motor1;
    wheelpod->motor2 = motor2;
    wheelpod->gpio_sensor = gpio_sensor;
    wheelpod->message_queue = xQueueCreate(5, sizeof(WheelpodCommand));
    xTaskCreate(vRunWheelpod, "wheelpod_run", 4096, wheelpod, PRIORITY_WHEELPOD, 
                &wheelpod->control_task_handle);
    return wheelpod;
}

/**
 * Task to run wheelpod PID and speed loops
 */
void vRunWheelpod(void * arg) {
    Wheelpod* wheelpod = (Wheelpod*) arg;

    float wheelSpeed = 0;
    float wheelAngleTarget = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        // Receive messages
        WheelpodCommand cmd;
        if (xQueueReceive(wheelpod->message_queue, &cmd, 0) == pdTRUE) {
            wheelSpeed = cmd.wheelSpeed;
            wheelAngleTarget = cmd.wheelAngle; 
        }

        // Todo smallest angle and speed reverse

        // Speed recombination
        float wheelAngle = 0; // TODO read from position sensor
        float error = wheelAngleTarget - wheelAngle;
        float dtheta_speed = kP_WHEELPOD * error; // Error P loop

        float m1 = (dtheta_speed + wheelSpeed / (M_PI * WHEEL_DIA * G2_WHEELPOD * kV_MOTOR)) / (2 * G1_WHEELPOD);
        float m2 = (dtheta_speed - wheelSpeed / (M_PI * WHEEL_DIA * G2_WHEELPOD * kV_MOTOR)) / (2 * G1_WHEELPOD);

        setMotorSpeed(wheelpod->motor1, m1 / SYSTEM_VOLTAGE);
        setMotorSpeed(wheelpod->motor2, m2 / SYSTEM_VOLTAGE);

        vTaskDelayUntil( &xLastWakeTime, PERIOD_WHEELPOD );
    }
}