#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "wheelpod_control.h"
#include "motor_control.h"
#include <math.h>
#include <stdlib.h>
#include <driver/adc.h>

// Stop VSCode from complaining
#ifndef pdFALSE
#define pdFALSE 0
#endif

Wheelpod* createWheelpod(Motor* motor1, Motor* motor2, Encoder encoder, float angle_offset) {
    Wheelpod* wheelpod = (Wheelpod*) malloc(sizeof(Wheelpod));
    wheelpod->motor1 = motor1;
    wheelpod->motor2 = motor2;
    wheelpod->message_queue = xQueueCreate(5, sizeof(WheelpodCommand));
    wheelpod->encoder = encoder;
    wheelpod->angle_offset = angle_offset;
    encoder_set_hysteresis(encoder, 0b00); // Turn off hysteresis for a continuous 360<->0 transition
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
    float wheelDirection = 1;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        float wheelAngle = getAngle(wheelpod);
        float realAngle = signAngle(wheelDirection, wheelAngle);

        // Receive messages
        WheelpodCommand cmd;
        if (xQueueReceive(wheelpod->message_queue, &cmd, 0) == pdTRUE) {
            wheelSpeed = cmd.wheelSpeed;
            wheelAngleTarget = cmd.wheelAngle; 

            // Reverse-wheel algorithm
            if (!within90(realAngle, wheelAngleTarget)) {
                wheelDirection *= -1;
                realAngle = signAngle(wheelDirection, wheelAngle);
            }
        }

        // Convert to target-angle reference frame
        realAngle -= wheelAngleTarget;
        realAngle = remainderf(realAngle, ANGLE_MAX);        

        wheelpod->angle = realAngle;
        // Speed recombination
        float error = -realAngle;  // Because targetAngle always is zero in target-angle space

        static float integrator = 0;
        integrator += error * PERIOD_WHEELPOD;
        if (fabs(error) > IZONE_WHEELPOD) {
            integrator = 0;
        }
        float dtheta_speed = (kP_WHEELPOD * error + kI_WHEELPOD * integrator) / kV_MOTOR; // Error P loop

        float wheelPow = 60 * wheelSpeed / (M_PI * WHEEL_DIA * G2_WHEELPOD * kV_MOTOR);
        float m1 = (-dtheta_speed + wheelDirection * wheelPow) / (2 * G1_WHEELPOD);
        float m2 = (-dtheta_speed - wheelDirection * wheelPow) / (2 * G1_WHEELPOD);

        setMotorSpeed(wheelpod->motor1, m1 / SYSTEM_VOLTAGE);
        setMotorSpeed(wheelpod->motor2, m2 / SYSTEM_VOLTAGE);

        vTaskDelayUntil( &xLastWakeTime, PERIOD_WHEELPOD );
    }
}

/**
 * Direction-ify an angle
 */
float signAngle(float sign, float angle) {
    if (sign > 0) {
        return angle;
    }
    else {
        if (angle >= 0) {
            return angle - ANGLE_MAX;
        }
        else {
            return angle + ANGLE_MAX;
        }
    }
}

/**
 * Whether two angles in -pi..pi space are within 90 degrees of eachother
 */
bool within90(float angle1, float angle2) {
    float a = angle1 - angle2;
    a = remainderf(a + ANGLE_MAX, 2*ANGLE_MAX) - ANGLE_MAX;
    return a <= ANGLE_MAX / 2;
}

float getAngle(Wheelpod* wheelpod) {
    float angle = encoder_read_angle(wheelpod->encoder) - wheelpod->angle_offset;
    if (angle > M_PI) {
        angle -= 2*M_PI;
    }
    return angle;
}

/**
 * Sends a message to a wheelpod
 * speed: in/s
 * angle: radians
 */
void setWheelpod(Wheelpod* wheelpod, float speed, float angle) {
    WheelpodCommand cmd;
    cmd.wheelAngle = angle;
    cmd.wheelSpeed = speed;
    xQueueSend(wheelpod->message_queue, &cmd, 0);
}