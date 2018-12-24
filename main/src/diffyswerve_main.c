#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "motor_control.h"


void test_motor_control(void *arg)
{
    Motor* motor1 = init_motor(23, 0);
    Motor* motor2 = init_motor(22, 0);
    Motor* motor3 = init_motor(21, 0);
    Motor* motor4 = init_motor(19, 0);
    Motor* motors[] = {motor1, motor2, motor3, motor4};
    while (1) {
        for (int i = 0; i < 4; i++) {
            set_motor_speed(motors[i], 0);
            vTaskDelay(100);
        }
        for (int i = 0; i < 4; i++) {
            set_motor_speed(motors[i], 1);
            vTaskDelay(100);
        }
    }
}

void app_main()
{
    xTaskCreate(test_motor_control, "test_motor_control", 4096, NULL, 5, NULL);
}