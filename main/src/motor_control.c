#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

#include "motor_control.h"
#include <stdlib.h>
#include <math.h>

void initUnit(int unit, int timer) {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit, timer, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

Motor* createMotor(int gpio_pwm) {
    // Keep track of unit and pin for auto allocation
    static int unit = MCPWM_UNIT_0;
    static int output = MCPWM0A;
    if (unit >= MCPWM_UNIT_MAX) {
        // todo throw error
        return NULL;
    }
    // Each unit has 6 outputs, indexed by the timer (0,1,2) then operator (A,B)
    int my_unit = unit;
    int my_timer = output / 2;
    int my_operator = output % 2;
    if (my_operator == 0) {
        // First time using this timer, need to initialize it
        initUnit(my_unit, my_timer);
    }
    mcpwm_gpio_init(unit, output++, gpio_pwm);
    if (output > MCPWM2B) {
        unit++;
        output = MCPWM0A;
    }

    // Allocate motor on heap
    Motor* motor = (Motor*) malloc(sizeof(Motor));
    motor->unit = my_unit;
    motor->timer = my_timer;
    motor->op = my_operator;

    return motor;
}

/**
 * Sets the speed between -1.0 and 1.0
 */
void setMotorSpeed(Motor* motor, float speed) {
    // Clamp magnitude
    speed = speed > 1.0 ? 1.0 : speed;
    speed = speed < -1.0 ? -1.0 : speed;
    // Interpolate magnitude to us
    int range = MAX_PULSE_US - MIN_PULSE_US;

    int micros = MIN_PULSE_US + (1 + speed) * range / 2;
    mcpwm_set_duty_in_us(motor->unit, motor->timer, motor->op, micros);
}