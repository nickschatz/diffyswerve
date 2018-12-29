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

Motor* createMotor(int gpio_pwm, int gpio_dir) {
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
        init_unit(my_unit, my_timer);
    }
    mcpwm_gpio_init(unit, output++, gpio_pwm);
    if (output > MCPWM2B) {
        unit++;
        output = MCPWM0A;
    }

    // Configure GPIO
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<gpio_dir;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // Allocate motor on heap
    Motor* motor = (Motor*) malloc(sizeof(Motor));
    motor->gpio_dir = gpio_dir;
    motor->unit = my_unit;
    motor->timer = my_timer;
    motor->op = my_operator;

    return motor;
}

/**
 * Sets the speed between -1.0 and 1.0
 */
void setMotorSpeed(Motor* motor, float speed) {
    float magnitude = fabs(speed);
    int direction = speed > 0;
    // Clamp magnitude
    magnitude = fmin(fmax(magnitude, 0), 1.0);
    // Interpolate magnitude to us
    int micros = MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * magnitude;
    mcpwm_set_duty_in_us(motor->unit, motor->timer, motor->op, micros);
    // Write direction pin
    gpio_set_level(motor->gpio_dir, direction);
}