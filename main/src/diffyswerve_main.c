#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/i2c.h"
#include "encoder_i2c.h"
#include <driver/adc.h>

#include "motor_control.h"
#include "wheelpod_control.h"

void main_task(void *arg) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 23;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    encoder_i2c_init();
    
    Motor* wp1m1 = createMotor(34);
    Motor* wp1m2 = createMotor(35);
    Wheelpod* wheelpod1 = createWheelpod(wp1m1, wp1m2, (Encoder){.port = I2C_NUM_0, .mux_channel = 2}, 0.0);

    Motor* wp2m1 = createMotor(33);
    Motor* wp2m2 = createMotor(25);
    Wheelpod* wheelpod2 = createWheelpod(wp2m1, wp2m2, (Encoder){.port = I2C_NUM_0, .mux_channel = 3}, 0.0);

    Motor* wp3m1 = createMotor(27);
    Motor* wp3m2 = createMotor(14);
    Wheelpod* wheelpod3 = createWheelpod(wp3m1, wp3m2, (Encoder){.port = I2C_NUM_0, .mux_channel = 1}, 0.0);

    while (1) {
        setWheelpod(wheelpod1, 30, 0);
        setWheelpod(wheelpod2, 30, 0);
        setWheelpod(wheelpod3, 30, 0);

        printf("Wheelpod 1: %f\n", wheelpod1->angle * 180. / M_PI);
        printf("Wheelpod 2: %f\n", wheelpod2->angle * 180. / M_PI);
        printf("Wheelpod 3: %f\n", wheelpod3->angle * 180. / M_PI);

        vTaskDelay(100);
    }
}

void app_main()
{
    //xTaskCreate(test_encoder, "test_encoder", 4096, NULL, 5, NULL);
    xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);
}