#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "wheelpod_control.h"
#include "motor_control.h"
#include "swerve_control.h"
#include <math.h>
#include <stdlib.h>
#include <driver/adc.h>

void mix3(float vx, float vy, float w, Wheelpod* podfw, Wheelpod* podbr, Wheelpod* podbl) {
    apply_speed_dir(podfw, vx, vy, w);
    apply_speed_dir(podbr, vx, vy, w);
    apply_speed_dir(podbl, vx, vy, w);
}

float calc_wx(Wheelpod* pod, float vx, float w) {
    return vx + pod->ypos * w;
}
float calc_wy(Wheelpod* pod, float vy, float w) {
    return vy - pod->xpos * w;
}

void apply_speed_dir(Wheelpod* pod, float vx, float vy, float w) {
    float wx = calc_wx(pod, vx, w);
    float wy = calc_wy(pod, vy, w);

    float theta = atan2f(wy, wx);
    float speed = sqrtf(wx * wx + wy * wy);

    setWheelpod(pod, speed, theta);
}