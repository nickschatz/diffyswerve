#pragma once

#include "wheelpod_control.h"

void mix3(float vx, float vy, float w, Wheelpod* podfw, Wheelpod* podbr, Wheelpod* podbl);

float calc_wx(Wheelpod* pod, float vx, float w);
float calc_wy(Wheelpod* pod, float vy, float w);

void apply_speed_dir(Wheelpod* pod, float vx, float vy, float w);