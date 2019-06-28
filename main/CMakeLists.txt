set(COMPONENT_SRCS "src/diffyswerve_main.c" 
                   "src/motor_control.c"
                   "src/wheelpod_control.c"
                   "src/encoder_i2c.c"
                   "src/i2c.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()