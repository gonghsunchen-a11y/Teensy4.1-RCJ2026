#include <Arduino.h>
#include <sub_core.h>
#include "RCJTaiwan.h"

void setup(){
    sub_core_init();
}

void loop(){
    update_gyro_sensor();
    update_line_sensor();

    FC_Vector_Motion(0,20,90);
}
