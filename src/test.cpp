#include <Arduino.h>
#include <sub_core.h>
#include "RCJTaiwan.h"

void setup(){
    sub_core_init();
}

void loop(){
    update_gyro_sensor();
    update_line_sensor();

    #define STRAFE_SPEED 35
    #define STRAFE_TOTAL 5

    // 等待碰到任何線
    Serial.println("Waiting for line...");
    while (!isLineTouched(LS_MASK_LEFT | LS_MASK_RIGHT | LS_MASK_FRONT | LS_MASK_BACK)) {
        update_line_sensor();
        stopRobot();
    }
    Serial.println("ON LINE, START STRAFE");

    for (int i = 0; i < STRAFE_TOTAL; i++) {
        moveUntilLeftTouch(-STRAFE_SPEED, 0, 5000);
        Serial.printf("HIT LEFT (%d)\n", i);
        moveUntilRightTouch(STRAFE_SPEED, 0, 5000);
        Serial.printf("HIT RIGHT (%d)\n", i);
    }

    Serial.println("STRAFE DONE");
    stopRobot();
    while(1); // 停在這裡，不重複執行
}