#include <Arduino.h>
#include <sub_core.h>

int sensorPin = A7;  
int sensorValue = 0;

int middlePin = A6;
int middleValue = 0;

int vx = 0;
int vy = 0;

static int forwardCounter = 0;
static int backwardCounter = 0;

void setup() {
    sub_core_init();  

    Serial.begin(9600);
    
}

void loop() {
    update_gyro_sensor();

    int mainSensor = analogRead(sensorPin);
    int middleSensor = analogRead(middlePin);

    int muxSensor0 = readMux(1, 1);
    int muxSensor9 = readMux(15, 1);
    int muxSensor1 = readMux(2, 1);
    int muxSensor8 = readMux(14, 1);
    int muxSensor2 = readMux(3, 1);
    int muxSensor7 = readMux(13, 1); 
    int muxSensor3 = readMux(4, 1);
    int muxSensor6 = readMux(12, 1);
    int muxSensor4 = readMux(5, 1);
    int muxSensor5 = readMux(11, 1);

    int muxSensormid = readMux(8, 1);

    int muxSensor10 = readMux(1, 2);
    int muxSensor11= readMux(15, 2);
    int muxSensor12 = readMux(2, 2);
    int muxSensor13 = readMux(14, 2);
    int muxSensor14 = readMux(3, 2);
    int muxSensor15 = readMux(13, 2); 
    int muxSensor16 = readMux(4, 2);
    int muxSensor17 = readMux(12, 2);
    int muxSensor18 = readMux(5, 2);
    int muxSensor19 = readMux(11, 2);
    
     
    


    Serial.printf("(1,2): %d\n", muxSensor10);
  
    if (muxSensor0 < 190) {
        forwardCounter = 0;
        backwardCounter = 0;
        vx = 0;
        vy = 0;
    }
    else if (muxSensor0 < 130 && muxSensor9 < 130 || muxSensor1 < 130 && muxSensor8 < 130 || muxSensor2 < 130 && muxSensor7 < 130 || muxSensor3 < 157 && muxSensor6 < 170 || muxSensor4 < 160 && muxSensor5 < 175 || muxSensor10 < 130 || muxSensor11 < 130 || muxSensor12 < 130 || muxSensor13 < 130 || muxSensor14 < 130 || muxSensor15 < 130 || muxSensor16 < 130 || muxSensor17 < 130 || muxSensor18 < 130 || muxSensor19 < 130) {
        forwardCounter = 0;
        backwardCounter++;
        int speed = -15 - backwardCounter / 30;
        if (speed < -25) speed = -25;
        vx = 15;
        vy = speed;  
    }  
    else if ((mainSensor < 255 || mainSensor > 268) && (middleSensor >= 35 && middleSensor <= 65)) {
        backwardCounter = 0;
        forwardCounter++;
        if (forwardCounter > 50) forwardCounter = 50;  
        int speed = 10 + forwardCounter / 50; 
        if (speed > 20) speed = 20;  
        vx = 15;
        vy = speed;
    } 
    else if (middleSensor < 65  && middleSensor > 35 || muxSensormid > 153 && muxSensormid < 160    ) {
        forwardCounter = 0; 
        backwardCounter = 0;
        vx = 15;
        vy = 0;
    }
    Vector_Motion(vx, vy, 0);
}