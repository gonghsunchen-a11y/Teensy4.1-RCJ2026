#include "arduino.h"
#include "main_core.h"
//#include "sub_core.h"
#include <Adafruit_GFX.h>


float pos_x_f = 0.0;
float pos_y_f = 0.0;

void test() {
    static uint32_t lastDisplayTime = 0;
    if (millis() - lastDisplayTime > 100) { // 每秒更新一次顯示
        lastDisplayTime = millis();
        // 在這裡更新顯示內容，例如：
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.printf("hello world\n");
        display.display();
        lastDisplayTime = millis();
        display.display();
    }
}
void setup(){
    main_core_init();
}
 void loop(){
    //update_gyro_sensor();
    //readBallCam();
    static uint32_t lastDisplayTime = 0;
    readussensor();
    //ballsensor();
    
    if (millis() - lastDisplayTime > 100) { // 每秒更新一次顯示
        lastDisplayTime = millis();
        // 在這裡更新顯示內容，例如：
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.printf("ball dist: %d\n", ballData.dist);
        display.printf("ball angle: %d\n",ballData.angle);
        display.printf("us f: %d\n", usData.dist_f);
        display.printf("us l: %d\n", usData.dist_l);
        display.printf("us r: %d\n", usData.dist_r);
        display.printf("us b: %d\n", usData.dist_b);
        //display.printf("gyro: %d\n", gyroData.angle);
        display.display();
        lastDisplayTime = millis();
        display.display();
    }
    if (digitalRead(BTN_UP) == LOW) {
        while (1)
        {
        test(); 
        }
    
    }
    if (digitalRead(BTN_ENTER) == LOW) {
        Serial8.write(0xCC); // 傳送校準指令
        drawMessage("SCANNING...");
        delay(200); 
    }
    if (digitalRead(BTN_ESC) == LOW) {
        Serial8.write(0xEE); // 傳送結束指令
        delay(200); 
    }
    if(Serial8.available()){
        
        uint8_t c = Serial8.read();
        if(c == 0xDD){
        drawMessage("SAVED!");
        delay(1000); // 讓 SAVED 停一下
            
            // 回到初始狀態
        drawMessage("READY");
        delay(200);
        display.clearDisplay();
        }
    }

}