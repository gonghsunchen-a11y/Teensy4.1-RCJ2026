#include <sub_core.h>
uint8_t op_mode;

static int forwardCounter = 0;
static int backwardCounter = 0;
const int MAX_VX = 30;

void update_all_sensor(){
    update_line_sensor();
    update_gyro_sensor();
}


void c_mode_main_function() {
    Serial.println("Cmode Started");
    while (1){
      read_cam_and_pos_data();
      update_line_sensor(); // Keep updating sensors!
      update_gyro_sensor();
      Serial.printf("Front LS: %d, Mid LS: %d\n", (lineData.state >> 32) & 1, (lineData.state >> 33) & 1);
      Serial.printf("Gyro Heading: %f\n", gyroData.heading);
      Serial.printf("Ball Valid: %d, Ball Angle: %d, Ball Distance: %d \n", ballData.valid, ballData.angle, ballData.dist);
      Serial.printf("Robot Pos: (%f, %f)\n", RobotPos.x, RobotPos.y);
              //White Line Handling Example
      bool back_touch = analogRead(Back_LS) < avg_ls[34];
      static bool was_back_touch = false;
      static bool was_front_touch = false;
      if(back_touch){
        was_back_touch = true;
      }
      if(was_back_touch == true){
        //後觸發，向後
        forwardCounter = 0;
        backwardCounter++;
        int speed = -10 - backwardCounter / 30;
        if (speed < -25) speed = -25;
        mainCommand.vx = 0;
        mainCommand.vy = speed;
      }
       
        bool front_touch = analogRead(Front_LS) < avg_ls[32];
        bool mid_touch = analogRead(Mid_LS) < avg_ls[33];
        Serial.printf("Front LS: %d, Mid LS: %d\n", front_touch, mid_touch);
      if (front_touch) {
        was_front_touch = true;
      }
      if(was_front_touch == true){
        //前觸發，向前
        forwardCounter++;
        backwardCounter = 0;
        int speed = 25 + forwardCounter / 30;
        if (speed > 45) speed = 45;          mainCommand.vy = speed;
      }
      if(mid_touch){
        //停止
        was_back_touch = false;
        was_front_touch = false;
        forwardCounter = 0;
        backwardCounter = 0;
        mainCommand.vy = 0; 
      }
        if(!((lineData.state >> 4) & 1) && !((lineData.state >> 12) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -25 - backwardCounter / 30;
            if (speed < -35) speed = -35;
            mainCommand.vy = speed;
        }

        if(!((lineData.state >> 5) & 1) && !((lineData.state >> 11) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -20 - backwardCounter / 30;
            if (speed < -30) speed = -30;
            mainCommand.vy = speed;
        } 
        if(!((lineData.state >> 6) & 1) && !((lineData.state >> 10) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -15 - backwardCounter / 30;
            if (speed < -25) speed = -25;
            mainCommand.vy = speed;
        }
        if(!((lineData.state >> 7) & 1 )&& !((lineData.state >> 9) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -10 - backwardCounter / 30;
            if (speed < -20) speed = -20;
            mainCommand.vy = speed;
        }

        if(!((lineData.state >> 8) & 1 )){
            mainCommand.vy = 0;
        }

        if (ballData.valid) {
            if (ballData.angle > 100 && ballData.angle < 260) {
                mainCommand.vx = -MAX_VX; // Want to go Left
            } 
            else if (ballData.angle < 80 || ballData.angle > 280) {
                mainCommand.vx = MAX_VX;  // Want to go Right
            }
        else { 
                mainCommand.vx = MAX_VX*0;
        }
        }


        static bool right_locked = false;
        static bool left_locked  = false;
        static int side_lock = 0;

        // ===== 右邊 =====
        bool right_outer = !((lineData.state >> 0) & 1);
        bool right_inner = !((lineData.state >> 28) & 1);

        // ===== 左邊 =====
        bool left_outer  = !((lineData.state >> 16) & 1);
        bool left_inner  = !((lineData.state >> 20) & 1);


        // =====================
        // 右邊邏輯
        // =====================

        // 還在線內但靠線
        if (side_lock == 0) {
    if (right_inner) {
        side_lock = 1;
    }
    else if (left_inner) {
        side_lock = 2;
    }
}



      if (side_lock == 1) {

          if (right_inner) {
              right_locked = true;

              if (mainCommand.vy < 0) {
                  mainCommand.vy = 0;
              }
          }

          if (right_locked) {
              mainCommand.vx = -15;
          }

          if (right_outer) {
              mainCommand.vx = 0;
              right_locked = false;
              side_lock = 0;
          }
      }


      if (side_lock == 2) {

          if (left_inner) {
              left_locked = true;

              if (mainCommand.vy < 0) {
                  mainCommand.vy = 0;
              }
          }

          if (left_locked) {
              mainCommand.vx = 15;
          }

          if (left_outer) {
              mainCommand.vx = 0;
              left_locked = false;
              side_lock = 0;
          }
      }

        Vector_Motion(mainCommand.vx, 0, mainCommand.rot_v);
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
    }

    /*

    while(1) {
      read_cam_and_pos_data();
      update_line_sensor(); // Keep updating sensors!
      update_gyro_sensor();
        // Add logic here
        readMotor();
        moveBackInBounds();
        //White Line Handling Example
       /* bool back_touch = analogRead(Back_LS) < avg_ls[34];
        if(back_touch){
          //後觸發，向後
          forwardCounter = 0;
          backwardCounter++;
          int speed = -10 - backwardCounter / 30;
          if (speed < -25) speed = -25;
          mainCommand.vx = 0;
          mainCommand.vy = speed;
        }*/
       /*
        bool front_touch = analogRead(Front_LS) < avg_ls[32];
        bool mid_touch = analogRead(Mid_LS) < avg_ls[33];
        Serial.printf("Front LS: %d, Mid LS: %d\n", front_touch, mid_touch);
        if(front_touch){
          //前觸發，向前
          forwardCounter++;
          backwardCounter = 0;
          int speed = 25 + forwardCounter / 30;
          if (speed > 45) speed = 45;
          mainCommand.vy = speed;
        }
        if(mid_touch){
          //停止
          forwardCounter = 0;
          backwardCounter = 0;
          mainCommand.vy = 0;
        }
        if(!((lineData.state >> 4) & 1) && !((lineData.state >> 12) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -25 - backwardCounter / 30;
            if (speed < -35) speed = -35;
            mainCommand.vy = speed;
        }

        if(!((lineData.state >> 5) & 1) && !((lineData.state >> 11) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -20 - backwardCounter / 30;
            if (speed < -30) speed = -30;
            mainCommand.vy = speed;
        } 
        if(!((lineData.state >> 6) & 1) && !((lineData.state >> 10) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -15 - backwardCounter / 30;
            if (speed < -25) speed = -25;
            mainCommand.vy = speed;
        }
        if(!((lineData.state >> 7) & 1 )&& !((lineData.state >> 9) & 1)){
            forwardCounter = 0;
            backwardCounter++;
            int speed = -10 - backwardCounter / 30;
            if (speed < -20) speed = -20;
            mainCommand.vy = speed;
        }

        if(!((lineData.state >> 8) & 1 )){
            mainCommand.vy = 0;
        }
        int s = lineData.state;

        bool rightLine =
            !((s >> 0) & 1)  ||
            !((s >> 1) & 1)  ||
            !((s >> 2) & 1)  ||
            !((s >> 3) & 1)  ||
            !((s >> 31) & 1) ||
            !((s >> 30) & 1) ||
            !((s >> 29) & 1) ||
            !((s >> 28) & 1) ||
            !((s >> 27) & 1) ||
            !((s >> 26) & 1);

        bool leftLine =
            !((s >> 16) & 1) ||
            !((s >> 13) & 1) ||
            !((s >> 14) & 1) ||
            !((s >> 15) & 1) ||
            !((s >> 17) & 1) ||
            !((s >> 18) & 1) ||
            !((s >> 19) & 1) ||
            !((s >> 20) & 1) ||
            !((s >> 21) & 1) ||
            !((s >> 22) & 1);


        // ===== 右邊界 =====
        if (rightLine) {
            if (mainCommand.vx > 0) {
                mainCommand.vx = 0;     // 禁止再往右
            }
            else if (mainCommand.vx == 0) {
                mainCommand.vx = -8;    // 自己往左退
            }
        }


        // ===== 左邊界 =====
        if (leftLine) {
            if (mainCommand.vx < 0) {
                mainCommand.vx = 0;     // 禁止再往左
            }
            else if (mainCommand.vx == 0) {
                mainCommand.vx = 8;     // 自己往右退
            }
        }
        
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
        if(moveBackInBounds()){
        //  Vector_Motion(lineVx, lineVy, 0);
          continue;
        }
        Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
    }
    */
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1) {
        update_line_sensor(); // Keep updating sensors!
        update_gyro_sensor();
        readMotorandSendSensors();
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v, mainCommand.heading);
        FC_Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.heading); 
    }
}


void setup(){
  sub_core_init();
  while(1){
  Serial.println("Waiting for MainCore...");
    if(Serial8.available()){
      op_mode = Serial8.read();
      Serial.printf("Received mode: 0x%X\n", op_mode);
      if(op_mode == T_MODE_HEADER || op_mode == C_MODE_HEADER){
        Serial8.write(PROTOCAL_ACT); // Acknowledge receipt
        break;
      }
    }
  }
}

void loop(){
  
  while(1){
    update_gyro_sensor();
    update_line_sensor();
    //Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    for(uint8_t i = 0; i < 32; i++){
      Serial.printf("%d", (lineData.state >> i) & 1);
    }
    Serial.println();
    if (Serial8.available()) {
      uint8_t cmd = Serial8.read();
      //Serial.print(cmd);
      if (cmd == LS_CAL_START) {
          uint16_t max_ls[32], min_ls[32];
          uint16_t front_max = 0, front_min = 4095;
          uint16_t mid_max = 0, mid_min = 4095;
          for (int i = 0; i < 32; i++) { 
            max_ls[i] = 0; 
            min_ls[i] = 4095; 
          }
          int timer = 0;
          while (1) {
            if(micros() - timer > 100000) { // 每 100ms 更新一次
              digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED for visual feedback
              timer = micros();
            }
            if(Serial8.available()){
              uint8_t cmd = Serial8.read();
              if(cmd == LS_CAL_END){ // End calibration command
                break;
              }
            }
            for (int i = 0; i < LS_count; i++) {
              int r = readMux(i % 16, (i < 16) ? 1 : 2);
              if (r > max_ls[i]) max_ls[i] = r; 
              if (r < min_ls[i]) min_ls[i] = r;
            }
            uint16_t reading = analogRead(Front_LS);
            if(reading > front_max) front_max = reading;
            if(reading < front_min) front_min = reading;
            reading = analogRead(Mid_LS);
            if(reading > mid_max) mid_max = reading;
            if(reading < mid_min) mid_min = reading;

          }

          for (int i = 0; i < LS_count; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          for (int i = 0; i < LS_count; i++) {
            Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
          }
          avg_ls[32] = (front_max + front_min) / 2;
          avg_ls[33] = (mid_max + mid_min) / 2;
          EEPROM.put(0, avg_ls);
          delay(1000); // Ensure EEPROM write completes
          Serial8.write(LS_CAL_ACK); // Send end calibration acknowledgment
        } 
      else if (cmd == MOVE_CMD) {// When BTN_UP is pressed, send a move command to the main core
        Serial8.write(PROTOCAL_ACT);
        break;
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
  if(op_mode == C_MODE_HEADER){
    c_mode_main_function();
  }
  else if(op_mode == T_MODE_HEADER){
    t_mode_main_function();
  }
}
/*
void loop(){
  update_line_sensor();
  update_gyro_sensor();
  readfrom_MainCore();
  switch (mainCommand.type) {
    case MainCoreCommand::ACTUATE:
      white_line_handle();
      break;
    case MainCoreCommand::CALIBRATE:
      calibrate();
      mainCommand.type = MainCoreCommand::ACTUATE; // Reset to default after calibration
      break;
  }
}
*/