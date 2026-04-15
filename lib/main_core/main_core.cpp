#include "main_core.h"

// --- MEMORY ALLOCATION (Fixes Linker Errors) ---
int _page = 0;
int _cursor = 0;
unsigned long _lastPress = 0;
unsigned long _lastUpdate = 0;

// If these aren't defined in sub_core.cpp, they MUST be here:
// RobotStatus robot; 
// GyroData gyroData;

// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BallData ballData;



void main_core_init() {
    // Initialize all Hardware Serials
    Serial.begin(115200);
    Serial2.begin(115200); 
    Serial3.begin(115200);
    Serial4.begin(115200); 
    Serial5.begin(115200);
    Serial6.begin(115200); 
    Serial7.begin(115200);
    Serial8.begin(115200);

    Wire.begin();
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Pin Setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    /*/
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW);
    digitalWrite(Kicker_Pin, LOW);*/
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);
    pinMode(A14, INPUT);
    pinMode(A15, INPUT);
    pinMode(A16, INPUT);
    pinMode(A17, INPUT);
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

void kicker_control(bool kick) {
    static uint32_t charge_start_time = 0;
    static bool is_charged = false;
    static bool is_charging = false;

    const uint32_t CHARGE_MS = 5000; 
    uint32_t now = millis();

    // 1. Kick Logic (Priority)
    if (kick && is_charged) {
        digitalWrite(Kicker_Pin, HIGH);
        delay(15); // Solenoid pulse
        digitalWrite(Kicker_Pin, LOW);
        
        is_charged = false; 
        is_charging = false;
        Serial.println("KICKED");
        return; // Exit to prevent immediate re-charge trigger in same frame
    }

    // 2. Charging State Machine
    if (!is_charged && !is_charging) {
        // Start a new charge cycle
        charge_start_time = now;
        is_charging = true;
        digitalWrite(Charge_Pin, HIGH);
        Serial.println("CHARGING...");
    }

    if (is_charging) {
        if (now - charge_start_time >= CHARGE_MS) {
            // Charge complete
            digitalWrite(Charge_Pin, LOW);
            is_charging = false;
            is_charged = true;
            Serial.println("READY TO KICK");
        }
    }
}

void read_OmniCam() {
    // To be implemented: Serial parsing for OmniCam
}

void readussensor() {
    static float dist_b_f = 0.0f;
    static float dist_l_f = 0.0f;
    static float dist_r_f = 0.0f;
    static float dist_f_f = 0.0f;

    // read raw ADC and convert to cm (or mm depending on your scaling)
    float dist_b_raw = analogRead(back_us) * 520.0f / 1024.0f;
    float dist_l_raw = analogRead(left_us) * 520.0f / 1024.0f;
    float dist_r_raw = analogRead(right_us) * 520.0f / 1024.0f;
    float dist_f_raw = analogRead(front_us) * 520.0f / 1024.0f;
    // complementary (low-pass) filtering
    dist_b_f = alpha * dist_b_f + (1.0f - alpha) * dist_b_raw;
    dist_l_f = alpha * dist_l_f + (1.0f - alpha) * dist_l_raw;
    dist_r_f = alpha * dist_r_f + (1.0f - alpha) * dist_r_raw;
    dist_f_f = alpha * dist_f_f + (1.0f - alpha) * dist_f_raw;
    // assign filtered values to struct
    usData.dist_b = dist_b_f;
    usData.dist_l = dist_l_f;
    usData.dist_r = dist_r_f;
    usData.dist_f = dist_f_f;
}
/*void ballsensor(){
  // 發送請求封包，通知感測器回傳資料
  uint8_t b[4];
  ballData.valid = false;

  Serial6.write(0xBB);
  while(!Serial6.available());
  Serial6.readBytes(b,4);
  if(b[1]==0xFF){
      ballData.valid = false;
      ballData.angle = 255;
      ballData.dist = 255;
  }
  else if(b[0]==0xAA){
    uint8_t temp =b[1];
    ballData.valid = true;
    ballData.angle = (temp & 0x0F);
    ballData.dist = (temp & 0xF0)>>4;
    ballData.possession = (uint8_t)((1-alpha) * b[2] + ballData.possession * alpha);
  }
  else{
    ballData.valid = false;
  }
}*/