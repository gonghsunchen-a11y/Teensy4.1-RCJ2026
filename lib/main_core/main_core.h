#ifndef MAIN_CORE_H
#define MAIN_CORE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Dual_Core_Config.h"

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Button Pins ---
#define BTN_UP    31
#define BTN_DOWN  30
#define BTN_ENTER 27
#define BTN_ESC   26

// --- Ultrasonic Sensor Pins ---
#define front_us A15
#define left_us A14
#define back_us A16
#define right_us A17
#define alpha 0.75

extern struct USSensor{uint16_t dist_b = 0; uint16_t dist_l = 0; uint16_t dist_r = 0;uint16_t dist_f = 0; } usData;

// --- Kicker Constants ---
#define Charge_Pin 33 // Update to your actual pins
#define Kicker_Pin 32

// --- UI / Menu Globals (Externs) ---
extern int _page;
extern int _cursor;
extern unsigned long _lastPress;
extern unsigned long _lastUpdate;

// --- OLED Instance (Extern) ---
extern Adafruit_SSD1306 display;

struct BallData {
    uint16_t dist = 255;
    uint16_t angle = 255;
    uint16_t possession = 255;
    bool valid = false;
    float Vx;
    float Vy;
};

extern BallData ballData;

// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void updateMenu();        // Logic for button presses
void displayInterface();  // Logic for drawing the screen
void readCameraData();
void readBallCam();
void kicker_control(bool kick);
void localization();
void readussensor();
//void ballsensor();

//#define alpha 0.75


#endif