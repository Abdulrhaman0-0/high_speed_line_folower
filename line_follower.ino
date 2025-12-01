#include "LineSensor.h"
#include "MotorController.h"
#include "Robot.h"

// --- Hardware Configuration ---
// TB6612FNG Pins
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// Sensor Pins (Digital)
// Using Analog pins A0-A4 as Digital Inputs
#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4

// Buttons & LED
#define BTN_CALIB 2  // Connect to Digital 2
#define BTN_START 8  // Connect to Digital 8
#define LED_PIN 13   // Built-in LED

// Objects
LineSensor lineSensor(SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5);
MotorController motorController(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY, 1, 1);
Robot robot(&lineSensor, &motorController);

// Debounce
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
    Serial.begin(9600);
    
    lineSensor.begin();
    
    pinMode(BTN_CALIB, INPUT_PULLUP);
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("Line Follower Ready.");
    
    // 3.1 Sensor Test Mode
    // Hold START button during power-on to enter Test Mode
    if (digitalRead(BTN_START) == LOW) {
        Serial.println("--- SENSOR TEST MODE ---");
        Serial.println("Press CALIB to exit (reset required).");
        while (digitalRead(BTN_CALIB) == HIGH) {
            Serial.print("Pattern: ");
            Serial.println(lineSensor.getRawPattern(), BIN);
            delay(100);
        }
    }
    
    Serial.println("Press CALIB for Background Calibration.");
}

void loop() {
    // Button Handling
    int readingCalib = digitalRead(BTN_CALIB);
    int readingStart = digitalRead(BTN_START);
    
    // Simple state machine for UI
    static bool calibPressed = false;
    
    if (readingCalib == LOW) { // Button Pressed (Active Low)
        delay(50); // Simple debounce
        if (digitalRead(BTN_CALIB) == LOW) {
            if (robot.getState() == STATE_IDLE || robot.getState() == STATE_ERROR) {
                // First press: Calibrate Background
                Serial.println("Calibrating Background...");
                robot.calibrateBackground();
                robot.setState(STATE_CALIBRATING_BG);
                digitalWrite(LED_PIN, HIGH); // Solid ON
                delay(500);
            } else if (robot.getState() == STATE_CALIBRATING_BG) {
                // Second press: Calibrate Line
                Serial.println("Calibrating Line...");
                robot.calibrateLine();
                
                // Check if calibration succeeded
                if (robot.getState() != STATE_ERROR) {
                    robot.setState(STATE_CALIBRATING_LINE);
                    // Blink to indicate done/ready
                    for(int i=0; i<5; i++) { digitalWrite(LED_PIN, LOW); delay(100); digitalWrite(LED_PIN, HIGH); delay(100); }
                    digitalWrite(LED_PIN, LOW);
                    robot.setState(STATE_IDLE); // Go back to idle, ready to start
                    Serial.println("Calibration Done. Press START.");
                } else {
                    // Error state handled by robot.setState(STATE_ERROR) inside calibrateLine
                    Serial.println("Calibration Failed. Retry.");
                }
            }
        }
        while(digitalRead(BTN_CALIB) == LOW); // Wait for release
    }

    if (readingStart == LOW) {
        delay(50);
        if (digitalRead(BTN_START) == LOW) {
            if (robot.getState() == STATE_IDLE || robot.getState() == STATE_CALIBRATING_LINE) {
                Serial.println("Starting...");
                robot.setState(STATE_RUNNING);
            } else if (robot.getState() == STATE_RUNNING || robot.getState() == STATE_RECOVERY || robot.getState() == STATE_ERROR) {
                Serial.println("Stopping...");
                robot.setState(STATE_IDLE);
            }
        }
        while(digitalRead(BTN_START) == LOW);
    }

    // Robot Update Loop
    robot.update();
    
    // LED Status
    if (robot.getState() == STATE_ERROR) {
        // Blink fast
        digitalWrite(LED_PIN, (millis() / 200) % 2);
    }
}
