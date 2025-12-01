#ifndef ROBOT_H
#define ROBOT_H

#include "LineSensor.h"
#include "MotorController.h"



// Debug Mode - Uncomment to enable
#define DEBUG 1

// Tuning Parameters - Adjustable
float Kp = 0.1; 
float Ki = 0.0;
float Kd = 0.5;
int BASE_SPEED = 150;
int MAX_TURN = 150; // Max turn speed difference
int RECOVERY_SPEED = 100;
unsigned long RECOVERY_TIMEOUT = 3000;

// History Buffer Size
#define HISTORY_SIZE 10

enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING_BG,
    STATE_CALIBRATING_LINE,
    STATE_RUNNING,
    STATE_RECOVERY,
    STATE_ERROR
};

class Robot {
private:
    LineSensor* sensors;
    MotorController* motors;
    RobotState currentState;
    
    // PID Variables
    int lastError;
    long integral;
    float I_MAX = 1000.0; // Anti-windup limit
    
    // History Buffer
    int errorHistory[HISTORY_SIZE];
    bool linePresentHistory[HISTORY_SIZE]; // Store if line was present
    int historyIndex;
    
    // Recovery
    unsigned long recoveryStartTime;
    int lastValidDirection; // -1 for Left, 1 for Right, 0 for Unknown
    
    // Debug
    unsigned long lastDebugTime;
    
    // Calibration Status
    bool isCalibrated;
    
public:
    Robot(LineSensor* s, MotorController* m) : sensors(s), motors(m) {
        currentState = STATE_IDLE;
        lastError = 0;
        integral = 0;
        historyIndex = 0;
        lastValidDirection = 0; // 0 = Unknown/No History
        lastDebugTime = 0;
        isCalibrated = false;
        
        // Initialize history
        for(int i=0; i<HISTORY_SIZE; i++) {
            errorHistory[i] = 0;
            linePresentHistory[i] = false;
        }
    }

    void setState(RobotState newState) {
        // Enforce calibration before running
        if (newState == STATE_RUNNING && !isCalibrated) {
            Serial.println("ERROR: Cannot start. Robot not calibrated!");
            return;
        }
        
        currentState = newState;
        if (newState == STATE_IDLE || newState == STATE_ERROR) {
            motors->stop();
        }
    }
    
    RobotState getState() {
        return currentState;
    }

    void calibrateBackground() {
        sensors->calibrateBackground();
    }

    void calibrateLine() {
        sensors->calibrateLine();
        sensors->printCalibrationReport(); // Print report after line calibration
        
        if (sensors->validateCalibration()) {
            isCalibrated = true;
            Serial.println("Calibration Successful.");
        } else {
            isCalibrated = false;
            Serial.println("Calibration Failed: Not enough active sensors.");
            setState(STATE_ERROR);
        }
    }

    void update() {
        if (currentState != STATE_RUNNING && currentState != STATE_RECOVERY) return;

        // 1.2 Single sensor read per loop
        int16_t position = sensors->readPosition();
        
        if (position == -1) {
            // Line Lost
            if (currentState == STATE_RUNNING) {
                // Switch to Recovery
                currentState = STATE_RECOVERY;
                recoveryStartTime = millis();
                
                // Determine direction from RECENT history
                // Look at last 5 samples
                long sumError = 0;
                int validSamples = 0;
                
                for(int i=1; i<=5; i++) {
                    int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
                    if (linePresentHistory[idx]) {
                        sumError += errorHistory[idx];
                        validSamples++;
                    }
                }
                
                if (validSamples > 0) {
                    float avgError = (float)sumError / validSamples;
                    lastValidDirection = (avgError > 0) ? -1 : 1;
                }
            }
            
            handleRecovery();
            return;
        }
        
        // Line Found - Switch back to Running if in Recovery
        if (currentState == STATE_RECOVERY) {
            currentState = STATE_RUNNING;
            integral = 0; // Reset integral on recovery
        }

        // PID Calculation
        int error = 2000 - position;
        
        // Update History
        errorHistory[historyIndex] = error;
        linePresentHistory[historyIndex] = true;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;

        long P = error;
        integral += error;
        
        // Anti-windup
        if (integral > I_MAX) integral = I_MAX;
        if (integral < -I_MAX) integral = -I_MAX;
        
        long D = error - lastError;
        lastError = error;

        float output = (Kp * P) + (Ki * integral) + (Kd * D);
        
        // 1.5 Clamp PID output
        if (output > MAX_TURN) output = MAX_TURN;
        if (output < -MAX_TURN) output = -MAX_TURN;

        // Motor Control
        int leftSpeed = BASE_SPEED - output;
        int rightSpeed = BASE_SPEED + output;

        motors->drive(leftSpeed, rightSpeed);
        
        #ifdef DEBUG
        // 1.3 Rate-limited debug
        if (millis() - lastDebugTime > 100) {
            lastDebugTime = millis();
            Serial.print("POS="); Serial.print(position);
            Serial.print(" ERR="); Serial.print(error);
            Serial.print(" P="); Serial.print(P);
            Serial.print(" I="); Serial.print(integral);
            Serial.print(" D="); Serial.print(D);
            Serial.print(" OUT="); Serial.print(output);
            Serial.print(" L="); Serial.print(leftSpeed);
            Serial.print(" R="); Serial.print(rightSpeed);
            Serial.print(" MODE="); 
            if (currentState == STATE_RUNNING) Serial.print("RUN");
            else if (currentState == STATE_RECOVERY) Serial.print("REC");
            else Serial.print("ERR");
            
            Serial.print(" PAT="); Serial.println(sensors->getRawPattern(), BIN);
        }
        #endif
    }

    void handleRecovery() {
        // 2.2 Use constants for timeout
        if (millis() - recoveryStartTime > RECOVERY_TIMEOUT) { 
            setState(STATE_ERROR);
            return;
        }
        
        if (lastValidDirection == 0) {
             motors->stop();
             return;
        }

        // 2.2 Use constants for speed
        if (lastValidDirection == -1) {
            motors->drive(-RECOVERY_SPEED, RECOVERY_SPEED);
        } else {
            motors->drive(RECOVERY_SPEED, -RECOVERY_SPEED);
        }
        
        #ifdef DEBUG
        if (millis() - lastDebugTime > 100) {
            lastDebugTime = millis();
            Serial.print("RECOVERY: Dir="); Serial.print(lastValidDirection);
            Serial.print(" Time="); Serial.println(millis() - recoveryStartTime);
        }
        #endif
    }
};

#endif
