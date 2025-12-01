#ifndef ROBOT_H
#define ROBOT_H

#include "LineSensor.h"
#include "MotorController.h"

// Tuning Parameters - Adjustable
float Kp = 0.1; 
float Ki = 0.0;
float Kd = 0.5;
int BASE_SPEED = 150;

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
    
    // History Buffer
    int errorHistory[HISTORY_SIZE];
    int historyIndex;
    
    // Recovery
    unsigned long recoveryStartTime;
    int lastValidDirection; // -1 for Left, 1 for Right
    
public:
    Robot(LineSensor* s, MotorController* m) : sensors(s), motors(m) {
        currentState = STATE_IDLE;
        lastError = 0;
        integral = 0;
        historyIndex = 0;
        lastValidDirection = 0;
        
        // Initialize history
        for(int i=0; i<HISTORY_SIZE; i++) errorHistory[i] = 0;
    }

    void setState(RobotState newState) {
        currentState = newState;
        if (newState == STATE_IDLE || newState == STATE_ERROR) {
            motors->stop();
        }
    }
    
    RobotState getState() {
        return currentState;
    }

    void calibrateBackground() {
        sensors->recordBackground();
    }

    void calibrateLine() {
        sensors->recordLine();
    }

    void update() {
        if (currentState != STATE_RUNNING && currentState != STATE_RECOVERY) return;

        int16_t position = sensors->readPosition();
        
        if (position == -1) {
            // Line Lost
            if (currentState == STATE_RUNNING) {
                // Switch to Recovery
                currentState = STATE_RECOVERY;
                recoveryStartTime = millis();
                
                // Determine direction from history
                // Average last few errors to decide direction
                long sumError = 0;
                for(int i=0; i<HISTORY_SIZE; i++) sumError += errorHistory[i];
                // If sumError is positive (Left), we should turn Left.
                // If sumError is negative (Right), we should turn Right.
                // Note: Error = 2000 - Position.
                // Position 0 (Left) -> Error +2000.
                // Position 4000 (Right) -> Error -2000.
                lastValidDirection = (sumError > 0) ? -1 : 1; 
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
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;

        long P = error;
        integral += error;
        // Anti-windup could be added here
        long D = error - lastError;
        lastError = error;

        float output = (Kp * P) + (Ki * integral) + (Kd * D);

        // Motor Control
        // Left = Base - Output
        // Right = Base + Output
        int leftSpeed = BASE_SPEED - output;
        int rightSpeed = BASE_SPEED + output;

        motors->drive(leftSpeed, rightSpeed);
    }

    void handleRecovery() {
        // Check timeout
        if (millis() - recoveryStartTime > 3000) { // 3 seconds timeout
            setState(STATE_ERROR);
            return;
        }

        // Spin in last valid direction
        int recoverySpeed = 100;
        if (lastValidDirection == -1) {
            // Turn Left (Left back, Right fwd)
            motors->drive(-recoverySpeed, recoverySpeed);
        } else {
            // Turn Right (Left fwd, Right back)
            motors->drive(recoverySpeed, -recoverySpeed);
        }
    }
};

#endif
