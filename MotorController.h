#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SparkFun_TB6612.h>

class MotorController {
private:
    Motor* motorL;
    Motor* motorR;
    
public:
    MotorController(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int stby, int offsetA, int offsetB) {
        motorL = new Motor(ain1, ain2, pwma, offsetA, stby);
        motorR = new Motor(bin1, bin2, pwmb, offsetB, stby);
    }

    void drive(int leftSpeed, int rightSpeed) {
        // Clamp speeds
        if (leftSpeed > 255) leftSpeed = 255;
        if (leftSpeed < -255) leftSpeed = -255;
        if (rightSpeed > 255) rightSpeed = 255;
        if (rightSpeed < -255) rightSpeed = -255;

        motorL->drive(leftSpeed);
        motorR->drive(rightSpeed);
    }

    void stop() {
        motorL->brake();
        motorR->brake();
    }
    
    // Optional: Force standby
    void standby() {
        // The library handles standby via the drive command if configured, 
        // but typically STBY pin needs to be pulled low.
        // The SparkFun library handles STBY in the constructor but doesn't expose a direct standby() method easily 
        // without accessing the pin directly if we didn't save it.
        // But brake() is usually sufficient.
    }
};

#endif
