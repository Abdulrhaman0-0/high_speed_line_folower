#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>

class LineSensor {
private:
    uint8_t sensorPins[5];
    bool sensorPolarity[5]; // true if HIGH means Line, false if LOW means Line
    bool sensorActive[5];   // true if sensor is working/calibrated
    int backgroundStates[5];
    int lineStates[5];      // Store for report
    uint32_t sensorWeights[5] = {0, 1000, 2000, 3000, 4000};
    
public:
    LineSensor(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
        sensorPins[0] = p0;
        sensorPins[1] = p1;
        sensorPins[2] = p2;
        sensorPins[3] = p3;
        sensorPins[4] = p4;
        
        // Default initialization
        for(int i=0; i<5; i++) {
            sensorPolarity[i] = false; // Default: LOW = Line (common for IR)
            sensorActive[i] = true;    // Assume active until proven otherwise
            backgroundStates[i] = 0;
            lineStates[i] = 0;
        }
    }

    void begin() {
        for (int i = 0; i < 5; i++) {
            pinMode(sensorPins[i], INPUT);
        }
    }

    // Helper: Read a sensor multiple times and return majority vote
    int readStable(uint8_t pin) {
        int highCount = 0;
        for (int i = 0; i < 10; i++) {
            if (digitalRead(pin) == HIGH) highCount++;
            delayMicroseconds(100); // Small delay between samples
        }
        return (highCount > 5) ? HIGH : LOW;
    }

    // Step 1: Calibrate Background (No Line)
    void calibrateBackground() {
        for (int i = 0; i < 5; i++) {
            backgroundStates[i] = readStable(sensorPins[i]);
            // Reset active flag to true before line calibration checks
            sensorActive[i] = true; 
        }
    }

    // Step 2: Calibrate Line (On Line)
    void calibrateLine() {
        for (int i = 0; i < 5; i++) {
            int lineState = readStable(sensorPins[i]);
            lineStates[i] = lineState; // Store for report
            
            if (lineState != backgroundStates[i]) {
                // Sensor detected a change, it is valid.
                // If lineState is HIGH and background was LOW, then HIGH = Line.
                sensorPolarity[i] = (lineState == HIGH);
                sensorActive[i] = true;
            } else {
                // Sensor did not change state. 
                // Mark as inactive to prevent noise/errors.
                sensorActive[i] = false;
            }
        }
    }
    
    // Check if calibration was successful (enough active sensors)
    bool validateCalibration() {
        int activeCount = 0;
        for (int i = 0; i < 5; i++) {
            if (sensorActive[i]) activeCount++;
        }
        return activeCount >= 3; // Require at least 3 active sensors
    }
    
    void printCalibrationReport() {
        Serial.println("--- Sensor Calibration Report ---");
        Serial.println("ID | BG  | Line | Polarity | Status");
        for (int i = 0; i < 5; i++) {
            Serial.print(i);
            Serial.print("  | ");
            Serial.print(backgroundStates[i] == HIGH ? "HIGH" : "LOW ");
            Serial.print("| ");
            Serial.print(lineStates[i] == HIGH ? "HIGH" : "LOW ");
            Serial.print("| ");
            if (sensorActive[i]) {
                Serial.print(sensorPolarity[i] ? "Line=HIGH" : "Line=LOW ");
            } else {
                Serial.print("UNKNOWN  ");
            }
            Serial.print("| ");
            
            if (sensorActive[i]) {
                Serial.println("OK");
            } else {
                Serial.println("WARNING: No Change!");
            }
        }
        Serial.println("---------------------------------");
        
        if (!validateCalibration()) {
            Serial.println("ERROR: Not enough active sensors! Adjust height/angle.");
        }
    }

    // Returns virtual position 0-4000.
    // Returns -1 if line is lost (no sensors active).
    int16_t readPosition() {
        uint32_t weightedSum = 0;
        uint16_t sum = 0;
        
        for (int i = 0; i < 5; i++) {
            // Skip inactive sensors
            if (!sensorActive[i]) continue;

            bool active = false;
            int state = digitalRead(sensorPins[i]);
            
            if (sensorPolarity[i]) {
                if (state == HIGH) active = true;
            } else {
                if (state == LOW) active = true;
            }

            if (active) {
                weightedSum += sensorWeights[i];
                sum++;
            }
        }

        if (sum == 0) {
            return -1; // Line lost
        }

        return weightedSum / sum;
    }
    
    // Get raw pattern for debug (1 bit per sensor)
    uint8_t getRawPattern() {
        uint8_t pattern = 0;
        for (int i = 0; i < 5; i++) {
            if (!sensorActive[i]) continue;
            
            int state = digitalRead(sensorPins[i]);
            bool active = false;
            if (sensorPolarity[i]) {
                if (state == HIGH) active = true;
            } else {
                if (state == LOW) active = true;
            }
            
            if (active) {
                pattern |= (1 << i);
            }
        }
        return pattern;
    }
    
    // Helper to check if line is lost
    bool isLineLost() {
        return readPosition() == -1;
    }
};

#endif
