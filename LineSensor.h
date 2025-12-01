#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>

class LineSensor {
private:
    uint8_t sensorPins[5];
    bool sensorPolarity[5]; // true if HIGH means Line, false if LOW means Line
    uint32_t sensorWeights[5] = {0, 1000, 2000, 3000, 4000};
    
public:
    LineSensor(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
        sensorPins[0] = p0;
        sensorPins[1] = p1;
        sensorPins[2] = p2;
        sensorPins[3] = p3;
        sensorPins[4] = p4;
    }

    void begin() {
        for (int i = 0; i < 5; i++) {
            pinMode(sensorPins[i], INPUT);
        }
    }

    // Step 1: Calibrate Background (No Line)
    // Returns true if successful
    void calibrateBackground() {
        for (int i = 0; i < 5; i++) {
            // If background is read as HIGH, then Line must be LOW (and vice versa)
            // We store the state that corresponds to BACKGROUND.
            // Wait, we want to store polarity.
            // Let's assume we just read the state.
            // If we read HIGH on background, then Line will be LOW.
            // So Polarity (LineIsHigh) = !ReadState
            int state = digitalRead(sensorPins[i]);
            // We will finalize polarity in step 2, but for now let's just store what we see.
            // Actually, to be robust, we need both steps.
            // Let's store the background state temporarily or just wait for step 2.
            // But the plan says:
            // Step 1: Record background state.
            // Step 2: Record line state.
            // Determine polarity.
            // We can't determine polarity until we see both or assume one is different.
            // Let's store background state in a member variable?
            // Or just assume the user follows instructions.
            // Let's store the "BackgroundState" for each sensor.
            // For simplicity in this class, let's just save the background reading.
             sensorPolarity[i] = (state == LOW); // Temporary guess: if background is LOW, line is HIGH.
        }
    }
    
    // Helper to store background states
    int backgroundStates[5];

    void recordBackground() {
        for (int i = 0; i < 5; i++) {
            backgroundStates[i] = digitalRead(sensorPins[i]);
        }
    }

    void recordLine() {
        for (int i = 0; i < 5; i++) {
            int lineState = digitalRead(sensorPins[i]);
            // If LineState != BackgroundState, we have a valid sensor.
            // If LineState == HIGH, then Polarity is true (High=Line).
            // If LineState == LOW, then Polarity is false (Low=Line).
            if (lineState != backgroundStates[i]) {
                sensorPolarity[i] = (lineState == HIGH);
            } else {
                // Error: Sensor didn't change. Default to HIGH=Line?
                // Or keep previous guess.
            }
        }
    }

    // Returns virtual position 0-4000.
    // Returns -1 if line is lost (no sensors active).
    int16_t readPosition() {
        uint32_t weightedSum = 0;
        uint16_t sum = 0;
        
        for (int i = 0; i < 5; i++) {
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
    
    // Helper to check if line is lost
    bool isLineLost() {
        return readPosition() == -1;
    }
};

#endif
