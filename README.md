````markdown
# Advanced Line Follower Robot

High-performance line follower robot based on **Arduino Nano**, **TB6612FNG** motor driver, and a **5-sensor digital line array**.  
The firmware implements a full **PID controller**, **history-based recovery**, and a **two-step calibration** routine designed for competition-grade tracking.

---

## 1. Project Overview

This project implements:

- A modular, reusable firmware architecture for a differential-drive line follower.
- Robust line detection using a **5-channel digital IR sensor array** (no analog thresholds).
- A **PID-based controller** for fast and smooth tracking.
- A **history buffer + recovery mode** to re-acquire the line when it is temporarily lost.
- A simple user interface with:
  - 2 push buttons (Calibrate / Start)
  - 1 status LED
  - Optional serial debugging for tuning and diagnostics.

---

## 2. Hardware Overview

- **Microcontroller**: Arduino Nano (ATmega328P)
- **Motor Driver**: TB6612FNG (or SparkFun TB6612 breakout)
- **Motors**: 2 × N20 DC gear motors (left / right)
- **Line Sensor Array**: 5 × digital IR line sensors in a straight row
- **User Interface**:
  - Button 1: Calibration (two-step: background & line)
  - Button 2: Start / Run
  - LED: Built-in or external status LED
- **Power**:
  - Motor supply (VM) from battery (e.g., 7.4 V LiPo)
  - 5 V logic for Arduino and TB6612 VCC
  - Common ground between all modules

> Pin mapping is defined inside the code (typically in `MotorController` and the main `.ino` file).  
> Always connect your hardware according to those definitions.

---

## 3. Features

- **Digital sensor array only** (no analog thresholding).
- **Two-step calibration**:
  - Step 1: Background (no line)
  - Step 2: Line (on the track line)
- **Per-sensor polarity detection**:
  - Determines if “line” corresponds to HIGH or LOW for each sensor.
- **Virtual position** (0–4000):
  - Sensor 0 ↦ 0, Sensor 1 ↦ 1000, ..., Sensor 4 ↦ 4000
- **Full PID controller**:
  - Error = 2000 − Position (target at center)
  - Output used as a **turn command** (differential drive)
- **History buffer**:
  - Stores recent errors and line-presence flags
  - Used to decide recovery direction (left/right)
- **Recovery mode** with timeout:
  - Spins in direction of last known line
  - Times out to ERROR state (no infinite spinning)
- **Serial debug mode**:
  - Prints sensor pattern, position, error, PID terms, motor speeds, and state

---

## 4. Repository Structure

Typical structure (file names may vary slightly):

- `line_follower.ino`  
  Main Arduino sketch:
  - Initializes hardware and objects
  - Handles setup sequence, buttons, LED, and main loop

- `Robot.h`  
  Contains the `Robot` class:
  - High-level state machine (IDLE, CALIB_BG, CALIB_LINE, READY, RUNNING, RECOVERY, ERROR)
  - PID logic
  - History buffer
  - Recovery behavior
  - Debug printing

- `LineSensor.h`  
  Contains the `LineSensor` class:
  - Manages the 5 digital sensor pins
  - Two-step calibration (background & line)
  - Per-sensor polarity & active/inactive mask
  - Virtual position computation (0–4000)
  - Raw pattern output for debugging

- `MotorController.h`  
  Contains the `MotorController` class:
  - Wraps TB6612FNG (or SparkFun TB6612 library)
  - Drives left/right motors with signed speed commands
  - Handles clamping, forward/backward, and brake/stop

---

## 5. System Architecture

### 5.1 High-Level Blocks

1. **LineSensor**
   - Reads 5 digital IR sensors (left → right).
   - After calibration, knows for each sensor:
     - Is the line detected as HIGH or LOW?
     - Is the sensor considered active or inactive?
   - Provides:
     - `readPosition()` → returns virtual position (0–4000) or −1 if no line.
     - `getRawPattern()` → 5-bit pattern of active sensors for debugging.

2. **MotorController**
   - Accepts `leftSpeed` and `rightSpeed` (signed).
   - Clamps to safe PWM range.
   - Translates direction and speed into TB6612 control signals.

3. **Robot**
   - Uses `LineSensor` and `MotorController`.
   - Implements:
     - Two-step calibration flow
     - Main run loop (RUNNING)
     - Recovery when the line is lost
     - State machine + LED + buttons
     - PID control

---

### 5.2 LineSensor: Digital Reading & Virtual Position

- On every read:
  - Each sensor `i` is sampled as HIGH or LOW.
  - Using calibration, we know if “line” means HIGH or LOW for that sensor.
  - Compute `active_i`:
    - `active_i = 1` if sensor sees line, `0` otherwise.
- Virtual position:
  - Weights: `w = {0, 1000, 2000, 3000, 4000}`.
  - If at least one sensor is active:

    \[
    position = \frac{\sum_i (w_i \cdot active_i)}{\sum_i active_i}
    \]

  - Range: 0 (far left) to 4000 (far right).
  - Center of track ≈ 2000.
- If no sensor is active:
  - `position = -1` and we consider the line “lost”.

---

### 5.3 PID Controller & Motor Mixing

- Target line position: `TargetPosition = 2000`.
- Error:

  ```cpp
  error = TargetPosition - position;  // i.e. 2000 - position
````

* PID terms:

  * `P = Kp * error`
  * `I = Ki * integral` (integral with anti-windup clamp)
  * `D = Kd * derivative` (difference between current and last error)

* PID output:

  ```cpp
  output = P + I + D;
  ```

* Turn limiting:

  * `output` (turn command) is clamped to `[-maxTurn, +maxTurn]`.

* Motor speed mixing:

  ```cpp
  leftSpeed  = BASE_SPEED - output;
  rightSpeed = BASE_SPEED + output;
  ```

* Intuition:

  * If the line is to the left ⇒ `position < 2000` ⇒ `error > 0`:

    * `output > 0` ⇒ leftSpeed decreases, rightSpeed increases ⇒ robot turns left.

---

### 5.4 History Buffer & Recovery

* The Robot stores recent samples in circular buffers:

  * `errorHistory[]`
  * `linePresentHistory[]`
* On each valid line reading:

  * Append error and mark linePresent = true.
* When line is lost (position = −1):

  * Enter RECOVERY state.
  * Look back through the last N samples (e.g., last 5) for linePresent = true.
  * Compute an average error over recent valid entries:

    * `avgError > 0` → last line was on the left ⇒ `lastValidDirection = LEFT`.
    * `avgError < 0` → last line was on the right ⇒ `lastValidDirection = RIGHT`.
* In RECOVERY state:

  * Robot spins or arcs in `lastValidDirection` at reduced speed.
  * Uses a timeout (e.g., 2–3 seconds).
  * If the line is re-acquired (position != −1) ⇒ return to RUNNING.
  * If timeout expires without re-acquiring the line:

    * Enter ERROR state:

      * Motors stopped
      * LED indicates error
      * Wait for user action (e.g., recalibrate or restart).

---

### 5.5 State Machine

Typical states:

* `IDLE / READY`:

  * After power-on or reset.
  * Robot is not moving, waiting for commands.
* `CALIB_BG`:

  * Background calibration step.
  * LED = solid ON.
  * User places robot on background (no line) and presses Calibrate.
* `CALIB_LINE`:

  * Line calibration step.
  * LED = blinking.
  * User places robot over the line and presses Calibrate again.
* `RUNNING`:

  * Normal line following using PID.
* `RECOVERY`:

  * Line lost, using history to spin and search for line.
* `ERROR`:

  * Fatal or unrecoverable condition (e.g., recovery timeout or not enough active sensors).
  * Motors stopped, LED indicates error.

---

## 6. Installation & Build

### 6.1 Prerequisites

* **Arduino IDE** (or PlatformIO)
* **Board**: Arduino Nano (ATmega328P)
* **Libraries**:

  * [SparkFun TB6612FNG](https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library) (or equivalent) if used in `MotorController.h`.
* Correct **board** and **port** selected in the IDE.

### 6.2 Building & Uploading

1. Open `line_follower.ino` in Arduino IDE.
2. Make sure all header files (`Robot.h`, `LineSensor.h`, `MotorController.h`) are in the same folder.
3. Select the correct board and COM port.
4. Click **Verify** to compile.
5. Click **Upload** to flash the firmware to the Arduino Nano.

---

## 7. Calibration Procedure (Engineering View)

The calibration is **two-step** and uses the Calibrate button and LED:

1. **Background Step**

   * State: `CALIB_BG`, LED = solid ON.
   * Place the robot so all sensors see **only background** (no black line).
   * Press the Calibrate button.
   * The firmware:

     * Reads each sensor multiple times.
     * Determines `backgroundState[i]` for each sensor.

2. **Line Step**

   * State: `CALIB_LINE`, LED = blinking.
   * Place the robot so all sensors are fully on the **line**.
   * Press the Calibrate button again.
   * The firmware:

     * Reads each sensor multiple times.
     * Determines `lineState[i]` for each sensor.
     * For each sensor:

       * If `lineState[i] != backgroundState[i]`:

         * Mark sensor as active.
         * Determine whether “line” is HIGH or LOW (polarity).
       * If `lineState[i] == backgroundState[i]`:

         * Mark sensor as inactive.
         * Print a warning over Serial.

3. **Minimum Active Sensors Check**

   * After calibration, if the number of active sensors is too low (e.g., < 3):

     * Stay in ERROR/READY and print an error message:

       * You need to mechanically adjust the sensor array (height, angle) and calibrate again.

---

## 8. PID Tuning Guidelines

* Start with relatively low **BASE_SPEED** (e.g., 120–150).
* Begin with:

  * Moderate `Kp`
  * Moderate `Kd`
  * `Ki = 0` initially
* On a straight track:

  * If the robot is sluggish and doesn’t correct quickly → increase `Kp`.
  * If it oscillates left/right quickly → reduce `Kp` or increase `Kd`.
* Once stable at lower speeds:

  * Gradually increase `BASE_SPEED`.
  * Adjust `Kp` and `Kd` again.
* Add a small `Ki` only if you see a consistent bias (robot consistently off-center on long straights).

Use the Serial debug output to monitor:

* `POS`, `ERR`, `P`, `I`, `D`, `OUT`, `L`, `R`, and `MODE`.

---

## 9. Debugging & Serial Output

With DEBUG enabled:

* On calibration:

  * For each sensor:

    * Background state, line state, polarity, and status.
    * Warnings if a sensor does not change between background and line.
* During run:

  * Position (0–4000)
  * Error
  * PID components
  * Motor speeds (left/right)
  * Current state (RUNNING / RECOVERY / ERROR)
  * Sensor pattern (e.g., `pattern = 0b10110`)

This helps you verify that the sensors, PID, and motors behave as expected.

---

## 10. Safety Notes

* Always test at low speed first.
* Keep hands away from wheels and moving parts during tests.
* Make sure wiring and battery connections are secure before placing the robot on the track.
* If the robot behaves unexpectedly:

  * Press reset or power off immediately.
  * Check calibration, wiring, and tuning.

---

# 11. User Guide – Simple Usage (English + Arabic)

This section is intended for non-technical users.
Follow the steps below to use the robot safely.

---

## 11.1 Before You Start

1. **Charge the battery and connect it to the robot.**
   *اشحن البطارية كويس ووصلها بالروبوت.*

2. **Place the robot on a flat surface near the track, but not on the line yet.**
   *حط الروبوت على سطح مستوي قريب من التراك، بس مش فوق الخط لسه.*

3. **Make sure the ON/OFF switch is OFF before you move the robot.**
   *اتأكد إن زرار الباور على وضع الإغلاق (OFF) قبل ما تحرك الروبوت.*

---

## 11.2 Power On

1. **Turn the power switch ON.**
   *حوّل زرار الباور إلى وضع التشغيل (ON).*

2. **Wait a few seconds until the robot LED shows it is ready (according to your firmware behavior).**
   *استنى ثواني لحد ما لمبة الروبوت توضح إنه جاهز (حسب طريقة الإشارة في الكود).*

---

## 11.3 Step 1 – Background Calibration

1. **Place the robot so that all sensors see only the floor (no black line).**
   *حط الروبوت بحيث كل الحساسات شايفة الأرض بس من غير أي خط أسود.*

2. **Check that the LED is in “background” mode (solid ON if implemented that way).**
   *اتأكد إن اللمبة في وضع الكالِبرايشن للخلفية (منورة ثابت لو معمول كده في الكود).*

3. **Press the Calibrate button once.**
   *اضغط على زر الكالِبرايشن مرة واحدة.*

4. **Wait until the LED or Serial output indicates the background step is done.**
   *استنى لحد ما اللمبة أو شاشة الـ Serial توضح إن خطوة الخلفية خلصت.*

---

## 11.4 Step 2 – Line Calibration

1. **Move the robot so that all 5 sensors are directly over the black line.**
   *حرك الروبوت بحيث الخمس حساسات يكونوا فوق الخط الأسود مباشرة.*

2. **Check that the LED is in “line” calibration mode (blinking if implemented that way).**
   *اتأكد إن اللمبة في وضع كالِبرايشن الخط (بتغمز لو معمول كده في الكود).*

3. **Press the Calibrate button again.**
   *اضغط على زر الكالِبرايشن مرة تانية.*

4. **Wait until calibration is completed.**
   *استنى لحد ما الكالِبرايشن تخلص.*

5. **If you are using the Serial Monitor, confirm that all sensors show “OK” (not “No Change”).**
   *لو بتستخدم شاشة الـ Serial، اتأكد إن كل الحساسات مكتوب قدامها “OK” ومفيش “No Change”.*

6. **If any sensor is shown as problematic, adjust the sensor height/angle and repeat calibration.**
   *لو أي حساس طالع فيه مشكلة، عدّل ارتفاعه أو زاويته وكرر الكالِبرايشن.*

---

## 11.5 Starting the Race (Line Following)

1. **Place the robot at the start of the track, centered on the line.**
   *حط الروبوت عند بداية التراك، والخط في نص الحساسات تقريبًا.*

2. **Make sure no one is in front of the robot and the track is clear.**
   *اتأكد إن مفيش حد قدام الروبوت وإن التراك فاضي.*

3. **Press the Start button.**
   *اضغط على زر Start.*

4. **The robot should begin to move and follow the line automatically.**
   *المفروض الروبوت يبدأ يتحرك ويتبع الخط لوحده.*

5. **Observe the motion; if it moves too slowly or too fast, an engineer can adjust the speed and PID values in the code.**
   *تابع حركة الروبوت؛ لو بطئ أو سريع قوي، مهندس يقدر يعدل السرعة وقيم الـ PID في الكود.*

---

## 11.6 When Things Go Wrong

1. **If the robot leaves the track or behaves strangely, turn OFF the power immediately.**
   *لو الروبوت خرج برا التراك أو بيتصرف بغرابة، اقفل الباور فورًا.*

2. **In some firmware versions, the robot may stop and blink the LED when it cannot find the line.**
   *في بعض الإصدارات، الروبوت هيقف ويلمع اللمبة بسرعة لما ميعرفش يلاقي الخط.*

3. **If that happens, place the robot back on the line and either press Start again or power cycle (depending on the design).**
   *لو ده حصل، رجّع الروبوت فوق الخط واضغط Start تاني أو اقفل وافتح الباور (حسب ما الكود متصمم).*

4. **If problems continue, ask a technical person to check the sensors, calibration, and battery.**
   *لو المشكلة مستمرة، خلي حد تقني يبص على الحساسات، والكالِبرايشن، والبطارية.*

---

## 11.7 Storage and Handling

1. **Turn OFF the robot when not in use.**
   *اقفل الباور لما تكون مش محتاج الروبوت.*

2. **Disconnect or switch off the battery to avoid draining it.**
   *افصل أو اقفل البطارية عشان ما تفضاش.*

3. **Avoid dropping the robot or pressing on the sensor array.**
   *بلاش ترمي الروبوت أو تضغط جامد على جزء الحساسات.*

4. **Store in a dry, dust-free place.**
   *احفظ الروبوت في مكان جاف وبعيد عن التراب.*

---

*End of README*

```
```
