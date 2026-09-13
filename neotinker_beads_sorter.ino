#include <Arduino.h>
#include <Adafruit_AS7341.h>
#include <ESP32Servo.h>
#include <TM1637.h>
#include <LittleFS.h>


// =========================
// HW PINS
// =========================

//Display pins
constexpr int CLK = 4;
constexpr int DIO = 5;

constexpr int BUTTON_PIN = 15;
constexpr int IR_LED = 26;

constexpr int SERVO1_PIN = 16;
constexpr int SERVO2_PIN = 17;
constexpr int SERVO3_PIN = 18;
constexpr int SERVO4_PIN = 19;

// ULN2003 Driver Pins
constexpr int IN1 = 27;
constexpr int IN2 = 14;
constexpr int IN3 = 12;
constexpr int IN4 = 13;

// =========================
// SW constants
// =========================

constexpr int SERVO_TRUE = 140;
constexpr int SERVO_FALSE = 45;

constexpr int N = 11;
constexpr int NUM_CLASSES = 15;
constexpr int MAX_SAMPLES = 20;

// Stepper Motor Constants
constexpr int STEPS_PER_REV = 2048;

TM1637 display;

// =========================
// SERVOS
// =========================

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// =========================
// AS7341 SENSOR
// =========================

Adafruit_AS7341 as7341;


hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastServoChange = 0;

class StepperMotor {
private:
    int pins[4];
    volatile int stepIndex = 0;
    volatile unsigned long stepDelay = 2000;
    volatile bool running = false;
    volatile bool clockwise = true;

    const int stepSequence[8][4] = {
        {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
        {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
    };

public:
    StepperMotor(int p1, int p2, int p3, int p4) {
        pins[0] = p1;
        pins[1] = p2;
        pins[2] = p3;
        pins[3] = p4;
    }

    void begin() {
        for (int i = 0; i < 4; i++) {
            pinMode(pins[i], OUTPUT);
            digitalWrite(pins[i], LOW);
        }
    }

    void setSpeed(float rpm) {
        if (rpm > 0) {
            stepDelay = (60UL * 1000000UL) / (4096.0 * rpm);
            if (timer) timerAlarm(timer, stepDelay, true, 0);
        }
    }

    void setDirection(bool cw) { clockwise = cw; }
    void start() { running = true; }

    void stop() {
        running = false;
        for (int i = 0; i < 4; i++)
            digitalWrite(pins[i], 0);

        servo1.detach();
        servo2.detach();
        servo3.detach();
        servo4.detach();
    }

    void doStep() {
        if (!running) return;

        if (clockwise) stepIndex = (stepIndex + 1) % 8;
        else stepIndex = (stepIndex + 7) % 8;

        for (int i = 0; i < 4; i++)
            digitalWrite(pins[i], stepSequence[stepIndex][i]);
    }

    unsigned long getStepDelay() { return stepDelay; }
};

StepperMotor motor(IN1, IN2, IN3, IN4);

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    motor.doStep();
    portEXIT_CRITICAL_ISR(&timerMux);
}


float trainingData[NUM_CLASSES][MAX_SAMPLES][N];
int sampleCount[NUM_CLASSES] = {0};



// =========================
// STATE MACHINE
// =========================

enum AppMode { MODE_STARTUP, MODE_TRAIN, MODE_WAIT, MODE_RUN };

AppMode appMode = MODE_STARTUP;

bool trainingMode = false;
int currentClass = 0;

float prevSpectrum[N];
bool hasPrev = false;

float bestSpectrum[N];
float bestScore = 999.0;

// Button debounce
volatile bool buttonTriggered = false;
volatile unsigned long lastInterruptTime = 0;
constexpr unsigned long DEBOUNCE_TIME = 350; // ms

void IRAM_ATTR buttonISR() {
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > DEBOUNCE_TIME) {
        buttonTriggered = true;
    }
    lastInterruptTime = interruptTime;
}


// =========================
// HELPERS
// =========================

void normalizeSpectrum(float *input, float *output) {

    float sum = 0;
    for (int i = 0; i < 8; i++) sum += input[i] / input[8];

    if (sum < 1e-6) sum = 1e-6;

    for (int i = 0; i < N-1; i++) {
        output[i] = input[i] / input[8];
    }
    output[N-1] = (log10f(input[8])) / 8.0f;
    output[8] = sum / 8;
}


float spectralDistance(float *a, float *b) {
    float sum = 0;
    for (int i = 0; i < N; i++) {
        float d = a[i] - b[i];
        sum += d * d;
    }
    return sqrt(sum);
}



int classify(float *input) {
    // Find 10 closest points from entire trainingData
    const int K = 10;
    float kDist[K];
    int kClass[K];
    for (int i = 0; i < K; i++) {
        kDist[i] = 999.0f;
        kClass[i] = -1;
    }

    for (int c = 0; c < NUM_CLASSES; c++) {
        for (int s = 0; s < sampleCount[c]; s++) {
            float dist = spectralDistance(input, trainingData[c][s]);
            // insert into sorted top-K list
            if (dist < kDist[K - 1]) {
                kDist[K - 1] = dist;
                kClass[K - 1] = c;
                // bubble up
                for (int i = K - 1; i > 0 && kDist[i] < kDist[i - 1]; i--) {
                    float td = kDist[i];
                    kDist[i] = kDist[i - 1];
                    kDist[i - 1] = td;
                    int tc = kClass[i];
                    kClass[i] = kClass[i - 1];
                    kClass[i - 1] = tc;
                }
            }
        }
    }

    // Count votes per class
    int votes[NUM_CLASSES] = {0};
    for (int i = 0; i < K; i++) {
        if (kClass[i] >= 0) votes[kClass[i]]++;
    }

    // Return class with more than 6 votes, else 0
    int res = NUM_CLASSES;
    for (int c = 0; c < NUM_CLASSES; c++) {
        Serial.print(votes[c]);
        Serial.print(",");
        if (votes[c] > 7) res = c;
    }
    Serial.println();
    return res;
}


void setServos(int value) {
    static int lastValue = -1;
    int changed = value ^ (lastValue < 0 ? ~value : lastValue); // bits that changed
    lastServoChange = millis();
    if (changed & 0x1) {
        servo1.attach(SERVO1_PIN);
        servo1.write((value & 0x1) ? SERVO_TRUE : SERVO_FALSE);
    }
    if (changed & 0x2) {
        servo2.attach(SERVO2_PIN);
        servo2.write((value & 0x2) ? SERVO_TRUE : SERVO_FALSE);
    }
    if (changed & 0x4) {
        servo3.attach(SERVO3_PIN);
        servo3.write((value & 0x4) ? SERVO_TRUE : SERVO_FALSE);
    }
    if (changed & 0x8) {
        servo4.attach(SERVO4_PIN);
        servo4.write((value & 0x8) ? SERVO_TRUE : SERVO_FALSE);
    }

    lastValue = value;
}


// =========================
// SETUP
// =========================

void setup() {
    Serial.begin(115200);

    if (!as7341.begin()) {
        Serial.println("AS7341 not found!");
    } else {
        as7341.setATIME(29);
        as7341.setASTEP(599);
        as7341.setGain(AS7341_GAIN_64X);
        as7341.setLEDCurrent(8);
        as7341.enableLED(false);
        Serial.println("AS7341 OK");
    }
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

    display.begin(CLK, DIO, 4); // 4 digits
    display.setBrightness(4); // 0–7
    //display.displayInt(1234);
    display.displayInt(6, true);

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
    }

    motor.begin();
    motor.setSpeed(10);
    motor.setDirection(false);
    //motor.start();

    // 1 MHz timer (1 tick = 1 microsecond)
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, motor.getStepDelay(), true, 0);

    pinMode(IR_LED, OUTPUT);
    // Servos - neutral position (90 degrees)
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);

    servo1.write(90);
    delay(100);
    servo2.write(90);
    delay(100);
    servo3.write(90);
    delay(100);
    servo4.write(90);
    digitalWrite(IR_LED, LOW);
    Serial.println("System ready");
}


// =========================
// FLASH STORAGE
// =========================

void saveTrainingToFlash() {
    display.displayInt(2222, true);

    File file = LittleFS.open("/training.bin", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    file.write((uint8_t *) trainingData, sizeof(trainingData));
    file.write((uint8_t *) sampleCount, sizeof(sampleCount));
    file.close();

    Serial.println("Training data saved to flash");
}

void loadTrainingFromFlash() {
    File file = LittleFS.open("/training.bin", FILE_READ);
    if (!file) {
        Serial.println("No training data found");
        return;
    }

    file.read((uint8_t *) trainingData, sizeof(trainingData));
    file.read((uint8_t *) sampleCount, sizeof(sampleCount));
    file.close();

    // Validate loaded sampleCount
    for (int c = 0; c < NUM_CLASSES; c++) {
        if (sampleCount[c] < 0 || sampleCount[c] > MAX_SAMPLES) {
            Serial.print("Resetting invalid sampleCount[");
            Serial.print(c);
            Serial.print("]: ");
            Serial.println(sampleCount[c]);
            sampleCount[c] = 0;
        }
    }


    Serial.println("Training data loaded from flash");
}

void dumpData() {
    for (int c = 0; c < NUM_CLASSES; c++) {
        Serial.print(sampleCount[c]);
        for (int s = 0; s < MAX_SAMPLES; s++) {
            for (int k = 0; k < N; k++) {
                Serial.print(",");
                Serial.print(trainingData[c][s][k], 6);
            }
        }
        Serial.println();
    }
}

void loadData() {
    Serial.println("Ready to receive data...");
    // We expect NUM_CLASSES lines of CSV
    for (int c = 0; c < NUM_CLASSES; c++) {
        // Read sampleCount[c]
        while(!Serial.available());
        String val = Serial.readStringUntil(',');
        sampleCount[c] = val.toInt();

        for (int s = 0; s < MAX_SAMPLES; s++) {
            for (int k = 0; k < N; k++) {
                while(!Serial.available());
                String val = Serial.readStringUntil(',');
                trainingData[c][s][k] = val.toFloat();
            }
        }
        // Consume the rest of the line (or newline)
        Serial.readStringUntil('\n'); 
    }
    saveTrainingToFlash();
    Serial.println("Data loaded and saved.");
}

// =========================
// LOOP
// =========================


void loop() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "DUMP") {
            dumpData();
        } else if (cmd == "LOAD") {
            loadData();
        }
    }

    bool btnPressed = buttonTriggered;
    buttonTriggered = false;
    digitalWrite(IR_LED, LOW);
    // ---- STARTUP: 5-second countdown ----
    if (appMode == MODE_STARTUP) {
        static bool startupInit = false;
        static unsigned long startupStart = 0;
        static int lastShown = -1;

        if (!startupInit) {
            startupStart = millis();
            startupInit = true;
            lastShown = -1;
            loadTrainingFromFlash();
        }

        unsigned long elapsed = millis() - startupStart;
        int remaining = 5 - (int) (elapsed / 1000);
        if (remaining < 0) remaining = 0;

        if (remaining != lastShown) {
            lastShown = remaining;
            display.displayInt(remaining, true);
        }

        if (btnPressed) {
            // Go to TRAIN mode
            appMode = MODE_TRAIN;
            trainingMode = true;
            currentClass = 0;
            setServos(currentClass);
            delay(300);
            motor.start();
            display.displayTwoInt(currentClass, sampleCount[currentClass], true);
            as7341.enableLED(true);
            for (int i = 0; i < NUM_CLASSES; i++) { sampleCount[i] = 0; }

            Serial.println("TRAIN MODE");
            return;
        }

        if (elapsed >= 5000) {
            // Go to WAIT mode
            appMode = MODE_WAIT;
            trainingMode = false;
            motor.stop();
            display.displayClear();
            Serial.println("WAIT MODE");
            return;
        }
        return;
    }

    // ---- TRAIN mode ----
    if (appMode == MODE_TRAIN) {
        if (btnPressed) {
            // NEXT
             currentClass++;
            if (currentClass >= NUM_CLASSES) {
                 saveTrainingToFlash();
                currentClass = 0;
                trainingMode = false;
                appMode = MODE_WAIT;
                motor.stop();
                as7341.enableLED(false);
                display.displayClear();
                Serial.println("WAIT MODE");
                return;
            }
            display.displayTwoInt(currentClass, sampleCount[currentClass], true);
            Serial.print("Reading color ");
            Serial.println(currentClass);
            setServos(currentClass);
        }
        // display refresh (in case sampleCount changed)
        static int lastClass = -1;
        static int lastCount = -1;
        if (currentClass != lastClass || sampleCount[currentClass] != lastCount) {
            lastClass = currentClass;
            lastCount = sampleCount[currentClass];
            display.displayTwoInt(currentClass, sampleCount[currentClass], true);
        }
    }

    // ---- WAIT mode ----
    if (appMode == MODE_WAIT) {
        if (btnPressed) {
            appMode = MODE_RUN;
            trainingMode = false;
            motor.start();
            Serial.println("RUN MODE");
        }
        return;
    }

    // ---- RUN mode ----
    if (appMode == MODE_RUN) {
        if (btnPressed) {
            appMode = MODE_WAIT;
            motor.stop();
            display.displayClear();
            Serial.println("WAIT MODE");
            return;
        }
    }

    as7341.setATIME(9);
    as7341.setASTEP(199);
    uint16_t readings[12];
    while (!as7341.readAllChannels(readings)) {
    };
    //  Serial.println(readings[11]);

if ( (millis()- lastServoChange)>800) {
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
}


    if (readings[11] < 1000) {

        digitalWrite(IR_LED, HIGH);
        delay(250);
        // servo1.detach();
        // servo2.detach();
        // servo3.detach();
        // servo4.detach();
        delay(10);

        as7341.setATIME(29);
        as7341.setASTEP(599);
        as7341.enableLED(true);
        while (!as7341.readAllChannels(readings)) {
        };
        as7341.enableLED(false);

        digitalWrite(IR_LED, LOW);
        float raw[10], norm[N];
        raw[0] = readings[0];
        raw[1] = readings[1];
        raw[2] = readings[2];
        raw[3] = readings[3];
        raw[4] = readings[6];
        raw[5] = readings[7];
        raw[6] = readings[8];
        raw[7] = readings[9];
        raw[8] = readings[10];
        raw[9] = readings[11];


        normalizeSpectrum(raw, norm);

        uint16_t maxVal = 0;
        for (int i = 0; i < 12; i++) {
            if (readings[i] > maxVal) maxVal = readings[i];
        }
        if (trainingMode) {
            int id = sampleCount[currentClass];

            if (id < MAX_SAMPLES) {

                for (int i = 0; i < N; i++)
                    trainingData[currentClass][id][i] = norm[i];

                sampleCount[currentClass]++;

                Serial.print("Stored ");
                Serial.print(currentClass);
                Serial.print(" #");
                Serial.println(sampleCount[currentClass]);
            }
        } else {
            int cls = classify(norm);
            if (appMode == MODE_RUN) {
                display.displayInt(cls, true);

                // Serial.print("CLASS: ");
                // Serial.print( cls );
                // Serial.print(" -> ");

                setServos(cls);
                for (int i = 0; i < N; i++) {
                    Serial.print(norm[i], 6);
                    if (i < N - 1) Serial.print(",");
                }
                Serial.println();
            }
        }
        as7341.setATIME(9);
        as7341.setASTEP(199);
        uint16_t readings[12];

        for ( int i =0; i<30;i++) {
            while (!as7341.readAllChannels(readings)) {
            };
            if (readings[11] > 1000) {
                break;
            }
        }
    }
}
