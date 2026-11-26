#include <Arduino.h>
#include <Adafruit_AS7341.h>
#include <ESP32Servo.h>
#include <TM1637.h>
#include <LittleFS.h>
#include "color_classifier.h"


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

constexpr int SERVO_TRUE = 135; //140
constexpr int SERVO_FALSE = 45;

constexpr int N = COLOR_FEATURE_COUNT;
constexpr int NUM_CLASSES = 15;
constexpr int MAX_SAMPLES = 30;

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

        // servo1.detach();
        // servo2.detach();
        // servo3.detach();
        // servo4.detach();
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

int classify(float *input) {
    int result = classifySpectrum(input, trainingData, sampleCount);
    Serial.print("CLASS: ");
    Serial.println(result);
    return result;
}


void setServos(int value) {
    static int lastValue = -1;
    int changed = value ^ (lastValue < 0 ? ~value : lastValue); // bits that changed
    lastServoChange = millis();
//    if (changed & 0x1) {
        servo1.attach(SERVO1_PIN);
        servo1.write((value & 0x1) ? SERVO_TRUE : SERVO_FALSE);
  //  }
  //  if (changed & 0x2) {
        servo2.attach(SERVO2_PIN);
        servo2.write((value & 0x2) ? SERVO_TRUE : SERVO_FALSE);
    // }
    // if (changed & 0x4) {
        servo3.attach(SERVO3_PIN);
        servo3.write((value & 0x4) ? SERVO_TRUE : SERVO_FALSE);
    // }
    // if (changed & 0x8) {
        servo4.attach(SERVO4_PIN);
        servo4.write((value & 0x8) ? SERVO_FALSE : SERVO_TRUE);
//    }

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

    const TrainingFileHeader header = {
        TRAINING_FILE_MAGIC,
        TRAINING_FILE_VERSION,
        NUM_CLASSES,
        MAX_SAMPLES,
        N
    };
    const bool saved = file.write((uint8_t *) &header, sizeof(header)) == sizeof(header) &&
                       file.write((uint8_t *) trainingData, sizeof(trainingData)) == sizeof(trainingData) &&
                       file.write((uint8_t *) sampleCount, sizeof(sampleCount)) == sizeof(sampleCount);
    file.close();

    Serial.println(saved ? "Training data saved to flash" : "Failed to save training data");
}

void loadTrainingFromFlash() {
    File file = LittleFS.open("/training.bin", FILE_READ);
    if (!file) {
        Serial.println("No training data found");
        return;
    }

    TrainingFileHeader header;
    int loadedSampleCount[NUM_CLASSES];
    bool loaded = trainingFileSizeIsValid(file.size(), sizeof(TrainingFileHeader),
                                          sizeof(trainingData), sizeof(sampleCount)) &&
                  file.read((uint8_t *) &header, sizeof(header)) == sizeof(header) &&
                  trainingHeaderIsCompatible(header, NUM_CLASSES, MAX_SAMPLES, N) &&
                  file.read((uint8_t *) trainingData, sizeof(trainingData)) == sizeof(trainingData) &&
                  file.read((uint8_t *) loadedSampleCount, sizeof(loadedSampleCount)) == sizeof(loadedSampleCount) &&
                  trainingSampleCountsAreValid(loadedSampleCount, NUM_CLASSES, MAX_SAMPLES);
    file.close();

    if (!loaded) {
        for (int c = 0; c < NUM_CLASSES; c++) sampleCount[c] = 0;
        Serial.println("Training data is incompatible or invalid; retrain colors");
        return;
    }

    for (int c = 0; c < NUM_CLASSES; c++) sampleCount[c] = loadedSampleCount[c];

    Serial.println("Training data loaded from flash");
}
void dumpData()
{
    File f = LittleFS.open("/training.bin", FILE_READ);

    uint32_t size = f.size();

    Serial.write((uint8_t *)&size, sizeof(size));

    uint8_t buf[256];

    while (f.available()) {
        size_t n = f.read(buf, sizeof(buf));
        Serial.write(buf, n);
    }

    f.close();
}

void loadData(uint32_t size)
{
    File f = LittleFS.open("/training.bin", FILE_WRITE);

    uint8_t buf[256];
    uint32_t received = 0;

    while (received < size) {

        int wanted = min((uint32_t)sizeof(buf), size - received);

        int n = Serial.readBytes((char*)buf, wanted);

        if (n > 0) {
            f.write(buf, n);
            received += n;
        }
    }

    f.close();

    Serial.println("OK");
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
        }
        else if (cmd.startsWith("LOAD ")) {
            loadData(cmd.substring(5).toInt());
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
            //display.displayClear();
            char message[] = "SLEP";
            display.displayPChar(message);

            Serial.println("WAIT MODE");
            setServos(15);
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
            char message[] = "RUN";
            display.displayPChar(message);
            Serial.println("RUN MODE");
        }
        return;
    }

    // ---- RUN mode ----
    if (appMode == MODE_RUN) {
        if (btnPressed) {
            appMode = MODE_WAIT;
            motor.stop();
            //display.displayClear();
            char message[] = "SLEP";
            display.displayPChar(message);
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

if ( (millis()- lastServoChange)>1000) {
    // servo1.detach();
    // servo2.detach();
    // servo3.detach();
    // servo4.detach();
}


    if (readings[11] < 1000) {

        digitalWrite(IR_LED, HIGH);
        delay(250);
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


        if (!normalizeSpectrum(raw, norm)) {
            Serial.println("Rejected invalid or saturated sensor reading");
            if (!trainingMode && appMode == MODE_RUN) {
                display.displayInt(NUM_CLASSES, true);
                setServos(NUM_CLASSES);
            }
        } else if (trainingMode) {
            Serial.print(currentClass);
            for (int i = 0; i < 10; i++) {
                Serial.print(",");
                Serial.print(raw[i], 0);
            }
            Serial.println();
            int id = sampleCount[currentClass];
            if (id < MAX_SAMPLES) {
                for (int i = 0; i < N; i++) {
                    trainingData[currentClass][id][i] = norm[i];
                }
                sampleCount[currentClass]++;
            }
        } else {
            int cls = classify(norm);
            if (appMode == MODE_RUN) {
                display.displayInt(cls, true);

                // Serial.print("CLASS: ");
                // Serial.print( cls );
                // Serial.print(" -> ");

                setServos(cls);
                // for (int i = 0; i < N; i++) {
                //     Serial.print(norm[i], 6);
                //     if (i < N - 1) Serial.print(",");
                // }
                // Serial.println();
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
