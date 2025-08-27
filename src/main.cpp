#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- LCD Pin Definitions ---
#define LCD_ADDRESS 0x27 // Verify your display's I2C address
#define LCD_COLUMNS 16   // Number of columns on the display
#define LCD_ROWS 2       // Number of rows on the display

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// --- STM32 Black Pill Pin Definitions ---
#define STEP1_PIN PA2    // Motor 1 Step Pin (DM860 IN PUL+)
#define DIR1_PIN PA3     // Motor 1 Direction Pin (DM860 IN DIR+)
#define STEP2_PIN PA4    // Motor 2 Step Pin (DM860 IN PUL+)
#define DIR2_PIN PA5     // Motor 2 Direction Pin (DM860 IN DIR+)
#define RELAY_PIN PA6    // Relay Output
#define BUTTON_PIN PA7   // Start/Stop Button

#define LIMIT_A_PIN PB0 // Motor 1 Start Limit Switch
#define LIMIT_B_PIN PB1 // Motor 1 End Limit Switch
#define LIMIT_C_PIN PA8 // Motor 2 Start Limit Switch
#define LIMIT_D_PIN PA9 // Motor 2 End Limit Switch

#define POT1_PIN PA0 // Motor 1 Speed Potentiometer
#define POT2_PIN PA1 // Motor 2 Speed Potentiometer

// --- Stepper Motors ---
AccelStepper stepper1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);

// --- State Variables ---
enum State {
    STOPPED,
    RUNNING_FORWARD,
    RUNNING_BACK,
    PAUSED,
    STATO_ERRORE,
    HOMING
};
State state = STOPPED;
bool relayActive = false;
bool buttonPressed = false;
bool homingDone = false;
int velocita1 = 500;
int velocita2 = 500;

// --- Display Utilities ---
void updateDisplay(const char *msg1, const char *msg2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(msg1);
    lcd.setCursor(0, 1);
    lcd.print(msg2);
}

void showError(const char *msg) {
    updateDisplay("Errore!", msg);
    stepper1.stop();
    stepper2.stop();
    digitalWrite(RELAY_PIN, LOW);
    relayActive = false;
    state = STATO_ERRORE;
}

// --- Control Functions ---
void stopMotors() {
    stepper1.stop();
    stepper2.stop();
}

void activateRelay() {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = true;
}

void deactivateRelay() {
    digitalWrite(RELAY_PIN, LOW);
    relayActive = false;
}

bool checkLimitsOK() {
    // Returns true if limit switches are not both pressed at the same time
    return !(digitalRead(LIMIT_A_PIN) == LOW && digitalRead(LIMIT_B_PIN) == LOW) &&
           !(digitalRead(LIMIT_C_PIN) == LOW && digitalRead(LIMIT_D_PIN) == LOW);
}

void aggiornaVelocita() {
    int potValue1 = analogRead(POT1_PIN);
    int potValue2 = analogRead(POT2_PIN);
    velocita1 = map(potValue1, 0, 4095, 100, 2000);
    velocita2 = map(potValue2, 0, 4095, 100, 2000);
    stepper1.setMaxSpeed(velocita1);
    stepper2.setMaxSpeed(velocita2);
}

void homingMotors() {
    const unsigned long HOMING_TIMEOUT = 10000; // 10 seconds
    unsigned long startTime;

    updateDisplay("Homing", "Motore 1 (A)...");
    stepper1.setMaxSpeed(400);
    stepper1.setAcceleration(200);
    startTime = millis();
    while (digitalRead(LIMIT_A_PIN) == HIGH) {
        stepper1.moveTo(stepper1.currentPosition() - 1000);
        stepper1.run();
        if (millis() - startTime > HOMING_TIMEOUT) {
            showError("Timeout A");
            return;
        }
    }
    stepper1.setCurrentPosition(0);

    updateDisplay("Homing", "Motore 2 (C)...");
    stepper2.setMaxSpeed(400);
    stepper2.setAcceleration(200);
    startTime = millis();
    while (digitalRead(LIMIT_C_PIN) == HIGH) {
        stepper2.moveTo(stepper2.currentPosition() - 1000);
        stepper2.run();
        if (millis() - startTime > HOMING_TIMEOUT) {
            showError("Timeout C");
            return;
        }
    }
    stepper2.setCurrentPosition(0);

    homingDone = true;
    state = STOPPED;
    updateDisplay("Homing", "Completato!");
    delay(1000);
}

void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);
    pinMode(LIMIT_C_PIN, INPUT_PULLUP);
    pinMode(LIMIT_D_PIN, INPUT_PULLUP);
    pinMode(POT1_PIN, INPUT_ANALOG);
    pinMode(POT2_PIN, INPUT_ANALOG);

    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin();

    lcd.init();
    lcd.backlight();
    
    // Initial display message
    updateDisplay("ClickUp Stepper", "Pronto");
    delay(2000);
    
    // New message after 5 seconds
    updateDisplay("Eseguire Homing", "Premendo Start");
    
    stepper1.setMaxSpeed(velocita1);
    stepper1.setAcceleration(500);
    stepper2.setMaxSpeed(velocita2);
    stepper2.setAcceleration(500);
}

void loop() {
    aggiornaVelocita();

    // Start/Stop button handling with debounce
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
        buttonPressed = true;
        
        if (!checkLimitsOK()) {
            showError("Finecorsa bloccati");
        } else {
            if (state == STOPPED) {
                if (!homingDone) {
                    state = HOMING;
                } else {
                    state = RUNNING_FORWARD;
                    activateRelay();
                    updateDisplay("Ciclo avviato", "");
                }
            } else if (state == PAUSED) {
                state = RUNNING_FORWARD;
                activateRelay();
                updateDisplay("Ciclo riavviato", "");
            } else if (state == RUNNING_FORWARD || state == RUNNING_BACK) {
                state = PAUSED;
                stopMotors();
                deactivateRelay();
                updateDisplay("Ciclo in pausa", "");
            }
        }
        delay(300); // Debounce
    }
    if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonPressed = false;
    }

    switch (state) {
        case HOMING:
            homingMotors();
            break;

        case RUNNING_FORWARD: {
            String msg2_forward = "V1: " + String(velocita1) + " V2: " + String(velocita2);
            updateDisplay("Avanti", msg2_forward.c_str());

            // Motor 1 Logic: Move from A to B
            if (digitalRead(LIMIT_B_PIN) == HIGH) {
                stepper1.moveTo(10000);
                stepper1.run();
            } else {
                stepper1.stop();
                deactivateRelay();
                state = RUNNING_BACK;
                updateDisplay("Inversione", "Ritorno...");
            }

            // Motor 2 Logic: Move forward to LIMIT_D
            stepper2.moveTo(stepper2.currentPosition() + 10000);
            stepper2.run();
            if (digitalRead(LIMIT_D_PIN) == LOW) {
                stepper2.stop();
                stepper2.moveTo(stepper2.currentPosition() - 500); // Back up slightly
                state = RUNNING_BACK;
                updateDisplay("Inversione a D", "");
            }
            break;
        }

        case RUNNING_BACK: {
            String msg2_back = "Pos1: " + String(stepper1.currentPosition()) + " Pos2: " + String(stepper2.currentPosition());
            updateDisplay("Indietro", msg2_back.c_str());

            // Motor 1 Logic: Move from B to A
            if (digitalRead(LIMIT_A_PIN) == HIGH) {
                stepper1.moveTo(0);
                stepper1.run();
            } else {
                stepper1.stop();
                stepper2.stop();
                state = STOPPED;
                updateDisplay("Ciclo", "Completato!");
            }

            // Motor 2 Logic: Move backward to LIMIT_C
            stepper2.moveTo(stepper2.currentPosition() - 10000);
            stepper2.run();
            if (digitalRead(LIMIT_C_PIN) == LOW) {
                stepper2.stop();
                stepper2.moveTo(stepper2.currentPosition() + 500); // Move forward slightly
                state = RUNNING_FORWARD;
                updateDisplay("Inversione a C", "");
            }
            break;
        }

        case PAUSED:
            break;

        case STOPPED:
            break;

        case STATO_ERRORE:
            break;
    }
}