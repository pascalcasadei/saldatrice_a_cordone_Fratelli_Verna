#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Display Pin Definitions ---
#define OLED_RESET -1 // Reset pin
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_ADDRESS 0x3C // I2C address for the OLED (Found with scanner!)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.print(msg1);
    
    display.setCursor(0, 16); // Move down a line (16 pixels)
    display.print(msg2);
    
    display.display(); // Push buffer to screen
}

void showError(const char *msg) {
    display.clearDisplay();
    display.setTextSize(2); // Use a bigger font for errors
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Errore!");
    display.setTextSize(1);
    display.println(msg);
    display.display();
    
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

    // OLED display initialization
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        // Handle initialization error (e.g., stuck in a loop)
        for(;;); 
    }
    
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

            // --- INIZIO LOGICA CORRETTA ---
            // Motor 1 Logic: Move from B to A. This is the main return loop.
            if (digitalRead(LIMIT_A_PIN) == HIGH) {
                stepper1.moveTo(0);
                stepper1.run();
            } else {
                stepper1.stop();
                stepper2.stop(); // Stop motor 2 as well
                state = STOPPED; // End the cycle
                updateDisplay("Ciclo", "Completato!");
            }

            // Motor 2 Logic: Move backward to LIMIT_C and stop
            if (digitalRead(LIMIT_C_PIN) == HIGH) {
                stepper2.moveTo(stepper2.currentPosition() - 10000);
                stepper2.run();
            } else {
                stepper2.stop(); // Stop here and wait
                // The state will NOT change, it will wait for motor 1 to finish.
            }
            // --- FINE LOGICA CORRETTA ---

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