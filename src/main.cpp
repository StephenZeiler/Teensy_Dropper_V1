#include <Arduino.h>
#include "SlotObject.h"
#include "MachineState.h"
#include "EasyNextionLibrary.h"

// === Teensy 4.1 Serial2 for Nextion ===
// RX = pin 7, TX = pin 8
EasyNex myNex(Serial2);

MachineState machine;
long startTime;

// ============================================================
//                TEENSY 4.1 PIN MAP (AUTO-ASSIGNED)
// ============================================================

// --- Motor Driver Pins ---
const int stepPin   = 2;   // STEP -> NEMA34 Driver
const int dirPin    = 3;   // DIR  -> NEMA34 Driver
const int enablePin = 4;   // ENA  -> NEMA34 Driver

// --- Buttons (INPUT_PULLUP) ---
const int startButtonPin            = 5;
const int pauseButtonPin            = 6;
const int finishProductionButtonPin = 9;
const int speedButtonPin            = 10;
const int emptySlotsButtonPin       = 11;

// --- Supply Sensors (HIGH = Supply Low) ---
const int pipetSupplySensorPin = 12;
const int bulbSupplySensorPin  = 13;
const int capSupplySensorPin   = 14;

// --- Supply Lights + Buzzer (Active LOW) ---
const int pipetLowSupplyLight = 15;
const int bulbLowSupplyLight  = 16;
const int capLowSupplyLight   = 17;
const int lowSupplyBuzzer     = 18;

// --- General Sensors ---
const int homeSensorPin  = 19;
const int pipetTipSensor = 20;

// --- Bulb Sensors ---
const int bulbRamHomeSensorPin         = 21;
const int bulbInPreLoadPosSensorPin    = 22;
const int preLoadCylinderHomeSensorPin = 23;
const int bulbInCapSensor              = 24;

// --- Bulb Actuators ---
const int bulbPreLoadCylinder = 25;
const int bulbRamPin          = 26;

// --- Ejectors ---
const int dropperEjectPin = 27;
const int junkEjectorPin  = 28;

// --- Cap System ---
const int capInjectPin         = 29;
const int capInWheel           = 30;
const int capPositionSensorPin = 31;

// --- Empty Slot Sensor ---
const int slotEmptySensor = 32;

// --- Pipet System ---
const int pipetRamPin              = 33;
const int pipetTwisterPin          = 34;
const int pipetTwisterHomeSensorPin = 35;

// --- Low Air Pressure Sensor ---
const int lowAirSensorPin = 36;

// --- Speed Potentiometer ---
const int speedPotPin = A1;   // Teensy analog

// ============================================================
//                     MACHINE PARAMETERS
// ============================================================

const int TOTAL_STEPS = 200;

// 30% speed increase parameters
int MIN_STEP_DELAY = 31;     // 40 * 0.77
int MAX_STEP_DELAY = 616;    // 800 * 0.77
int ACCEL_STEPS    = 46;     // 60 * 0.77
int DECEL_STEPS    = 15;     // 20 * 0.77

unsigned long PAUSE_AFTER = 90000; // microseconds

const unsigned long PAUSE_MIN_US = 90000UL;
const unsigned long PAUSE_MAX_US = 400000UL;

const unsigned long PAUSE_LOW_SUPPLY = 2000000UL;

// --- Motor state ---
unsigned long lastStepTime = 0;
unsigned long pauseStartTime = 0;
int stepsTaken = 0;
bool isMoving = false;
bool stepHigh = false;
bool pauseRequested = false;
bool emptySlotsRequested = false;
bool finsihProdRequested = false;

// ============================================================
//                     BULB SYSTEM PARAMETERS
// ============================================================

const unsigned long PRELOAD_PULSE_US = 160000;

int currentHomePosition = 0;

// ============================================================
//                     PIPET SYSTEM STATE
// ============================================================

enum PipetState {
    PIPET_HOMING,
    PIPET_HOMING_COMPLETE,
    PIPET_RAM_EXTENDING,
    PIPET_RAM_RETRACTING,
    PIPET_TWISTER_ACTIVE
};

PipetState currentPipetState = PIPET_HOMING;
unsigned long pipetStateStartTime = 0;
bool twisterAtHome = false;

// ============================================================
//                     DROPPER EJECTION STATE
// ============================================================

enum DropperState {
    DROPPER_IDLE,
    DROPPER_EJECTING,
    DROPPER_RETRACTING
};
DropperState currentDropperState = DROPPER_IDLE;

// ============================================================
//                     CAP INJECTION STATE
// ============================================================

enum CapState {
    CAP_IDLE,
    CAP_INJECTING,
    CAP_RETRACTING
};
CapState currentCapState = CAP_IDLE;

// ============================================================
//                     BULB STATE MACHINE
// ============================================================

enum BulbState {
    BULB_IDLE,
    BULB_AIR_PUSHING,
    BULB_SEPARATING,
    BULB_RAM_EXTENDING,
    BULB_RAM_RETRACTING
};

BulbState currentBulbState = BULB_IDLE;

unsigned long bulbStateStartTime = 0;
float motorPausePercent;
bool motorDecelerated = false;

// ============================================================
//                     MACHINE SPEED MODE
// ============================================================

int motorSpeedMode = 0;
int lastSw0Val = -1;
const uint8_t SW0_PAGE_ID = 0;
const uint8_t SW0_COMPONENT_ID = 10;

// ============================================================
//                     SLOT TRACKING
// ============================================================

SlotObject slots[] = {
    SlotObject(0), SlotObject(1), SlotObject(2), SlotObject(3),
    SlotObject(4), SlotObject(5), SlotObject(6), SlotObject(7),
    SlotObject(8), SlotObject(9), SlotObject(10), SlotObject(11),
    SlotObject(12), SlotObject(13), SlotObject(14), SlotObject(15)
};

int slotIdCapInWheelInjection;
int slotIdCapInWheelConfirm;
int slotIdBulbPreLoad;
int slotIdBulbInjection;
int slotIdBulbInCapConfirm;
int slotIdPipetInjection;
int slotIdPipetConfirm;
int slotIdDropeprEjection;
int slotIdJunkEjection;
int slotIdJunkConfirm;
int slotIdFailedJunkEject;
int test;

// ============================================================
//           FUNCTIONS: Slot Assignment, Error Handling
// ============================================================

void setSlotIdByPosition(SlotObject slots[]) {
    for (int i = 0; i < 16; i++) {
        if (slots[i].getPosition() == 1) slotIdCapInWheelInjection = slots[i].getId();
        if (slots[i].getPosition() == 2) slotIdCapInWheelConfirm = slots[i].getId();
        if (slots[i].getPosition() == 4) test = slots[i].getId();
        if (slots[i].getPosition() == 5) slotIdBulbPreLoad = slots[i].getId();
        if (slots[i].getPosition() == 6) slotIdBulbInjection = slots[i].getId();
        if (slots[i].getPosition() == 6) slotIdBulbInCapConfirm = slots[i].getId();
        if (slots[i].getPosition() == 9) slotIdPipetInjection = slots[i].getId();
        if (slots[i].getPosition() == 10) slotIdPipetConfirm = slots[i].getId();
        if (slots[i].getPosition() == 13) slotIdDropeprEjection = slots[i].getId();
        if (slots[i].getPosition() == 14) slotIdJunkEjection = slots[i].getId();
        if (slots[i].getPosition() == 15) slotIdJunkConfirm = slots[i].getId();
        if (slots[i].getPosition() == 0) slotIdFailedJunkEject = slots[i].getId();
    }
}

bool hasConsecutiveErrors() {
    int slotCount = sizeof(slots) / sizeof(slots[0]);

    for (int i = 0; i < slotCount; ++i) {
        int i1 = (i + 1) % slotCount;
        int i2 = (i + 2) % slotCount;

        bool threeMissingCaps =
            slots[i].hasMissingCap() &&
            slots[i1].hasMissingCap() &&
            slots[i2].hasMissingCap();

        bool threeMissingPipets =
            slots[i].hasJunk() &&
            slots[i1].hasJunk() &&
            slots[i2].hasJunk();

        bool threeMissingBulbs =
            slots[i].hasMissingBulb() &&
            slots[i1].hasMissingBulb() &&
            slots[i2].hasMissingBulb();

        if (threeMissingCaps || threeMissingPipets || threeMissingBulbs) {
            if (threeMissingCaps){
                slots[i].setMissingCap(false);
                slots[i1].setMissingCap(false);
                slots[i2].setMissingCap(false);
                machine.hasConsecutiveCapErrors = true;
            }
            if (threeMissingPipets){
                slots[i].setJunk(false);
                slots[i1].setJunk(false);
                slots[i2].setJunk(false);
                machine.hasConsecutivePipetErrors = true;
            }
            if (threeMissingBulbs){
                slots[i].setMissingBulb(false);
                slots[i1].setMissingBulb(false);
                slots[i2].setMissingBulb(false);
                machine.hasConsecutiveBulbErrors = true;
            }

            slots[i].setError(true);
            slots[i1].setError(true);
            slots[i2].setError(true);

            return true;
        }
    }

    machine.hasConsecutiveBulbErrors = false;
    machine.hasConsecutiveCapErrors = false;
    machine.hasConsecutivePipetErrors = false;

    return false;
}

void setSlotErrors(SlotObject slots[]) {
    for (int i = 0; i < 16; i++) {
        if (slots[i].hasJunk() || slots[i].hasMissingBulb() || slots[i].hasMissingCap()) {
            slots[i].setError(true);
        }
    }
}

bool handleLowSupplies() {
    if (digitalRead(capSupplySensorPin) == HIGH) return true;
    if (digitalRead(bulbSupplySensorPin) == HIGH) return true;
    if (digitalRead(pipetSupplySensorPin) == HIGH) return true;
    return false;
}
void handleSupplyAlert()
{
    bool capLow = (digitalRead(capSupplySensorPin) == HIGH);
    bool bulbLow = (digitalRead(bulbSupplySensorPin) == HIGH);
    bool pipetLow = (digitalRead(pipetSupplySensorPin) == HIGH);

    // Active low outputs; LOW = ON when supply is low, HIGH = OFF otherwise
    digitalWrite(capLowSupplyLight, capLow ? LOW : HIGH);
    digitalWrite(bulbLowSupplyLight, bulbLow ? LOW : HIGH);
    digitalWrite(pipetLowSupplyLight, pipetLow ? LOW : HIGH);

    // Buzzer constant ON (LOW) if any supply is low; otherwise OFF (HIGH)
    bool anyLow = capLow || bulbLow || pipetLow;
    digitalWrite(lowSupplyBuzzer, anyLow ? LOW : HIGH);
}

void handlePipetSystem()
{
    static bool lastMotorState = false;
    static bool homingComplete = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    bool twisterAtHome = digitalRead(pipetTwisterHomeSensorPin); // Sensor is HIGH when at home

    // Handle twister homing at startup
    if (!homingComplete)
    {
        if (!twisterAtHome)
        {
            digitalWrite(pipetTwisterPin, HIGH); // Activate twister to move toward home
        }
        else
        {
            digitalWrite(pipetTwisterPin, LOW); // Stop twister when home is reached
            homingComplete = true;
            currentPipetState = PIPET_HOMING_COMPLETE;
        }
        return;
    }

    // Track motor state transitions
    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros(); // Record when motor stopped
    }
    if (!lastMotorState && isMoving)
    {
        motorStartTime = micros(); // Record when motor started
    }
    lastMotorState = isMoving;

    // Only proceed if homing is complete
    if (currentPipetState == PIPET_HOMING_COMPLETE)
    {
        if (machine.canPipetProcessStart())
        {

            if (isMoving)
            {
                // Motor is moving - handle twister activation after 25% of movement
                unsigned long elapsedSteps = stepsTaken;

                // Calculate percentage of movement completed
                float movementPercent = (float)elapsedSteps / TOTAL_STEPS;

                // Activate twister after 25% of movement
                if (movementPercent >= 0.25)
                {
                    digitalWrite(pipetTwisterPin, HIGH);
                }
                machine.setPipetSystemReady(false);
            }
            else
            {
                // Motor is stopped - handle ram and twister timing based on pause duration
                unsigned long stopDuration = micros() - motorStopTime;
                float pausePercent = (float)stopDuration / PAUSE_AFTER;

                // Activate ram after 5% of pause time
                if (pausePercent >= 0.01 && pausePercent < 0.90 && digitalRead(pipetRamPin) == LOW)
                {
                    if (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction())
                    {
                        digitalWrite(pipetRamPin, HIGH);
                    }
                }

                // Deactivate ram after 90% of pause time
                if (pausePercent >= 0.90)
                {
                    digitalWrite(pipetRamPin, LOW);
                    if (twisterAtHome)
                    {
                        machine.setPipetSystemReady(true);
                    }
                }
                // Deactivate twister after 75% of pause time
                if (pausePercent >= 0.75 && digitalRead(pipetTwisterPin) == HIGH && (!slots[slotIdPipetInjection].hasError() && !slots[slotIdPipetInjection].shouldFinishProduction()))
                {
                    digitalWrite(pipetTwisterPin, LOW);
                }
            }
            if (slots[slotIdPipetInjection].shouldFinishProduction() || slots[slotIdPipetInjection].hasError())
            {
                machine.setPipetSystemReady(true);
            }
        }
        else
        {
            digitalWrite(pipetTwisterPin, HIGH);
            machine.setPipetSystemReady(true);
        }
    }
}

void motorPauseTime()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;

    // Track motor state transitions
    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros(); // Record when motor stopped
    }
    lastMotorState = isMoving;

    // Only calculate pause percent when motor is stopped
    if (!isMoving)
    {
        unsigned long stopDuration = micros() - motorStopTime;
        motorPausePercent = (float)stopDuration / PAUSE_AFTER;
    }
    else
    {
        motorPausePercent = 0; // Reset when motor starts moving
    }
}

void handleBulbSystem()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;
    static unsigned long motorStartTime = 0;
    static bool ramExtended = false;
    static bool ramRetracted = true;

    // NEW: preloader state (fires once per stop)
    static bool preloadArmed = false;           // becomes true when we detect motor stopped
    static bool preloadFiredThisStop = false;   // ensure only one fire per stop
    static unsigned long preloadPulseStart = 0; // for fast retract timing

    // Track motor state transitions
    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros(); // Record when motor stopped
        ramRetracted = false;     // Ram needs to retract again

        // NEW: arm preloader for this stop
        preloadArmed = true;
        preloadFiredThisStop = false;
    }
    if (!lastMotorState && isMoving)
    {
        motorStartTime = micros(); // Record when motor started
        machine.setBulbSystemReady(false);
        machine.setBulbPreLoadReady(false);
        // NEW: disarm on movement; will re-arm at next stop
        preloadArmed = false;
    }
    lastMotorState = isMoving;

    // Read sensors
    bool ramHome = digitalRead(bulbRamHomeSensorPin);            // HIGH if home
    bool bulbInPreload = digitalRead(bulbInPreLoadPosSensorPin); // HIGH if present
    bool bulbInCap = digitalRead(bulbInCapSensor);
    bool preLoadCylinderHome = digitalRead(preLoadCylinderHomeSensorPin);

    // ===================== NEW: Preloader one-shot =====================
    if (!machine.canPreLoadBulbProcessStart())
    {
        machine.setBulbPreLoadReady(true);
    }
    if (!isMoving && preloadArmed && !preloadFiredThisStop && machine.canPreLoadBulbProcessStart() && bulbInPreload)
    {
        if (machine.inProduction && !slots[slotIdBulbPreLoad].hasError() && !slots[slotIdBulbPreLoad].shouldFinishProduction())
        {
            digitalWrite(bulbPreLoadCylinder, HIGH);
        }
        preloadPulseStart = micros();
        preloadFiredThisStop = true; // ensure only once per stop
    }
    if (!isMoving && digitalRead(preLoadCylinderHomeSensorPin) == LOW && (slots[slotIdBulbPreLoad].hasError() || slots[slotIdBulbPreLoad].shouldFinishProduction()))
    {
        machine.setBulbPreLoadReady(true);
    }

    // Fast retract: end the pulse as soon as we've met the minimum actuation time
    if (preloadFiredThisStop)
    {
        unsigned long stopDuration = micros() - motorStopTime;
        float pausePercent = (float)stopDuration / PAUSE_AFTER;
        if (pausePercent >= 0.95)
        {
            digitalWrite(bulbPreLoadCylinder, LOW); // retract ASAP
            if (digitalRead(preLoadCylinderHomeSensorPin) == LOW)
            {
                machine.setBulbPreLoadReady(true);
            }
        }
    }

    if (!preloadFiredThisStop && machine.canPreLoadBulbProcessStart())
    {
        // Intentionally left blank to preserve behavior
    }
    // ===================================================================

    if (machine.canBulbProcessStart())
    {
        if (!isMoving)
        {
            // Motor is stopped - handle timing based on pause duration
            unsigned long stopDuration = micros() - motorStopTime;
            float pausePercent = (float)stopDuration / PAUSE_AFTER;

            // Activate ram after 5% of pause time (only if bulb position sensor reads LOW)
            if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && bulbInCap)
            {
                if (!slots[slotIdBulbInjection].hasError() && !slots[slotIdBulbInjection].shouldFinishProduction())
                {
                    digitalWrite(bulbRamPin, HIGH);
                }

                ramExtended = true;
                ramRetracted = false;
            }
            else if (pausePercent >= 0.01 && pausePercent < 0.95 && !digitalRead(bulbRamPin) && !bulbInCap)
            {
                machine.bulbPresent = false;
            }

            if (ramHome && (slots[slotIdBulbInjection].hasError() || slots[slotIdBulbInjection].shouldFinishProduction()))
            {
                machine.setBulbSystemReady(true);
            }

            // Deactivate ram after 95% of pause time
            if (pausePercent >= 0.95 && digitalRead(bulbRamPin))
            {
                digitalWrite(bulbRamPin, LOW);
            }

            // Only set system ready when ram is confirmed home and retracted
            if (ramExtended && ramHome && !digitalRead(bulbRamPin))
            {
                machine.setBulbSystemReady(true);
                ramExtended = false;
                ramRetracted = true;
            }
        }
    }
    else
    {
        machine.setBulbSystemReady(true);
    }
}

void updateSlotPositions()
{
    for (int i = 0; i < 16; i++)
    {
        int relativePos = (currentHomePosition + slots[i].getId()) % 16;
        slots[i].setPosition(relativePos);
    }
    machine.IncrementPositionsMoved();
}
bool shouldRunTracker = true;

void machineTracker()
{
    static bool lastMotorState = false;
    static unsigned long motorStopTime = 0;

    if (lastMotorState && !isMoving)
    {
        motorStopTime = micros(); // Record when motor stopped
    }
    lastMotorState = isMoving;

    unsigned long stopDuration = micros() - motorStopTime;

    if (!isMoving && shouldRunTracker)
    {
        shouldRunTracker = false;
        motorPauseTime();

        if (finsihProdRequested)
        {
            slots[slotIdFailedJunkEject].setFinsihProduction(true);
        }

        if (digitalRead(pipetTipSensor) == HIGH &&
            machine.canPipetConfirmStart() &&
            !slots[slotIdPipetConfirm].shouldFinishProduction() &&
            !slots[slotIdPipetConfirm].hasError())
        {
            slots[slotIdPipetConfirm].setJunk(true);
        }
        else
        {
            slots[slotIdPipetConfirm].setJunk(false);
        }

        if (digitalRead(bulbInCapSensor) == LOW &&
            machine.canBulbConfirmStart() &&
            !slots[slotIdBulbInCapConfirm].shouldFinishProduction() &&
            !slots[slotIdBulbInCapConfirm].hasError())
        {
            slots[slotIdBulbInCapConfirm].setMissingBulb(true);
        }
        else
        {
            slots[slotIdBulbInCapConfirm].setMissingBulb(false);
        }

        if (digitalRead(capInWheel) == LOW &&
            machine.canCapConfirmStart() &&
            !slots[slotIdCapInWheelConfirm].shouldFinishProduction() &&
            !slots[slotIdCapInWheelConfirm].hasError())
        {
            slots[slotIdCapInWheelConfirm].setMissingCap(true);
        }
        else
        {
            slots[slotIdCapInWheelConfirm].setMissingCap(false);
        }

        if (machine.canJunkEjectionStart())
        {
            if (stopDuration < 18000 && !isMoving)
            {
                digitalWrite(junkEjectorPin, HIGH);
            }
            if (slots[slotIdJunkEjection].hasError())
            {
                machine.incrementErroredDroppers();
            }
        }

        if (machine.canDropperEjectionStart() &&
            !slots[slotIdDropeprEjection].hasError() &&
            !slots[slotIdDropeprEjection].shouldFinishProduction())
        {
            machine.incrementDroppersCompleted();
            if (stopDuration < 18000 && !isMoving)
            {
                digitalWrite(dropperEjectPin, HIGH);
            }
        }

        if (machine.canCheckForEmptyStart() &&
            digitalRead(slotEmptySensor) == HIGH)
        {
            slots[slotIdJunkConfirm].setFailedJunkEject(true);
        }
        else if (machine.canCheckForEmptyStart() &&
                 digitalRead(slotEmptySensor) == LOW)
        {
            slots[slotIdJunkConfirm].setFailedJunkEject(false);
            slots[slotIdJunkConfirm].setJunk(false);
            slots[slotIdJunkConfirm].setMissingBulb(false);
            slots[slotIdJunkConfirm].setMissingCap(false);
            slots[slotIdJunkConfirm].setError(false);
        }

        setSlotErrors(slots);

        if (hasConsecutiveErrors())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }

        if (slots[slotIdFailedJunkEject].hasFailedJunkEject())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }
        if (slots[slotIdJunkEjection].shouldFinishProduction())
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
        }
    }

    if (isMoving || machine.isStopped || machine.isPaused)
    {
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
    }
    if (stopDuration > 18000)
    {
        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(dropperEjectPin, LOW);
    }
}

void handleCapInjection()
{
    if (machine.inProduction &&
        !slots[slotIdFailedJunkEject].hasError() &&
        !slots[slotIdFailedJunkEject].shouldFinishProduction())
    {
        digitalWrite(capInjectPin, HIGH);
    }
    else
    {
        digitalWrite(capInjectPin, LOW);
    }

    if (!isMoving && digitalRead(capPositionSensorPin) == HIGH)
    {
        machine.capInjectionReady = true;
    }
    if (slots[slotIdFailedJunkEject].hasError() ||
        slots[slotIdFailedJunkEject].shouldFinishProduction())
    {
        machine.capInjectionReady = true;
    }
    if (isMoving)
    {
        machine.capInjectionReady = false;
    }
}

void homeMachine()
{
    unsigned long stepDelay = 5000;
    unsigned long lastStep = micros();

    while (digitalRead(homeSensorPin) == HIGH)
    {
        digitalWrite(capInjectPin, LOW);

        if (!digitalRead(pauseButtonPin))
        {
            machine.stop();
            machine.updateStatus(myNex, "Homing Stopped");
            break;
        }

        if (micros() - lastStep >= stepDelay)
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(stepPin, LOW);
            lastStep = micros();
        }
    }

    if (digitalRead(homeSensorPin) == LOW)
    {
        machine.inProduction = true;

        digitalWrite(junkEjectorPin, LOW);
        digitalWrite(capInjectPin, HIGH);
        delay(500);
        digitalWrite(capInjectPin, LOW);
        delay(250);
    }

    currentHomePosition = 0;
    machine.homingComplete();
    isMoving = true;
    lastStepTime = micros();
}

bool puasedStateProcessing = false;

void stepMotor()
{
    unsigned long currentTime = micros();

    if (isMoving)
    {
        unsigned long stepDelay;

        if (stepsTaken < ACCEL_STEPS)
        {
            float progress = (float)stepsTaken / ACCEL_STEPS;
            stepDelay =
                MAX_STEP_DELAY * pow((float)MIN_STEP_DELAY / MAX_STEP_DELAY, progress);
        }
        else
        {
            float progress =
                (float)(stepsTaken - ACCEL_STEPS) / DECEL_STEPS;
            stepDelay =
                MIN_STEP_DELAY * pow((float)MAX_STEP_DELAY / MIN_STEP_DELAY, progress);
        }

        stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);

        if (currentTime - lastStepTime >= stepDelay)
        {
            if (!stepHigh)
            {
                digitalWrite(stepPin, HIGH);
                stepHigh = true;
            }
            else
            {
                digitalWrite(stepPin, LOW);
                stepHigh = false;
                stepsTaken++;

                if (stepsTaken >= TOTAL_STEPS)
                {
                    shouldRunTracker = true;
                    isMoving = false;
                    pauseStartTime = currentTime;
                    stepsTaken = 0;
                    machine.resetAllPneumatics();

                    currentHomePosition =
                        (currentHomePosition + 1) % 16;

                    updateSlotPositions();
                }
                else
                {
                    puasedStateProcessing = true;
                }
            }
            lastStepTime = currentTime;
        }
    }
    else
    {
        if (handleLowSupplies())
        {
            MIN_STEP_DELAY = 31 * 16;
            MAX_STEP_DELAY = 616 * 16;
            ACCEL_STEPS = 46 * 16;
            DECEL_STEPS = 15 * 16;
        }
        else
        {
            MIN_STEP_DELAY = 31;
            MAX_STEP_DELAY = 616;
            ACCEL_STEPS = 46;
            DECEL_STEPS = 15;
        }

        if (pauseRequested)
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
            pauseRequested = false;
            return;
        }

        if (machine.isReadyToMove() &&
            (currentTime - pauseStartTime >= PAUSE_AFTER))
        {
            if (digitalRead(bulbRamHomeSensorPin) &&
                !digitalRead(bulbRamPin))
            {
                isMoving = true;
                lastStepTime = currentTime;
            }
        }
    }
}

void emptySlots()
{
    machine.updateStatus(myNex, "Emptying Slots");
    const unsigned long stepDelay = 4000;

    digitalWrite(pipetTwisterPin, LOW);

    int i = 0;
    while (i <= 3200)
    {
        if (!digitalRead(pauseButtonPin))
        {
            machine.stop();
            machine.updateStatus(myNex, "Emptying Stopped");
            break;
        }
        if (i % 200 == 0)
        {
            digitalWrite(junkEjectorPin, HIGH);
            delay(200);
            digitalWrite(junkEjectorPin, LOW);
            delay(200);
        }

        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);

        ++i;
    }

    machine.updateStatus(myNex, "Emptying Completed");

    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(pipetTwisterPin, HIGH);

    machine.stop();
}

void handleEmptySlots()
{
    if (!machine.inProduction &&
        !digitalRead(emptySlotsButtonPin))
    {
        emptySlots();
    }
}

void handleButtons()
{
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;
    static bool lastStartState = LOW;
    static bool lastPauseState = LOW;
    static bool lastFinishState = LOW;

    bool startState = digitalRead(startButtonPin);
    bool pauseState = digitalRead(pauseButtonPin);
    bool finishState = digitalRead(finishProductionButtonPin);

    if (millis() - lastDebounceTime < debounceDelay)
        return;

    handleEmptySlots();

    if (startState == LOW && lastStartState == HIGH)
    {
        lastDebounceTime = millis();
        machine.start();
        pauseRequested = false;
        finsihProdRequested = false;
        machine.updateStatus(myNex, "In Production");
    }

    if (pauseState == LOW && lastPauseState == HIGH &&
        !machine.isStopped && !machine.isPaused)
    {
        machine.updateStatus(myNex, "Paused");
        lastDebounceTime = millis();
        pauseRequested = true;
    }

    if (finishState == LOW && lastFinishState == HIGH &&
        !machine.isStopped)
    {
        if (machine.inProduction)
        {
            machine.updateStatus(myNex, "End Production");
        }
        lastDebounceTime = millis();
        finsihProdRequested = true;
    }

    lastStartState = startState;
    lastPauseState = pauseState;
    lastFinishState = finishState;
}

// --- Potentiometer smoothing ---
const uint8_t POT_EMA_ALPHA_NUM = 1;
const uint8_t POT_EMA_ALPHA_DEN = 8;
const unsigned long PAUSE_DEADBAND_US = 2000UL;

void updatePauseAfterFromPot()
{
    static int ema = -1;
    static unsigned long lastPause = PAUSE_AFTER;

    int raw = analogRead(speedPotPin);

    if (ema < 0)
    {
        ema = raw;
    }
    else
    {
        ema = (int)((long)ema * (POT_EMA_ALPHA_DEN - POT_EMA_ALPHA_NUM) +
                    (long)raw * POT_EMA_ALPHA_NUM) /
              POT_EMA_ALPHA_DEN;
    }

    unsigned long span = (PAUSE_MAX_US - PAUSE_MIN_US);
    unsigned long mappedPause =
        PAUSE_MAX_US - ((unsigned long)ema * span) / 1023UL;

    if (mappedPause < PAUSE_MIN_US) mappedPause = PAUSE_MIN_US;
    if (mappedPause > PAUSE_MAX_US) mappedPause = PAUSE_MAX_US;

    if ((mappedPause > lastPause + PAUSE_DEADBAND_US) ||
        (mappedPause + PAUSE_DEADBAND_US < lastPause))
    {
        PAUSE_AFTER = mappedPause;
        lastPause = mappedPause;
    }
}

void systemNotReadyTimeout()
{
    static unsigned long notReadySince = 0;
    static bool tracking = false;

    const unsigned long TIMEOUT_MS = 2000UL;
    unsigned long now = millis();

    if (machine.isReadyToMove())
    {
        machine.timeoutMachine = false;
    }

    bool condition =
        (!isMoving) &&
        (!machine.isReadyToMove()) &&
        (!machine.isPaused) &&
        (!machine.isStopped);

    if (condition)
    {
        if (!tracking)
        {
            tracking = true;
            notReadySince = now;
        }
        else if (now - notReadySince >= TIMEOUT_MS)
        {
            machine.pause(junkEjectorPin, dropperEjectPin);
            tracking = false;
            machine.timeoutMachine = true;
        }
    }
    else
    {
        tracking = false;
    }
}

void handleLowAirPressure()
{
    if (digitalRead(lowAirSensorPin) == LOW)
    {
        machine.hasLowAirPressure = true;
        machine.pause(junkEjectorPin, dropperEjectPin);
        machine.updateStatus(myNex, "Low Air - Pause");
    }
    else
    {
        machine.hasLowAirPressure = false;
    }
}
void setup()
{
    // Start Nextion
    myNex.begin(115200);

    // ============================
    // PIN MODES — Teensy 4.1
    // ============================

    pinMode(speedPotPin, INPUT);

    // Motor driver pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Buttons (INPUT_PULLUP)
    pinMode(startButtonPin, INPUT_PULLUP);
    pinMode(pauseButtonPin, INPUT_PULLUP);
    pinMode(finishProductionButtonPin, INPUT_PULLUP);
    pinMode(emptySlotsButtonPin, INPUT_PULLUP);
    pinMode(speedButtonPin, INPUT_PULLUP);

    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);
    digitalWrite(dropperEjectPin, LOW);

    // Pneumatic outputs
    pinMode(bulbRamPin, OUTPUT);
    pinMode(dropperEjectPin, OUTPUT);
    pinMode(pipetTwisterPin, OUTPUT);
    pinMode(pipetRamPin, OUTPUT);
    pinMode(capInjectPin, OUTPUT);
    pinMode(junkEjectorPin, OUTPUT);
    pinMode(bulbPreLoadCylinder, OUTPUT);

    // Sensors
    pinMode(pipetTwisterHomeSensorPin, INPUT);
    pinMode(preLoadCylinderHomeSensorPin, INPUT);

    pinMode(homeSensorPin, INPUT);
    pinMode(bulbRamHomeSensorPin, INPUT);
    pinMode(pipetTipSensor, INPUT);
    pinMode(bulbInCapSensor, INPUT);
    pinMode(capInWheel, INPUT);
    pinMode(slotEmptySensor, INPUT);
    pinMode(pipetSupplySensorPin, INPUT);
    pinMode(bulbSupplySensorPin, INPUT);
    pinMode(capSupplySensorPin, INPUT);
    pinMode(capPositionSensorPin, INPUT);

    pinMode(lowAirSensorPin, INPUT);

    // Lights + buzzer
    pinMode(capLowSupplyLight, OUTPUT);
    pinMode(bulbLowSupplyLight, OUTPUT);
    pinMode(pipetLowSupplyLight, OUTPUT);
    pinMode(lowSupplyBuzzer, OUTPUT);

    // Default states
    digitalWrite(pipetTwisterPin, LOW);
    digitalWrite(bulbRamPin, LOW);
    digitalWrite(capInjectPin, LOW);
    digitalWrite(pipetRamPin, LOW);
    digitalWrite(junkEjectorPin, LOW);
    digitalWrite(bulbPreLoadCylinder, LOW);

    currentPipetState = PIPET_HOMING;

    updateSlotPositions();
}

int i = 0;

void loop()
{
    handleLowAirPressure();
    updatePauseAfterFromPot();
    handleButtons();
    handleSupplyAlert();
    setSlotIdByPosition(slots);

    startTime = millis();
    motorPauseTime();

    if ((!isMoving && motorPausePercent > .90) || machine.isPaused)
    {
        machine.updateMachineDisplayInfo(myNex, startTime, slots);
    }

    machineTracker();
    handleCapInjection();
    handleBulbSystem();
    handlePipetSystem();
    systemNotReadyTimeout();

    if (machine.isStopped)
        return;

    if (machine.needsHoming)
    {
        if (machine.needsHoming)
        {
            machine.updateStatus(myNex, "Motor Homing");
            homeMachine();
        }
        if (!machine.needsHoming && !machine.isPaused && !machine.isStopped)
        {
            machine.updateStatus(myNex, "In Production");
        }
        return;
    }

    if (machine.inProduction)
    {
        stepMotor();
    }
}
