/*
  ============================================================================
  System Overview (LandingGear)
  It controls a Central Motorized Dual-Arm Maneuvering/Stand Assist System,
  enabling secure parking and very low-speed movement of the motorcycle.
  It includes safety, synchronization, and data logging functions.

  Description:
  Control system for dual motorized arms.
  - Controlled via two BTS7960 drivers (left/right)
  - Overload detection using ACS712 30A current sensors
  - Automatic shutdown in case of fault or desynchronization
  - Event logging on microSD card (64 GB FAT32)
  - Automatic sensor calibration via long press on Button 1
  - Saving parameters and states in EEPROM

  Interfaces:
  - Arduino Nano R4 (14-bit ADC, 5V logic)
  - Nano Connector Carrier via SPI: CS=D4, MOSI=D11, MISO=D12, SCK=D13
  - Limit switches: D9 (left), D8 (right)
  - Buttons:
    ArmButton (A7), CommandButton (D10), CalibButton (D2) and ArmLED (D5)
  - Arm LED: Located inside the ArmButton
  - Motor drivers:
      Left: EN_G=A6, RPWM_G=A3, LPWM_G=A2
      Right: EN_D=D3, RPWM_D=D6, LPWM_D=D7

  - Current sensors: A0 (left), A1 (right)

  - Active GPS Protection: The NEO-M8N (D0/D1) continuously monitors speed to ensure system safety.
  - If arms are deployed and speed ≥30 km/h, they retract automatically (forced REST).
  - At ≥25 km/h, any deployment is blocked: motors off, LED off, drivers disabled, REST state enforced.
  - The button remains active, but ON/OFF logic ensures the next action will be a safe deployment.

    * AUTHOR: lsb71620
    * DATE: 2025
    * LICENSE:
    * This code is distributed under the MIT License.
    * For details, see the LICENSE file at the root of the repository.
    * COPYRIGHT (©) 2025 lsb71620

  Notes:
  - Do not connect motors before empty calibration.
  - Long press CalibButton (5s) to calibrate sensors.
  - Check current thresholds and limit switch logic.
  ============================================================================
*/

// -------------------------------------------------------------------------------------
// Libraries
#include <EEPROM.h> // to store offsets/thresholds in non-volatile memory (EEPROM)
#include <SD.h>     // for microSD card management (D4)
#include <SPI.h>    // SPI bus used by SD reader

// If GPS is integrated into the system
#include <TinyGPS++.h>
// -------------------------------------------------------------------------------------

// ==========================
// Pin declarations
// ==========================

// -------------------------------------------------------------------------------------
TinyGPSPlus gps;
// Instance to parse GPS data
// -------------------------------------------------------------------------------------

// Chip Select pin for SD card (D4 as per SPI comments)
constexpr int chipSelectSD = D4;
// Buttons and LED
constexpr int Bouton_Armement = A7;  // Arm button (left)
constexpr int Bouton_Commande = D10; // Command button (right)
constexpr int Led_Armement = D5;     // Arm LED (left)
constexpr int Bouton_Calib = D2;     // Calibration button (right)

// Limit switches - Screws 1 + 2 → NC contact, Screws 3 + 4 → NO contact
constexpr int finCourseReplieG = D9; // Left arm limit switch NC (screws 1 + 2)
constexpr int finCourseReplieD = D8; // Right arm limit switch NC (screws 1 + 2)

// Motor drivers
// Left
constexpr int EN_G = A6;   // Enable left driver
constexpr int RPWM_G = A3; // PWM direction 1 left motor
constexpr int LPWM_G = A2; // PWM direction 2 left motor

// Right
constexpr int EN_D = D3;   // Enable right driver
constexpr int RPWM_D = D6; // PWM direction 1 right motor
constexpr int LPWM_D = D7; // PWM direction 2 right motor

// Current sensors
constexpr int ACS_G_Pin = A0; // Left current sensor
constexpr int ACS_D_Pin = A1; // Right current sensor

// ==========================
// System constants and parameters
// ==========================
// ADC reference of the Nano R4
const float Vref = 5.0f; // 5 V (ADC reference voltage on Nano R4)

const int ADC_BITS = 14;
const int ADC_STEPS = (1 << ADC_BITS); // 16384
const int ADC_MAX_VAL = ADC_STEPS - 1; // 16383

// Current sensor sensitivity (typical value for ACS712 30A): 66 mV/A
const float ACS_SENS_V_PER_A = 0.066f; // V per ampere (e.g., 0.066 V/A)

// Timings / safety
const unsigned long dureeMaxAction = 1500UL;   // max time for a movement (ms) -> fault if exceeded
const unsigned long delaiCalibBouton = 5000UL; // long press on Button1 for calibration (ms)

// PWM / synchronization
const int PWM_BASE = 200; // initial nominal power (0..255)
const int PWM_MAX = 255;
const float SYNC_KP = 20.0f; // coefficient for left/right current synchronization

// ==========================
// EEPROM structure (persistent)
// ==========================
// We store: magic (validation), zero offsets in volts, thresholds in amps,
// and precomputed ADC values (to avoid recalculating each loop)
struct Config
{
  uint32_t magic;
  float zeroOffsetV_G;    // sensor output voltage at 0 A (left)
  float zeroOffsetV_D;    // sensor output voltage at 0 A (right)
  float seuilReposA;      // threshold to consider 'rest' (A)
  float seuilMouvementA;  // threshold to consider 'in motion' (A)
  float seuilSurchargeA;  // overload threshold (A) -> fault
  uint16_t seuilReposADC; // thresholds converted to ADC (14-bit)
  uint16_t seuilMouvementADC;
  uint16_t seuilSurchargeADC;
  uint8_t lastState;
  bool nextActionIsDeployment; // CORRECTION: Renamed for clarity and consistency
  uint8_t checksum;            // for data validation
};

const uint32_t EEPROM_MAGIC = 0xA5A5A5A5; // value to detect valid config
Config cfg;                               // instance in RAM

// ==========================
// Variables for SD card
// ==========================

bool sdOk = false; // SD card initialization status
File logFile;      // Logging file

// ==========================
// State machine states
// ==========================
enum State
{
  REPOS,
  DEPLOIEMENT,
  RETRACTION,
  DEFAUT
};
State currentState = REPOS;

bool nextActionIsDeployment = false;

// Variables for long press handling, calibration, action timing
bool bouton1EtaitActif = false;
unsigned long bouton1PressMillis = 0;
bool calibratedThisPress = false;
unsigned long actionStartMillis = 0;

// ==========================
// Variables for tracking max current in log file
// ==========================
float courantMaxG = 0.0;
float courantMaxD = 0.0;

// ==========================
// Utility functions (conversion, filtered reading)
// ==========================

// Converts an ADC value (0..ADC_MAX) to voltage (V)
// Formula: V = (adc / ADC_MAX) * Vref
float adcToVoltage(int adc)
{
  return ((float)adc / (float)ADC_STEPS) * Vref;
}

// Converts a voltage to ADC value - useful for threshold calculations
int voltageToAdc(float v) 
{
  int a = (int)round((v / Vref) * (float)ADC_STEPS);
  if (a < 0) return 0;
  if (a > ADC_MAX_VAL) return ADC_MAX_VAL;
  return a;
}

// Converts amperes -> ADC value using zero offset (in volts)
// Vout = zeroOffsetV + I * sensitivity
uint16_t ampsToAdc(float amps, float zeroOffsetV)
{
  float vout = zeroOffsetV + amps * ACS_SENS_V_PER_A;
  return (uint16_t)voltageToAdc(vout);
}

// Average reading of an ADC channel to reduce noise (fast, non-blocking)
int readAdcAvg(int pin, int samples = 16)
{
  long s = 0;
  for (int i = 0; i < samples; ++i)
  {
    s += analogRead(pin);
  }
  return (int)(s / samples);
}

// ==========================
// Checksum calculation function (for EEPROM)
// ==========================
uint8_t calculateChecksum()
{
  uint8_t cs = 0;
  // Calculate checksum over the entire structure EXCEPT the last byte (the checksum itself)
  for (size_t i = 0; i < sizeof(Config) - 1; ++i)
  {
    cs ^= *(((uint8_t *)&cfg) + i); // XOR each byte
  }
  return cs;
}

// ==========================
// EEPROM helpers: load / save (CORRECTED)
// ==========================
void loadConfig()
{
  EEPROM.get(0, cfg);
  uint8_t calculated_cs = calculateChecksum();

  if (cfg.magic != EEPROM_MAGIC || cfg.checksum != calculated_cs)
  {
    Serial.println("EEPROM not initialized or corrupted. Loading default values.");
    logEvent("!!! ALERT: EEPROM corrupted/uninitialized -> Loading default values.");

    // Default values if EEPROM not initialized
    cfg.magic = EEPROM_MAGIC;
    cfg.zeroOffsetV_G = 2.5f; // default value (typical for 5V sensor) - to be recalibrated
    cfg.zeroOffsetV_D = 2.5f;
    cfg.seuilReposA = 1.0f;
    cfg.seuilMouvementA = 3.0f;
    cfg.seuilSurchargeA = 15.0f;

    // Default states
    currentState = REPOS;
    nextActionIsDeployment = true;

    // Precompute ADC thresholds
    cfg.seuilReposADC = ampsToAdc(cfg.seuilReposA, cfg.zeroOffsetV_G);
    cfg.seuilMouvementADC = ampsToAdc(cfg.seuilMouvementA, cfg.zeroOffsetV_G);
    cfg.seuilSurchargeADC = ampsToAdc(cfg.seuilSurchargeA, cfg.zeroOffsetV_G);

    // Save new checksum
    cfg.checksum = calculateChecksum();
    EEPROM.put(0, cfg);
  }
  else
  {
    // Restore previous states
    currentState = (State)cfg.lastState;
    nextActionIsDeployment = cfg.nextActionIsDeployment;
  }
}

void saveConfig()
{
  cfg.checksum = calculateChecksum();
  EEPROM.put(0, cfg);
}

// ==========================
// Logging function to SD card (NON-BLOCKING)
// ==========================
void logEvent(const String &event)
{
  if (!sdOk)
  {
    return; // Guaranteed non-blocking
  }

  logFile = SD.open("LOG.TXT", FILE_WRITE);

  if (logFile)
  {
    unsigned long t = millis();
    logFile.print("[");
    logFile.print(t);
    logFile.print(" ms] ");
    logFile.println(event);
    logFile.close();
  }
  else
  {
    Serial.print("!!! SD ERROR: Unable to open LOG.TXT. Missed event: ");
    Serial.println(event);
  }
}

// ==========================
// Motor driver functions
// ==========================

// Enable/disable drivers (EN pins) - used for hardware cutoff on fault
void enableDrivers(bool en)
{
  digitalWrite(EN_G, en ? HIGH : LOW);
  digitalWrite(EN_D, en ? HIGH : LOW);
}

// Immediately stop all PWM outputs (motors)
void stopAllMotors()
{
  analogWrite(RPWM_G, 0);
  analogWrite(LPWM_G, 0);
  analogWrite(RPWM_D, 0);
  analogWrite(LPWM_D, 0);
}

/* ==========================
 * Missing motor control functions
 * ==========================
 */

/**
 * Controls the LEFT motor
 * deployer = true  -> rotates in "deployment" direction
 * deployer = false -> rotates in "retraction" direction
 */
void setMotorGauche(bool deployer, int pwm)
{
  if (deployer)
  {
    analogWrite(RPWM_G, pwm);
    analogWrite(LPWM_G, 0);
  }
  else
  {
    analogWrite(RPWM_G, 0);
    analogWrite(LPWM_G, pwm);
  }
}

/**
 * Controls the RIGHT motor
 * deployer = true  -> rotates in "deployment" direction
 * deployer = false -> rotates in "retraction" direction
 */
void setMotorDroit(bool deployer, int pwm)
{
  if (deployer)
  {
    analogWrite(RPWM_D, pwm);
    analogWrite(LPWM_D, 0);
  }
  else
  {
    analogWrite(RPWM_D, 0);
    analogWrite(LPWM_D, pwm);
  }
}

// ==========================
// ACS Calibration (long press on Button1)
// ==========================
// Measures zero offset (Vout at 0 A) averaged over N samples,
// then saves to EEPROM.
// Useful if sensor powered by 3.3V/5V.
void calibrerACS() {
  const int ledCalib = 13;
  pinMode(ledCalib, OUTPUT);

  const int samples = 200;
  long sG = 0, sD = 0;

  for (int i = 0; i < samples; ++i) {
    sG += analogRead(ACS_G_Pin);
    sD += analogRead(ACS_D_Pin);
    digitalWrite(ledCalib, (i % 2 == 0) ? HIGH : LOW);
    delay(2);
  }

  digitalWrite(ledCalib, LOW);

  for (int i = 0; i < 6; ++i) {
    digitalWrite(ledCalib, HIGH);
    delay(100);
    digitalWrite(ledCalib, LOW);
    delay(100);
  }

  float avgG = (float)sG / samples;
  float avgD = (float)sD / samples;
  float vG = adcToVoltage((int)round(avgG));
  float vD = adcToVoltage((int)round(avgD));

  cfg.zeroOffsetV_G = vG;
  cfg.zeroOffsetV_D = vD;

  cfg.seuilReposADC     = ampsToAdc(cfg.seuilReposA, cfg.zeroOffsetV_G);
  cfg.seuilMouvementADC = ampsToAdc(cfg.seuilMouvementA, cfg.zeroOffsetV_G);
  cfg.seuilSurchargeADC = ampsToAdc(cfg.seuilSurchargeA, cfg.zeroOffsetV_G);

  saveConfig();

  Serial.println("=== ACS Calibration complete ===");
  Serial.print("ZeroOffsetG (V) = "); Serial.println(cfg.zeroOffsetV_G, 6);
  Serial.print("ZeroOffsetD (V) = "); Serial.println(cfg.zeroOffsetV_D, 6);
  Serial.print("Recalculated ADC thresholds: ");
  Serial.print(cfg.seuilReposADC); Serial.print(", ");
  Serial.print(cfg.seuilMouvementADC); Serial.print(", ");
  Serial.println(cfg.seuilSurchargeADC);

  if (abs(cfg.zeroOffsetV_G - 2.5f) > 1.0f) {
    Serial.println("Suspicious calibration: check left sensor.");
  }
  if (abs(cfg.zeroOffsetV_D - 2.5f) > 1.0f) {
    Serial.println("Suspicious calibration: check right sensor.");
  }
}
// ==========================
// Main function for arm synchronization and control
// ==========================
// Returns true if the action (deployment/retraction) is complete
// - deployer = true  => deploying (motors push downward)
// - deployer = false => retracting (motors pull upward)

// ====================================================================
// Function: synchroniserBras()
// Controls both motors in sync, monitors current,
// limit switches (TZ-8122), and desynchronization.
// ====================================================================
bool synchroniserBras(bool deployer) {
  static unsigned long lastMotorUpdate = 0;
  if (millis() - lastMotorUpdate < 10) return false;
  lastMotorUpdate = millis();

  int adcG = readAdcAvg(ACS_G_Pin, 8);
  int adcD = readAdcAvg(ACS_D_Pin, 8);

  float courantG_A = (adcToVoltage(adcG) - cfg.zeroOffsetV_G) / ACS_SENS_V_PER_A;
  float courantD_A = (adcToVoltage(adcD) - cfg.zeroOffsetV_D) / ACS_SENS_V_PER_A;

  if (courantG_A > courantMaxG) courantMaxG = courantG_A;
  if (courantD_A > courantMaxD) courantMaxD = courantD_A;

  bool G_surcharge = (abs(courantG_A) > cfg.seuilSurchargeA);
  bool D_surcharge = (abs(courantD_A) > cfg.seuilSurchargeA);

  if (G_surcharge || D_surcharge) {
    stopAllMotors();
    enableDrivers(false);
    Serial.println("FAULT: overload detected!");
    logEvent("!!! FAULT: overload detected -> safety shutdown");

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Currents G=%.2fA D=%.2fA", courantG_A, courantD_A);
    logEvent(buffer);

    currentState = DEFAUT;
    cfg.lastState = (uint8_t)currentState;
    cfg.nextActionIsDeployment = nextActionIsDeployment;
    saveConfig();

    return true;
  }

  const float CURRENT_LIMIT_THRESHOLD = 6.0f;

  bool finReplieG = (!deployer) && (digitalRead(finCourseReplieG) == LOW);
  bool finReplieD = (!deployer) && (digitalRead(finCourseReplieD) == LOW);
  bool finDeployeG = (deployer) && (abs(courantG_A) > CURRENT_LIMIT_THRESHOLD);
  bool finDeployeD = (deployer) && (abs(courantD_A) > CURRENT_LIMIT_THRESHOLD);

  if ((finReplieG && finReplieD) || (finDeployeG && finDeployeD)) {
    stopAllMotors();
    Serial.print("Arms fully ");
    Serial.println(deployer ? "deployed (current)." : "retracted (limit switch).");
    logEvent(deployer ? "Cycle complete - arms G/D deployed (current)" : "Cycle complete - arms G/D retracted (limit switch)");
    return true;
  }

  bool desynchronisation = (finReplieG && !finReplieD) ||
                           (!finReplieG && finReplieD) ||
                           (finDeployeG && !finDeployeD) ||
                           (!finDeployeG && finDeployeD);

  if (desynchronisation) {
    stopAllMotors();
    enableDrivers(false);
    Serial.println("FAULT: desynchronization detected!");
    logEvent("FAULT: desynchronization detected (unsynchronized blockage)");

    currentState = DEFAUT;
    cfg.lastState = (uint8_t)currentState;
    cfg.nextActionIsDeployment = nextActionIsDeployment;
    saveConfig();

    return true;
  }

  float diffI = courantG_A - courantD_A;
  int correctionPWM = (int)round(diffI * SYNC_KP);
  int pwmG = constrain(PWM_BASE - correctionPWM, 0, PWM_MAX);
  int pwmD = constrain(PWM_BASE + correctionPWM, 0, PWM_MAX);

  if (deployer) {
    setMotorGauche(true, pwmG);
    setMotorDroit(true, pwmD);
  } else {
    setMotorGauche(false, pwmG);
    setMotorDroit(false, pwmD);
  }

  return false;
}


// ==========================
// Long press handling for calibration
// ==========================
void gestionBoutonCalibration()
{
  bool boutonCalibActif = (digitalRead(Bouton_Calib) == LOW); // active LOW

  if (!boutonCalibActif)
  {
    bouton1EtaitActif = false;
    bouton1PressMillis = 0;
    calibratedThisPress = false;
    return;
  }

  if (!bouton1EtaitActif)
  {
    bouton1EtaitActif = true;
    bouton1PressMillis = millis();
  }

  if (!calibratedThisPress && (millis() - bouton1PressMillis > delaiCalibBouton))
  {
    calibrerACS();
    calibratedThisPress = true;
  }
}

// The Arm LED (D5) must light up to confirm the system is armed and the command button (D10) can be used.
// If the LED is off, the command button action is ignored.
// ==========================
// State machine and button handling (CORRECTED)
// ==========================
void gestionBras() {
  bool bouton1Actif = (digitalRead(Bouton_Armement) == LOW);
  bool bouton2Etat = (digitalRead(Bouton_Commande) == LOW);

  digitalWrite(Led_Armement, bouton1Actif ? HIGH : LOW);

  static bool lastBouton2 = false;

  if (!bouton1Actif || digitalRead(Led_Armement) == LOW) {
    if (currentState != REPOS) {
      stopAllMotors();
      enableDrivers(false);
      currentState = REPOS;
      cfg.lastState = (uint8_t)currentState;
      cfg.nextActionIsDeployment = nextActionIsDeployment;
      saveConfig();
    }
    lastBouton2 = bouton2Etat;
    return;
  }

  enableDrivers(true);

  if (bouton2Etat && !lastBouton2) {
    currentState = nextActionIsDeployment ? DEPLOIEMENT : RETRACTION;
    cfg.lastState = (uint8_t)currentState;
    cfg.nextActionIsDeployment = nextActionIsDeployment;
    saveConfig();

    actionStartMillis = millis();

    if (nextActionIsDeployment) logEvent(">>> START DEPLOYMENT");
    else logEvent(">>> START RETRACTION");
  }

  courantMaxG = 0.0;
  courantMaxD = 0.0;
  lastBouton2 = bouton2Etat;

  switch (currentState) {
    case REPOS:
      break;

    case DEPLOIEMENT:
    case RETRACTION:
      if (millis() - actionStartMillis > dureeMaxAction) {
        Serial.println("!!! FAULT: action timeout -> driver cutoff");
        logEvent("!!! FAULT: timeout - driver cutoff");

        currentState = DEFAUT;
        cfg.lastState = (uint8_t)currentState;
        cfg.nextActionIsDeployment = nextActionIsDeployment;
        saveConfig();

        enableDrivers(false);
        stopAllMotors();
        break;
      }

      if (synchroniserBras(currentState == DEPLOIEMENT)) {
        stopAllMotors();
        currentState = REPOS;
        nextActionIsDeployment = !nextActionIsDeployment;

        cfg.lastState = (uint8_t)currentState;
        cfg.nextActionIsDeployment = nextActionIsDeployment;
        saveConfig();

        if (sdOk) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer),
                   "Cycle complete - Max current G=%.2fA D=%.2fA", courantMaxG, courantMaxD);
          logEvent(buffer);
        }

        logEvent(">>> ACTION COMPLETED OK");
      }
      break;

    case DEFAUT:
      digitalWrite(Led_Armement, LOW);
      break;
  }
}

// ==========================
// Main setup and loop
// Code executed once at startup
// ==========================
void setup() {
  // -------------------------------------------------------------------------------------
  // GPS initialization (hardware serial port Serial1 on D0/D1)
  // -------------------------------------------------------------------------------------
  Serial1.begin(9600);

  // Serial port for debugging
  Serial.begin(115200);

  // Configure 14-bit ADC (Nano R4)
  analogReadResolution(14); // ADC resolution: 0..16383

  // Pin configuration
  pinMode(Bouton_Armement, INPUT_PULLUP);
  pinMode(Bouton_Commande, INPUT_PULLUP);
  pinMode(Bouton_Calib, INPUT_PULLUP);
  pinMode(finCourseReplieG, INPUT_PULLUP);
  pinMode(finCourseReplieD, INPUT_PULLUP);
  pinMode(Led_Armement, OUTPUT);
  digitalWrite(Led_Armement, LOW); // LED off at startup

  // Left motor
  pinMode(EN_G, OUTPUT);
  pinMode(RPWM_G, OUTPUT);
  pinMode(LPWM_G, OUTPUT);

  // Right motor
  pinMode(EN_D, OUTPUT);
  pinMode(RPWM_D, OUTPUT);
  pinMode(LPWM_D, OUTPUT);

  // Startup safety
  stopAllMotors();
  enableDrivers(false);

  // Load EEPROM configuration
  loadConfig();

  // Check limit switches at startup
  bool finG = (digitalRead(finCourseReplieG) == LOW);
  bool finD = (digitalRead(finCourseReplieD) == LOW);

  if (finG && finD) {
    currentState = REPOS;
    cfg.nextActionIsDeployment = true;
    cfg.lastState = (uint8_t)currentState;
    saveConfig();
    Serial.println("Arms detected as retracted at startup → state corrected to REST");
  }

  // -------------------------------------------------------------------------------------
  // microSD card initialization
  // -------------------------------------------------------------------------------------
  Serial.print("Initializing SD card... ");
  if (!SD.begin(chipSelectSD)) {
    Serial.println("FAILURE: SD card not found or not initialized.");
    sdOk = false;
  } else {
    Serial.println("OK SD card ready!");
    sdOk = true;

    logFile = SD.open("LOG.TXT", FILE_WRITE);
    if (logFile) {
      logFile.println("==== SYSTEM STARTUP ====");
      logFile.close();
    }
  }
}

unsigned long lastSerialPrint = 0;

void loop() {
  // =========================================================================
  // Code runs continuously while the board is powered
  // =========================================================================

  // Read GPS data
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  float currentSpeedKMPH = gps.speed.kmph();

  if (gps.speed.isValid()) {
    // Automatic retraction if arms deployed and speed ≥ 30 km/h
    if (currentSpeedKMPH >= 30.0 && currentState == REPOS && !cfg.nextActionIsDeployment) {
      Serial.println("Safety: speed > 30 km/h with arms deployed → forced retraction");
      logEvent(">>> Safety: automatic retraction at 30 km/h");

      currentState = RETRACTION;
      cfg.nextActionIsDeployment = true;
      actionStartMillis = millis();
      enableDrivers(true);
    }

    // Block deployment if speed ≥ 25 km/h or ≥ 30 km/h
    if ((cfg.nextActionIsDeployment && currentSpeedKMPH >= 25.0) || currentSpeedKMPH >= 30.0) {
      if (currentState != REPOS) {
        logEvent("!!! SAFETY: SPEED BLOCK active (Action stopped at " + String(currentSpeedKMPH) + " km/h)");
      }

      digitalWrite(Led_Armement, LOW);
      stopAllMotors();
      enableDrivers(false);
      currentState = REPOS;

      if (!cfg.nextActionIsDeployment) {
        cfg.nextActionIsDeployment = true;
      }
    }
  }

  // Calibration and arm control
  gestionBoutonCalibration(); // Long press on calibration button
  gestionBras();              // State machine (buttons, motors, limit switches)

  // =========================================================================
  // PERIODIC DISPLAY (every 500 ms)
  // =========================================================================
  if (millis() - lastSerialPrint > 500) {
    int adcG = readAdcAvg(ACS_G_Pin, 12);
    int adcD = readAdcAvg(ACS_D_Pin, 12);
    float vG = adcToVoltage(adcG);
    float vD = adcToVoltage(adcD);
    float iG = (vG - cfg.zeroOffsetV_G) / ACS_SENS_V_PER_A;
    float iD = (vD - cfg.zeroOffsetV_D) / ACS_SENS_V_PER_A;

    const float ALERT_FACTOR = 1.5;
    bool floatingAlert = (currentState == REPOS) &&
                         ((abs(iG) > cfg.seuilReposA * ALERT_FACTOR) || (abs(iD) > cfg.seuilReposA * ALERT_FACTOR));
    bool physicalSensorAlert = (adcG < 100 || adcG > 16000) || (adcD < 100 || adcD > 16000);

    static bool printedHardwareAlert = false;
        if (!printedHardwareAlert) {
      Serial.println();
      Serial.println("HARDWARE REMINDER: Key components (ACS712, BTS7960) not connected.");
      Serial.println("System is in simulation/test mode until connected.");
      Serial.println("Long press Button 1 (5s) to calibrate sensors.");
      Serial.println();
      printedHardwareAlert = true;
    }

    if (floatingAlert || physicalSensorAlert) {
      Serial.println("  WARNING: Current sensors (A0/A1) may be disconnected or miscalibrated. Read values:");
      Serial.print("  ADC G:"); Serial.print(adcG); Serial.print("  ");
      Serial.print("ADC D:"); Serial.print(adcD); Serial.print("  ");
      Serial.print("I G:"); Serial.print(iG, 3); Serial.print("A  ");
      Serial.print("I D:"); Serial.print(iD, 3); Serial.print("A");
      Serial.println();
      Serial.println("  Action: Check wiring or run calibration (long press Button 1).");
      Serial.println("========================================================");
    } else {
      Serial.print("ADC G:"); Serial.print(adcG); Serial.print("  ");
      Serial.print("ADC D:"); Serial.print(adcD); Serial.print("  ");
      Serial.print("I G:"); Serial.print(iG, 3); Serial.print("A  ");
      Serial.print("I D:"); Serial.print(iD, 3); Serial.print("A  ");
      Serial.print("State:");
      switch (currentState) {
        case REPOS:       Serial.print("REST"); break;
        case DEPLOIEMENT: Serial.print("DEPLOYMENT"); break;
        case RETRACTION:  Serial.print("RETRACTION"); break;
        case DEFAUT:      Serial.print("FAULT"); break;
      }
      Serial.print("  OverloadThresholdADC:");
      Serial.print(cfg.seuilSurchargeADC);
      Serial.println();
    }

    lastSerialPrint = millis();
  }
}
