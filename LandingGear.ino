/*
  ============================================================================
  Présentation du dispositif (LandingGear)
  Il assure le pilotage d’un Système d'Assistance à la Manœuvre/Béquillage Central Motorisé à Double Bras, 
  permettant le stationnement sécurisé et le déplacement à très faible vitesse de la moto. 
  Il intègre les fonctions de sécurité, de synchronisation et de journalisation des données.

  Description :
  Système de commande pour double bras motorisé.
  - Pilotage via deux drivers BTS7960 (gauche/droit)
  - Détection de surcharge par capteurs de courant ACS712 30A
  - Arrêt automatique en cas de défaut ou désynchronisation
  - Journalisation des événements sur carte microSD (64 Go FAT32)
  - Calibration automatique des capteurs par appui long sur Bouton 1
  - Sauvegarde des paramètres et états dans l’EEPROM

  Interfaces :
  - Arduino Nano R4 (ADC 14 bits, logique 5V)
  - Nano Connector Carrier via SPI : CS=D4, MOSI=D11, MISO=D12, SCK=D13
  - Fins de course : D9 (gauche), D8 (droit)
  - Les Boutons :
  Bouton_Armement (A7), Bouton_Commande (D10), BoutonCalib (D2) et Led_Armement (D5)
  - LED armement : Elle se trouve à l'intérieur du Bouton Armement
  - Drivers moteurs :
      Gauche : EN_G=A6, RPWM_G=A3, LPWM_G=A2
      Droit  : EN_D=D3, RPWM_D=D6, LPWM_D=D7
      
  - Capteurs de courant : A0 (gauche), A1 (droit)

  - Protection GPS Active : Le NEO-M8N (D0/D1) surveille en continu la vitesse pour garantir la sécurité du système.
  - Si les bras sont déployés et que la vitesse ≥30 km/h, ils se rétractent automatiquement (REPOS forcé).
  - À ≥25 km/h, tout déploiement est bloqué : moteurs coupés, LED éteinte, drivers désactivés, état REPOS imposé.
  - Le bouton reste actif, mais la logique ON/OFF garantit que la prochaine action sera un déploiement sécurisé.

    * AUTEUR : lsb71620
    * DATE : 2025
    * LICENCE :
    * Ce code est distribué sous la Licence MIT.
    * Pour plus de détails, consultez le fichier LICENSE à la racine du dépôt.
    * COPYRIGHT (©) 2025 lsb71620

  Remarques :
  - Ne pas connecter les moteurs avant calibration à vide.
  - Appui long sur bouton_Calib (5s) pour calibrer les capteurs.
  - Vérifier les seuils de courant et la logique des fins de course.
  ============================================================================
*/

// -------------------------------------------------------------------------------------
// Bibliothèques
#include <EEPROM.h> // pour stocker offsets/seuils en mémoire non-volatile (EEPROM)
#include <SD.h>     // pour la gestion de la carte microSD (D4)
#include <SPI.h>    // bus SPI utilisé par le lecteur SD

// Si intégration du GPS dans le système
#include <TinyGPS++.h>
// -------------------------------------------------------------------------------------

// ==========================
// Déclaration des broches
// ==========================

// -------------------------------------------------------------------------------------
TinyGPSPlus gps;
// Instance pour analyser les données GPS
// -------------------------------------------------------------------------------------

// Broche Chip Select pour la carte SD (D4 selon les commentaires SPI)
constexpr int chipSelectSD = D4;
// Boutons et LED
constexpr int Bouton_Armement = A7;  // Bouton armement (gauche)
constexpr int Bouton_Commande = D10; // Bouton commande (droite)
constexpr int Led_Armement = D5;     // LED Armement (Gauche)
constexpr int Bouton_Calib = D2;     // Bouton de calibration (droite)

// Fins de course - Vis 1 + Vis 2 → forment le contact NC, - Vis 3 + Vis 4 → forment le contact NO
constexpr int finCourseReplieG = D9; // Fin de course bras gauche NC (vis 1 + 2)
constexpr int finCourseReplieD = D8; // Fin de course bras droit NC (vis 1 + 2)

// Drivers moteurs
// Gauche
constexpr int EN_G = A6;   // Enable driver gauche
constexpr int RPWM_G = A3; // PWM sens 1 moteur gauche
constexpr int LPWM_G = A2; // PWM sens 2 moteur gauche

// Droit
constexpr int EN_D = D3;   // Enable driver droit
constexpr int RPWM_D = D6; // PWM sens 1 moteur droit
constexpr int LPWM_D = D7; // PWM sens 2 moteur droit

// Capteurs de courant
constexpr int ACS_G_Pin = A0; // Capteur courant gauche
constexpr int ACS_D_Pin = A1; // Capteur courant droit

// ==========================
// Constantes système et paramètres
// ==========================
// Référence ADC de la Nano R4
const float Vref = 5.0f; // 5 V (tension de référence de l'ADC sur Nano R4)
// const int ADC_MAX = 16383; // ADC 14-bits => valeurs 0 .. 16383

const int ADC_BITS = 14;
const int ADC_STEPS = (1 << ADC_BITS); // 16384
const int ADC_MAX_VAL = ADC_STEPS - 1; // 16383

// Sensibilité capteur courant (valeur typique pour ACS712 30A) : 66 mV/A
const float ACS_SENS_V_PER_A = 0.066f; // V par ampere (ex: 0.066 V/A)

// Timings / sécurité
const unsigned long dureeMaxAction = 1500UL;   // temps max pour un mouvement (ms) -> défaut si dépassé
const unsigned long delaiCalibBouton = 5000UL; // appui long sur Bouton1 pour calibration (ms)

// PWM / synchronisation
const int PWM_BASE = 200; // puissance nominale initiale (0..255)
const int PWM_MAX = 255;
const float SYNC_KP = 20.0f; // coefficient pour synchronisation courant gauche/droit

// ==========================
// Structure EEPROM (persistante)
// ==========================
// On stocke : magic (validation), offsets zéro en volts, seuils en ampères,
// et valeurs ADC pré-calculées (pour éviter de recalculer à chaque boucle)
struct Config
{
  uint32_t magic;
  float zeroOffsetV_G;    // tension de sortie capteur gauche à 0 A (volts)
  float zeroOffsetV_D;    // tension de sortie capteur droit à 0 A (volts)
  float seuilReposA;      // seuil pour considérer 'repos' (A)
  float seuilMouvementA;  // seuil pour considérer 'en mouvement' (A)
  float seuilSurchargeA;  // seuil surcharge (A) -> défaut
  uint16_t seuilReposADC; // seuils convertis en ADC (14-bit)
  uint16_t seuilMouvementADC;
  uint16_t seuilSurchargeADC;
  uint8_t lastState;
  bool nextActionIsDeployment; // CORRECTION: Renommée pour clarté et cohérence
  uint8_t checksum;            // pour validation des données
};

const uint32_t EEPROM_MAGIC = 0xA5A5A5A5; // valeur pour détecter config valide
Config cfg;                               // instance en RAM

// ==========================
// Variables pour la carte SD
// ==========================

bool sdOk = false; // État d'initialisation de la carte SD
File logFile;      // Fichier de journalisation

// ==========================
// Etats machine
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

// Variables pour gestion appui long, calibration, temps d'action
bool bouton1EtaitActif = false;
unsigned long bouton1PressMillis = 0;
bool calibratedThisPress = false;
unsigned long actionStartMillis = 0;
// ==========================
// Variables pour suivi du courant max dans le fichier log
// ==========================
float courantMaxG = 0.0;
float courantMaxD = 0.0;
// ==========================
// Fonctions utilitaires (conversion, lecture filtrée)
// ==========================

// Convertit une valeur ADC (0..ADC_MAX) en tension (V)
// Formule: V = (adc / ADC_MAX) * Vref
float adcToVoltage(int adc)
{
  // adc in [0 .. ADC_MAX_VAL]
  return ((float)adc / (float)ADC_STEPS) * Vref;
}

// Convertit une tension en valeur ADC - utile pour calculs seuils
int voltageToAdc(float v) 
{
  int a = (int)round((v / Vref) * (float)ADC_STEPS);
  if (a < 0) return 0;
  if (a > ADC_MAX_VAL) return ADC_MAX_VAL;
  return a;
}


// Convertit ampères -> valeur ADC en utilisant l'offset zero (en volts)
// Vout = zeroOffsetV + I * sensibilité
uint16_t ampsToAdc(float amps, float zeroOffsetV)
{
  float vout = zeroOffsetV + amps * ACS_SENS_V_PER_A; // tension attendue au capteur
  return (uint16_t)voltageToAdc(vout);
}

// Lecture moyenne d'un canal ADC pour réduire le bruit (rapide, non bloquante)
int readAdcAvg(int pin, int samples = 16)
{
  long s = 0;
  for (int i = 0; i < samples; ++i)
  {
    s += analogRead(pin); // Lecture rapide
    // On peut laisser les lectures se faire rapidement
  }
  return (int)(s / samples);
}

// ==========================
// Fonction de calcul de Checksum (pour EEPROM)
// ==========================
uint8_t calculateChecksum()
{
  uint8_t cs = 0;
  // Calcul du checksum sur toute la structure SAUF le dernier octet (le checksum lui-même)
  // sizeof(Config) - sizeof(cfg.checksum)
  for (size_t i = 0; i < sizeof(Config) - 1; ++i)
  {
    cs ^= *(((uint8_t *)&cfg) + i); // XOR de l'octet
  }
  return cs;
}

// ==========================
// EEPROM helpers: chargement / sauvegarde (CORRIGÉE)
// ==========================
void loadConfig()
{
  EEPROM.get(0, cfg);
  uint8_t calculated_cs = calculateChecksum();

  if (cfg.magic != EEPROM_MAGIC || cfg.checksum != calculated_cs)
  {
    Serial.println("EEPROM non initialisée ou corrompue. Chargement des valeurs par défaut.");
    // Journalisation de l'erreur EEPROM
    logEvent("!!! ALERTE: EEPROM corrompue/non init. -> Chargement des valeurs par defaut.");

    // Valeurs par défaut si EEPROM non initialisée
    cfg.magic = EEPROM_MAGIC;
    cfg.zeroOffsetV_G = 2.5f; // valeur par défaut (typique si capteur 5V) - à recalibrer
    cfg.zeroOffsetV_D = 2.5f;
    cfg.seuilReposA = 1.0f;
    cfg.seuilMouvementA = 3.0f;
    cfg.seuilSurchargeA = 15.0f;

    // Valeurs par défaut pour les états
    currentState = REPOS;
    nextActionIsDeployment = true; // Prochaine action par défaut = Déploiement

    // Calculer les valeurs ADC correspondantes (utile pour comparaisons rapides)
    cfg.seuilReposADC = ampsToAdc(cfg.seuilReposA, cfg.zeroOffsetV_G);
    cfg.seuilMouvementADC = ampsToAdc(cfg.seuilMouvementA, cfg.zeroOffsetV_G);
    cfg.seuilSurchargeADC = ampsToAdc(cfg.seuilSurchargeA, cfg.zeroOffsetV_G);

    // Calcul et sauvegarde immédiate du nouveau checksum
    cfg.checksum = calculateChecksum();
    EEPROM.put(0, cfg); // PUT pour écrire toutes les valeurs par défaut
  }
  else
  {
    // Rétablissement des états précédents
    currentState = (State)cfg.lastState;
    nextActionIsDeployment = cfg.nextActionIsDeployment;
  }
}

void saveConfig()
{
  // Mettre à jour le checksum avant de sauvegarder
  cfg.checksum = calculateChecksum();

  EEPROM.put(0, cfg); // pour écrire toute la structure
}
// ==========================
// Fonction de journalisation sur carte SD (NON BLOQUANTE)
// ==========================
void logEvent(const String &event)
{
  if (!sdOk)
  {
    // Si la carte SD n'a pas été initialisée (sdOk = false), on ignore.
    return; // Non-blocage garanti
  }

  // Tente d'ouvrir le fichier. Si le système SD est planté, cela ne doit pas geler.
  // Cependant, on ne peut pas garantir que l'appel `SD.open` lui-même ne bloque pas
  // si le hardware SPI est en défaut total. C'est le meilleur compromis logiciel.
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
    // Log d'erreur I/O sur le port série si l'écriture échoue
    Serial.print("!!! ERREUR SD : Impossible d'ouvrir LOG.TXT. Événement manqué: ");
    Serial.println(event);
  }
}
// ==========================
// Fonctions matérielles (drivers moteurs)
// ==========================

// Activer/désactiver les drivers (EN pins) - utilisé pour coupure matérielle en défaut
void enableDrivers(bool en)
{
  digitalWrite(EN_G, en ? HIGH : LOW);
  digitalWrite(EN_D, en ? HIGH : LOW);
}

// Arrête immédiatement toutes les sorties PWM (moteurs)
void stopAllMotors()
{
  analogWrite(RPWM_G, 0);
  analogWrite(LPWM_G, 0);
  analogWrite(RPWM_D, 0);
  analogWrite(LPWM_D, 0);
}

  /* ==========================
  * Définitions des fonctions moteur manquantes
  * ==========================
  */

  /**
  * Pilote le moteur GAUCHE
  * deployer = true  -> tourne dans le sens "déploiement"
  * deployer = false -> tourne dans le sens "rétraction"
  */

void setMotorGauche(bool deployer, int pwm)
{
  // NOTE : Peut-être inverser RPWM et LPWM
  // si le moteur tourne dans le mauvais sens.
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
  * Pilote le moteur DROIT
  * deployer = true  -> tourne dans le sens "déploiement"
  * deployer = false -> tourne dans le sens "rétraction"
  */
  void setMotorDroit(bool deployer, int pwm)
  {
    // NOTE :  peut-être inverser RPWM et LPWM ?
    // si le moteur tourne dans le mauvais sens.
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
// Calibration ACS (appui long Bouton1)
// ==========================
// Mesure le zero offset (Vout à 0 A) en moyenne sur N échantillons,
// puis enregistre en EEPROM.
// Utile si capteur alimenté en 3.3V/5V.
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

  Serial.println("=== Calibration ACS terminée ===");
  Serial.print("ZeroOffsetG (V) = "); Serial.println(cfg.zeroOffsetV_G, 6);
  Serial.print("ZeroOffsetD (V) = "); Serial.println(cfg.zeroOffsetV_D, 6);
  Serial.print("Seuils ADC recalculés: ");
  Serial.print(cfg.seuilReposADC); Serial.print(", ");
  Serial.print(cfg.seuilMouvementADC); Serial.print(", ");
  Serial.println(cfg.seuilSurchargeADC);

  if (abs(cfg.zeroOffsetV_G - 2.5f) > 1.0f) {
    Serial.println("Calibration suspecte : vérifiez le capteur gauche.");
  }
  if (abs(cfg.zeroOffsetV_D - 2.5f) > 1.0f) {
    Serial.println("Calibration suspecte : vérifiez le capteur droit.");
  }
}

// ==========================
// Fonction principale de synchronisation et contrôle des bras
// ==========================
// Retourne true si l'action (déploiement/retraction) est terminée
// - deployer = true  => on est en déploiement (moteurs poussent vers le bas)
// - deployer = false => on est en rétraction (moteurs remontent)

// ====================================================================
// Fonction : synchroniserBras()
// Gère les deux moteurs de manière synchronisée, avec surveillance
// du courant, des fins de course (TZ-8122) et de la désynchronisation.
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
    Serial.println("DEFAUT : surcharge détectée !");
    logEvent("!!! DEFAUT : surcharge détectée -> arrêt sécurité");

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Courants G=%.2fA D=%.2fA", courantG_A, courantD_A);
    logEvent(buffer);

    currentState = DEFAUT;
    cfg.lastState = (uint8_t)currentState;
    cfg.nextActionIsDeployment = nextActionIsDeployment;
    saveConfig();

    return true;
  }

  const float SEUIL_FIN_COURSE_COURANT = 6.0f;

  bool finReplieG = (!deployer) && (digitalRead(finCourseReplieG) == LOW);
  bool finReplieD = (!deployer) && (digitalRead(finCourseReplieD) == LOW);
  bool finDeployeG = (deployer) && (abs(courantG_A) > SEUIL_FIN_COURSE_COURANT);
  bool finDeployeD = (deployer) && (abs(courantD_A) > SEUIL_FIN_COURSE_COURANT);

  if ((finReplieG && finReplieD) || (finDeployeG && finDeployeD)) {
    stopAllMotors();
    Serial.print("Bras complètement ");
    Serial.println(deployer ? "déployés (courant)." : "repliés (FC).");
    logEvent(deployer ? "Fin de cycle OK - bras G/D déployés (courant)" : "Fin de cycle OK - bras G/D repliés (FC)");
    return true;
  }

  bool desynchronisation = (finReplieG && !finReplieD) ||
                           (!finReplieG && finReplieD) ||
                           (finDeployeG && !finDeployeD) ||
                           (!finDeployeG && finDeployeD);

  if (desynchronisation) {
    stopAllMotors();
    enableDrivers(false);
    Serial.println("DEFAUT : désynchronisation détectée !");
    logEvent("DEFAUT : désynchronisation détectée (blocage non synchrone)");

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
// Gestion de l'appui long pour calibration
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

//  la LED Armement (D5) doit s'allumer pour confirmer que le système est armé et que le bouton de commande (D10) peut être utilisé.
// Si la LED est éteinte, l'action du bouton de commande est ignorée.
// ==========================
// Machine d'état et gestion des boutons (CORRIGÉE)
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

    if (nextActionIsDeployment) logEvent(">>> DEBUT DEPLOIEMENT");
    else logEvent(">>> DEBUT RETRACTION");
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
        Serial.println("!!! DEFAUT: timeout action -> coupure drivers");
        logEvent("!!! DEFAUT: timeout - coupure drivers");

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
                   "Fin cycle OK - Courant max G=%.2fA D=%.2fA", courantMaxG, courantMaxD);
          logEvent(buffer);
        }

        logEvent(">>> ACTION TERMINEE OK");
      }
      break;

    case DEFAUT:
      digitalWrite(Led_Armement, LOW);
      break;
  }
}

// ==========================
// Setup et loop principaux
// Code exécuté une seule fois au démarrage
// ==========================
void setup() {
  // -------------------------------------------------------------------------------------
  // Initialisation du GPS (port série matériel Serial1 sur D0/D1)
  // -------------------------------------------------------------------------------------
  Serial1.begin(9600);

  // Initialisation du port série pour le debug
  Serial.begin(115200);

  // Configuration de l'ADC 14 bits (Nano R4)
  analogReadResolution(14); // Résolution ADC : 0..16383

  // Configuration des broches
  pinMode(Bouton_Armement, INPUT_PULLUP);
  pinMode(Bouton_Commande, INPUT_PULLUP);
  pinMode(Bouton_Calib, INPUT_PULLUP);
  pinMode(finCourseReplieG, INPUT_PULLUP);
  pinMode(finCourseReplieD, INPUT_PULLUP);
  pinMode(Led_Armement, OUTPUT);
  digitalWrite(Led_Armement, LOW); // LED éteinte au démarrage

  // Moteur gauche
  pinMode(EN_G, OUTPUT);
  pinMode(RPWM_G, OUTPUT);
  pinMode(LPWM_G, OUTPUT);

  // Moteur droit
  pinMode(EN_D, OUTPUT);
  pinMode(RPWM_D, OUTPUT);
  pinMode(LPWM_D, OUTPUT);

  // Sécurité au démarrage
  stopAllMotors();
  enableDrivers(false);

  // Chargement de la configuration EEPROM
  loadConfig();

  // Vérification des fins de course au démarrage
  bool finG = (digitalRead(finCourseReplieG) == LOW);
  bool finD = (digitalRead(finCourseReplieD) == LOW);

  if (finG && finD) {
    currentState = REPOS;
    cfg.nextActionIsDeployment = true;
    cfg.lastState = (uint8_t)currentState;
    saveConfig();
    Serial.println("Bras détectés comme repliés au démarrage → état corrigé à REPOS");
  }

  // -------------------------------------------------------------------------------------
  // Initialisation de la carte microSD
  // -------------------------------------------------------------------------------------
  Serial.print("Initialisation carte SD... ");
  if (!SD.begin(chipSelectSD)) {
    Serial.println("ECHEC : Carte SD non trouvée ou non initialisée.");
    sdOk = false;
  } else {
    Serial.println("OK Carte SD prête !");
    sdOk = true;

    logFile = SD.open("LOG.TXT", FILE_WRITE);
    if (logFile) {
      logFile.println("==== DEMARRAGE SYSTEME ====");
      logFile.close();
    }
  }
}

unsigned long lastSerialPrint = 0;

void loop() {
  // =========================================================================
  // Code exécuté en boucle tant que la carte est alimentée
  // =========================================================================

  // Lecture des données GPS
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  float currentSpeedKMPH = gps.speed.kmph();

  if (gps.speed.isValid()) {
    // Rétraction automatique si bras déployés (nextActionIsDeployment=false) et vitesse ≥ 30 km/h
    if (currentSpeedKMPH >= 30.0 && currentState == REPOS && !cfg.nextActionIsDeployment) {
      Serial.println("Sécurité : vitesse > 30 km/h avec bras déployés → rétraction forcée");
      logEvent(">>> Sécurité : rétraction automatique à 30 km/h");

      currentState = RETRACTION;
      cfg.nextActionIsDeployment = true; // Prépare le prochain cycle pour déploiement
      actionStartMillis = millis();
      enableDrivers(true);
    }

    // Blocage si bras repliés commandés (nextActionIsDeployment=true) à ≥25 km/h ou vitesse ≥30 km/h
    if ((cfg.nextActionIsDeployment && currentSpeedKMPH >= 25.0) || currentSpeedKMPH >= 30.0) {
      if (currentState != REPOS) {
        logEvent("!!! SECURITE : BLOCAGE VITESSE actif (Arret action à " + String(currentSpeedKMPH) + " km/h)");
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

  // Gestion calibration et bras
  gestionBoutonCalibration(); // Appui long sur bouton calibrage
  gestionBras();              // Machine d'état (boutons, moteurs, fins de course)

  // =========================================================================
  // AFFICHAGE PÉRIODIQUE (toutes les 500 ms)
  // =========================================================================
  if (millis() - lastSerialPrint > 500) {
    int adcG = readAdcAvg(ACS_G_Pin, 12);
    int adcD = readAdcAvg(ACS_D_Pin, 12);
    float vG = adcToVoltage(adcG);
    float vD = adcToVoltage(adcD);
    float iG = (vG - cfg.zeroOffsetV_G) / ACS_SENS_V_PER_A;
    float iD = (vD - cfg.zeroOffsetV_D) / ACS_SENS_V_PER_A;

    const float ALERTE_FACTOR = 1.5;
    bool alerteFlottant = (currentState == REPOS) &&
                          ((abs(iG) > cfg.seuilReposA * ALERTE_FACTOR) || (abs(iD) > cfg.seuilReposA * ALERTE_FACTOR));
    bool alertePhysiqueFlottant = (adcG < 100 || adcG > 16000) || (adcD < 100 || adcD > 16000);

    static bool printedHardwareAlert = false;
    if (!printedHardwareAlert) {
      Serial.println();
      Serial.println("RAPPEL MATÉRIEL : Composants clés (ACS712, BTS7960) non connectés.");
      Serial.println("Le système est en mode simulation/test jusqu'à connexion.");
      Serial.println("Appui long sur Bouton 1 (5s) pour calibrer les capteurs.");
      Serial.println();
      printedHardwareAlert = true;
    }

    if (alerteFlottant || alertePhysiqueFlottant) {
      Serial.println("  ATTENTION: Capteurs de courant (A0/A1) potentiellement");
      Serial.println("  déconnectés ou mal calibrés. Valeurs lues :");
      Serial.print("  ADC G:"); Serial.print(adcG); Serial.print("  ");
      Serial.print("ADC D:"); Serial.print(adcD); Serial.print("  ");
      Serial.print("I G:"); Serial.print(iG, 3); Serial.print("A  ");
      Serial.print("I D:"); Serial.print(iD, 3); Serial.print("A");
      Serial.println();
      Serial.println("  Action: Vérifiez le câblage ou lancez calibration (Bouton 1 long).");
      Serial.println("========================================================");
    } else {
      Serial.print("ADC G:"); Serial.print(adcG); Serial.print("  ");
      Serial.print("ADC D:"); Serial.print(adcD); Serial.print("  ");
      Serial.print("I G:"); Serial.print(iG, 3); Serial.print("A  ");
      Serial.print("I D:"); Serial.print(iD, 3); Serial.print("A  ");
      Serial.print("Etat:");
      switch (currentState) {
        case REPOS:       Serial.print("REPOS"); break;
        case DEPLOIEMENT: Serial.print("DEPLOIEMENT"); break;
        case RETRACTION:  Serial.print("RETRACTION"); break;
        case DEFAUT:      Serial.print("DEFAUT"); break;
      }
      Serial.print("  SeuilSurchargeADC:");
      Serial.print(cfg.seuilSurchargeADC);
      Serial.println();
    }

    lastSerialPrint = millis();
  }
}
