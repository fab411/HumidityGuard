// Der Code ist eine Regelung zur Trockenbox für Pflanzenzuschnitt in einem Karton
// Dieses impliziert 
// eine Feuchtigkeitsmessung mit DHT22 (AM2302) 
// Zur Umsetzung ist eine Lüftersteuerung mit Hysteresenverhalten,
// ein gleitender Mittelwert und 
// Regelung anhand Temperatur Thresholds und deren Anzeige mit drei LED
// dynamischer Konstruktor für Thresholds
// Retry-Mechanismus wenn Lesen der Werte vom Sensor fehlschlägt und Status LED
// Nicht-blockierende Verzögerungen lassen sich durch die Verwendung von millis() statt delay() implementieren. Dabei wird ein Timer-Konzept genutzt, um die Zeitspanne zu messen, die seit dem letzten Aufruf vergangen ist, ohne den Haupt-Loop zu blockieren.
// Mehrstufige Fehlerbehandlung durch SystemState-Enum verwaltet verschiedene Betriebsmodi 
//mit Status-LED für verschiedene Fehlertypen, speichern und Begrenzen der Fehleranzahl, automatische Wiederherstellung nach Fehlern und Abschließende Fehlermeldung nach mehrfachem Fehlversuch und Statusüberwachung mit Zuständen  
//Automatische Fehlerwiederholung bis anzahl Sensorleseversuche
// Modularisierte Lüftersteuerung
// Adaptive Schwellenwerte, Die PID-Steuerung für den Lüfter würde die Luftfeuchtigkeit als Regelgröße verwenden, um den Lüfter so zu steuern, dass die Luftfeuchtigkeit den Zielwert erreicht.
// Lüftersteuerung mit einem Feuchtigkeitssensor könnten wir die Kalibrierung verwenden, um den Schwellenwert für die Luftfeuchtigkeit basierend auf den Umgebungsbedingungen dynamisch zu setzen.  
// verwendet wurden ein arduino nano, eine Motorbrücke und 4 LED mit Vorwiederstand und ein Feuchtigkeitssensor DHT22 und ein 12V Lüfter

//weitere Ideen: EEPROM-Speicherung, Lüfter Regelung PWM-Signal, Hardware oder Pin-Mappings über eine zentrale config.h-Datei, Logs zur Fehleraufzeichnung und Speicherung der letzten X-Feuchtigkeits- und Temperaturwerte. Dies könnte einfach als Array oder zyklischer Puffer im RAM gespeichert werden
// Datenspeicherung und Visualisierung,  

///////////////////// Header ////////////////////////
#include <DHT.h> // DHT Sensor Library Adafruit
// Definitionen für den DHT-Sensor
#define DHT_PIN 11 // Pin für den DHT-Sensor
#define DHT_TYPE DHT22 // DHT22 (AM2302)

// Initialisierung des DHT-Sensors
DHT dht(DHT_PIN, DHT_TYPE);

// Pins für den Lüfter und die LEDs
const int FAN1 = 5; // Haupt-Lüfter
const int LUEFTER_EIN = 4; //Steuerung Lüfter ein
const int LUEFTER_AUS = 3; //Steuerung Lüfter aus
const int led1Pin = 10; // LED erster Schwellenwert
const int led2Pin = 9; // LED zweiter Schwellenwert
const int led3Pin = 8; // LED dritter Schwellenwert
const int errorLedPin = 7; // LED für Fehleranzeige

// Einstellungen für Fehlerwiederholungen und Zeitverzögerungen
const int maxRetries = 3; // Maximalanzahl an Versuchen beim Lesen des Sensors
int errorCount = 0;
// Nicht-blockierende Verzögerungen durch millis(), Status-LED für verschiedene Fehlertypen:
unsigned long previousMillis = 0;  // Zeitstempel der letzten Sensorablesung
const long retryInterval = 5000; // Intervall für Sensorablesung (5 Sekunden)
const long ledBlinkInterval = 500; // // Intervall für Fehler-LED-Blinken im Wartungsmodus (0.5 Sekunden)

// Betriebszustände
enum SystemState { NORMAL, SENSOR_ERROR, FAN_ERROR, MAINTENANCE }; //Normalbetrieb, Sensorfehler, Lüfterfehler, Wartung
SystemState systemState = NORMAL;

// Struktur für PID-Parameter und Schwellenwerte
struct ControlSettings {
  // PID-Parameter
  float Kp, Ki, Kd;   // Proportionaler Koeffizient, Integral-Koeffizient, Differential-Koeffizient
  float previousError, integral; // Fehlerberechnung für PID-Regelung
  float targetHumidity; // Zielwert der Luftfeuchtigkeit
  float ThresholdOn, ThresholdOff; // Adaptive Schwellenwerte für die Lüftersteuerung
  bool fanStatus; // Lüfterstatus (an/aus)

  // Konstruktor für PID 
  ControlSettings(float kp, float ki, float kd, float targetHum, 
                  float onThreshold, float offThreshold)
    : Kp(kp), Ki(ki), Kd(kd), previousError(0), integral(0), targetHumidity(targetHum), 
      ThresholdOn(onThreshold), ThresholdOff(offThreshold),
      fanStatus(false) {}
};

// Struktur für die Steuerung der LED-Schwellenwerte
struct LEDControlSettings {
  float led1, led2, led3;  // Schwellenwerte für die LEDs

  // Konstruktor für die Initialisierung der LED-Schwellenwerte
  LEDControlSettings(float led1Thresh, float led2Thresh, float led3Thresh)
    : led1(led1Thresh), led2(led2Thresh), led3(led3Thresh) {}
};


// Initialisieren der Steuerungsparameter
float baseHumidity = 60;
ControlSettings settings(2.0, 0.1, 1.0, baseHumidity, baseHumidity + 5.0, baseHumidity - 5.0);
LEDControlSettings setLED(50.0, 60.0, 70.0);

// Lüfterstatus (an/aus)
bool fanStatus = false;

///////////////////// Setup-Funktionen ////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println(F("START"));

  // Sensor und Hardware-Komponenten vorbereiten
  setupDHTSensor();
  setupFanAndLEDs();

  // Initialisierung der Fehler-LED
  pinMode(errorLedPin, OUTPUT);
  digitalWrite(errorLedPin, LOW);

}

/////////////////  Initialisierung des DHT-Sensors ///////////////// 
void setupDHTSensor() {
  dht.begin();
}

///////////////// Initialisierung der Lüfter und LED //////////////////////
void setupFanAndLEDs() {
  // Pin-Modi für den Lüfter
  pinMode(FAN1, OUTPUT);
  pinMode(LUEFTER_EIN, OUTPUT);
  pinMode(LUEFTER_AUS, OUTPUT);

  // Pin-Modi für die LEDs
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);

  // Lüfter und LEDs initial ausschalten
  // Pin_Modi error LED
  pinMode(errorLedPin, OUTPUT);
  digitalWrite(errorLedPin, LOW);
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  digitalWrite(led3Pin, LOW);
}


///////////////////// Loop und Hauptfunktionen ////////////////////////
void loop() {
  if (systemState == MAINTENANCE) {
    blinkErrorLED(ledBlinkInterval);
    return;
  }

  // Nicht-blockierende Verzögerungen mit Sensorablesung im festgelegten Intervall
  if (millis() - previousMillis >= retryInterval) {
    previousMillis = millis();
    // Luftfeuchtigkeit und Temperatur messen mit Fehlerbehandlung
    float h, t; //float h = dht.readHumidity(); //float t = dht.readTemperature();
    
    // Fehler-LED einschalten und Fehler melden
    if (!readDHTWithRetry(h, t)) { 
      systemState = SENSOR_ERROR;
      errorCount++;
      if (errorCount >= maxRetries) {
        Serial.println("Sensorfehler: Wartungsmodus aktiviert.");
        systemState = MAINTENANCE;
      } else {
        Serial.println("Sensorfehler: Wiederholung in 5 Sekunden.");
      }
      return;
    }

    // Fehler-LED ausschalten, da Messung erfolgreich war
    digitalWrite(errorLedPin, LOW);
    errorCount = 0;
       
    // Berechnung des gleitenden Durchschnitts der Luftfeuchtigkeit und Temperatur
    float averageHumidity = calculateMovingAverage(h);
    float averageTemperature = calculateMovingAverage(t);
;   // Grundwerte als Referenzwerte speichern, falls benötigt (z.B. in Settings oder globalen Variablen)blen)

    // LED-Steuerung
    handleLEDs(averageHumidity);
      
    // Lüftersteuerung
    // handleFan(averageHumidity);
    handleFanWithPID(averageHumidity);
      
    systemState = NORMAL;

    Serial.print(F("Luftfeuchtigkeit: "));
    Serial.print(averageHumidity);
    Serial.print(F("%  Temperatur: "));
    Serial.print(averageTemperature);
    Serial.println(F("°C"));
  }
}

///////////////////// Unterstützende FUNKTIONEN ////////////////////////

// Funktion zum Lesen des DHT-Sensors mit Retry-Mechanismus
bool readDHTWithRetry(float &humidity, float &temperature) {
  for (int i = 0; i < maxRetries; i++) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    // Überprüfen, ob die Messwerte gültig sind
    if (!isnan(humidity) && !isnan(temperature)) {
      return true; // Erfolgreiches Lesen
    }
    Serial.print("Sensorlesefehler. Versuch ");
    Serial.print(i + 1);
    Serial.println(" von 3...");
    delay(1000); // Kleine Verzögerung vor dem nächsten Versuch
  }
  return false; // Alle Versuche fehlgeschlagen
}

void blinkErrorLED(unsigned long interval) {
  static unsigned long previousBlinkMillis = 0;
  if (millis() - previousBlinkMillis >= interval) {
    previousBlinkMillis = millis();
    digitalWrite(errorLedPin, !digitalRead(errorLedPin));
  }
}


// Funktion zur Steuerung des Lüfters basierend auf PID
void handleFanWithPID(float currentHumidity) {
  // PID-Regelung anwenden, um den Lüfter entsprechend der aktuellen Luftfeuchtigkeit zu steuern
  float pidOutput = calculatePID(currentHumidity);

  // Wenn der PID-Ausgang positiv ist, den Lüfter einschalten
  if (pidOutput > 0 && !settings.fanStatus && currentHumidity > settings.ThresholdOn) {
    turnOnFan();  // Lüfter einschalten
  } 
  // Wenn der PID-Ausgang negativ ist, den Lüfter ausschalten
  else if (pidOutput < 0 && settings.fanStatus && currentHumidity < settings.ThresholdOff) {
    turnOffFan();  // Lüfter ausschalten
  }
}

void turnOnFan() {
  // Lüfter einschalten
  digitalWrite(LUEFTER_EIN, LOW);
  digitalWrite(LUEFTER_AUS, HIGH);
  digitalWrite(FAN1, HIGH);
  settings.fanStatus = true;
  systemState = NORMAL;
  Serial.println(F("Lüfter eingeschaltet!"));
}

void turnOffFan() {
  // Lüfter ausschalten und Pins zurücksetzen
  digitalWrite(FAN1, LOW);
  // Beide Steuer-Pins auf LOW setzen, um den Lüfter vollständig auszuschalten
  digitalWrite(LUEFTER_EIN, LOW);  
  digitalWrite(LUEFTER_AUS, LOW); 
  settings.fanStatus = false;
  Serial.println(F("Lüfter ausgeschaltet und zurückgesetzt!"));
}

/////////////////// Funktion zur Steuerung der LEDs basierend auf den Schwellenwerten ////////////////////
// currentHumidity >= thresholds.ledX gibt entweder true (1) oder false (0), damit direkt an digitalWrite()
void handleLEDs(float currentHumidity) {
  // LED1 leuchtet bei überschreiten von led1
  digitalWrite(led1Pin, currentHumidity >= setLED.led1);
  // LED2 leuchtet bei überschreiten von led2
  digitalWrite(led2Pin, currentHumidity >= setLED.led2);
  // LED3 leuchtet bei überschreiten von led3
  digitalWrite(led3Pin, currentHumidity >= setLED.led3);
  
  // Fehler-LED steuern, z.B. bei Sensorausfällen
  if (isnan(currentHumidity)) {
    digitalWrite(errorLedPin, HIGH);
  } else {
    digitalWrite(errorLedPin, LOW);
  }
}

// Funktion zur Berechnung des PID-Fehlers
float calculatePID(float currentHumidity) {
  // Fehler berechnen
  float error = settings.targetHumidity - currentHumidity;
  // Proportionaler Anteil
  float P = settings.Kp * error;
  // Integralanteil
  settings.integral += error;
  float I = settings.Ki * settings.integral;
  // Differentialanteil
  float D = settings.Kd * (error - settings.previousError);
  // Gesamt PID-Ausgang
  float output = P + I + D;
  // Fehler für den nächsten Zyklus speichern
  settings.previousError = error;
  return output;
}


/////////////// Berechnung des gleitenden Durchschnitts der Luftfeuchtigkeit /////////////
// Jedes Mal, wenn eine neue Messung durchgeführt wird, wird der Wert an der aktuellen 
// readIndex-Position im Array aus der Summe total entfernt und durch den neuen Messwert ersetzt.
float calculateMovingAverage(float newHumidity) {
  // statische Variablen für den gleitenden Mittelwert
  static const int numReadings = 10;
  static float humidityReadings[numReadings];
  static int readIndex = 0;
  static float total = 0;
  
  // Den alten Wert von der Summe abziehen
  total = total - humidityReadings[readIndex];
  // Den neuen Wert in den Puffer speichern
  humidityReadings[readIndex] = newHumidity;
  // Den neuen Wert zur Summe hinzufügen
  total = total + humidityReadings[readIndex];
  // Index für den nächsten Wert erhöhen (Ringpuffer)
  readIndex = (readIndex + 1) % numReadings;
  // Gleitenden Durchschnitt berechnen
  return total / numReadings;
}
