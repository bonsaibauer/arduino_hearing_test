#include <Arduino.h> //AIPIC_Opta 1.2.0, ArduinoRS485 1.1.0, Arduino_DebugUtils 1.4.0, Arduino_Opta_Blueprint 0.2.5, Firmata 2.5.9
#include <SoftwareSerial.h> //EspSoftwareSerial 8.1.0, Arduino_SerialUpdater 0.0.1
#include <DFRobotDFPlayerMini.h> //DFRobotDFPlayerMini 1.0.6
#include <LiquidCrystal_I2C.h> //LiquidCrystal I2C 1.1.2

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX - Software Serial zur Kommunikation mit dem DFPlayer Mini
DFRobotDFPlayerMini myDFPlayer; // DFPlayer Mini-Bibliothek zur Audiosteuerung

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C-LCD-Bildschirm zur Anzeige (16x2)

const int swButtonPin = 2; // Pin für den S1-Button
int currentVolume = 0; // Aktuelle Lautstärke (0 bis 30)
int currentFrequencyIndex = 1; // Aktuelle Frequenzindex (1 = 125 Hz, usw.)
bool isLeftEar = false; // Variable zur Unterscheidung zwischen rechtem und linkem Ohr

// Frequenzen
const unsigned int frequencies[] = {125, 250, 500, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000};
bool completed = false; // Durchgang abgeschlossen
unsigned long lastVolumeIncreaseTime = 0; // Zeitstempel für die Lautstärkeanpassung

void setup() {
  mySoftwareSerial.begin(9600); // Software Serial-Kommunikation
  Serial.begin(9600); // Serielle Kommunikation

  lcd.init(); // LCD initialisieren
  lcd.backlight(); // LCD-Hintergrundbeleuchtung einschalten

  // Anfangsnachricht auf dem LCD
  lcd.setCursor(1, 0);
  lcd.print("S1 zum starten");
  lcd.setCursor(4, 1);
  lcd.print("druecken!");

  pinMode(swButtonPin, INPUT_PULLUP); // S1 als Eingang mit Pull-Up-Widerstand

  // Warten auf das Drücken von S1 zum Starten
  while (digitalRead(swButtonPin) == HIGH) {
    // Warten, bis der Knopf gedrückt wird
  }

  // "Rechtes Ohr" anzeigen, bevor der rechte Ohrtest startet
  showEarIndicator("Rechtes Ohr");
  delay(1000); // Anzeige für eine Sekunde

  // Start des Lautstärkentests
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Check connection!"));
    Serial.println(F("2. Insert SD card!"));
    while (true); // Endlosschleife bei Fehler
  }

  myDFPlayer.volume(currentVolume); // Initiale Lautstärke setzen
  myDFPlayer.play(currentFrequencyIndex); // Erster Frequenz-Track
  updateDisplay(); // LCD-Anzeige aktualisieren
}

void loop() {
  unsigned long currentTime = millis();

  // Automatische Lautstärkeerhöhung alle 800 ms
  if (currentTime - lastVolumeIncreaseTime >= 800 && currentVolume < 30) {
    lastVolumeIncreaseTime = currentTime; // Zeitstempel aktualisieren
    currentVolume++; // Lautstärke erhöhen
    myDFPlayer.volume(currentVolume); // Lautstärke einstellen
    updateDisplay(); // Anzeige aktualisieren
  }

  // S1-Button-Abfrage für Frequenzwechsel
  if (digitalRead(swButtonPin) == LOW) {
    delay(50); // Entprellungszeit
    if (digitalRead(swButtonPin) == LOW) { // Prüfen, ob S1 noch gedrückt ist
      nextFrequency(); // Nächste Frequenz aufrufen
      while (digitalRead(swButtonPin) == LOW); // Warten, bis der Button losgelassen wird
    }
  }
}

void updateDisplay() {
  lcd.clear(); // LCD löschen
  lcd.setCursor(0, 0);
  lcd.print("Volume:");
  lcd.print(currentVolume); // Aktuelle Lautstärke anzeigen

  lcd.setCursor(0, 1);
  lcd.print("Frequenz:");
  lcd.print(frequencies[currentFrequencyIndex - 1]); // Aktuelle Frequenz anzeigen
  lcd.print("Hz");
  lcd.setCursor(15, 0);
  lcd.print(isLeftEar ? 'L' : 'R'); // 'L' für links, 'R' für rechts
}

void nextFrequency() {
  // Lautstärke zurücksetzen und ggf. zwischen rechtem und linkem Ohr wechseln
  currentVolume = 0; // Lautstärke zurücksetzen
  myDFPlayer.volume(currentVolume); // Lautstärke setzen

  // Falls alle Frequenzen für das rechte Ohr durchlaufen sind, zum linken Ohr wechseln
  if (currentFrequencyIndex >= sizeof(frequencies) / sizeof(frequencies[0])) {
    if (!isLeftEar) {
      showEarIndicator("Linkes Ohr"); // Linkes Ohr anzeigen
      delay(1000); // Anzeige für eine Sekunde
      isLeftEar = true; // Wechselt zum linken Ohr
      currentFrequencyIndex = 1; // Frequenzindex auf Anfang setzen
    } else {
      // Wenn alle Frequenzen für beide Ohren durchlaufen sind
      if (!completed) {
        completed = true;
        lcd.clear(); // LCD löschen
        lcd.setCursor(4, 0);
        lcd.print("Ende");
        lcd.setCursor(2, 1);
        lcd.print("Durchgang");
        while (true) {} // Endlosschleife zur Beendigung
      }
    }
  } else {
    currentFrequencyIndex++; // Zum nächsten Frequenzindex wechseln
  }

  myDFPlayer.play(currentFrequencyIndex); // Neue Frequenz abspielen
  updateDisplay(); // LCD aktualisieren
  delay(500); // Entprellungszeit
}

void showEarIndicator(const char* earText) {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print(earText); // Zeige "Rechtes Ohr" oder "Linkes Ohr"
}
