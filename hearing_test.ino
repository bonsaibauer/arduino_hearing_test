#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX - Software Serial zum Kommunizieren mit dem DFPlayer Mini
DFRobotDFPlayerMini myDFPlayer; // DFPlayer Mini-Bibliothek für die Audiosteuerung umbenenen in myDFPlayer

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C-LCD-Bildschirm zur Anzeige von Informationen (16x2)

const int buttonIncreasePin = 5; // Pin zur Erhöhung des Volumens
const int buttonDecreasePin = 6; // Pin zur Verringerung des Volumens
const int swButtonPin = 2; // Pin für den Schalter

int currentVolume = 0; // Aktuelle Lautstärke (Wert zwischen 0 und 30)
int currentFrequencyIndex = 1; // Aktuelle Frequenz (0 = 0000, 1 = 125 Hz, 2 = 250 Hz, usw.)

// Array mit den Frequenzen in Hertz
const unsigned int frequencies[] = {0, 125, 250, 500, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000, 125, 250, 500, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000};

// Array mit 'R' und 'L' entsprechend den Frequenzen (ASCII-Werte)
const char side[] = {' ', 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76};

bool completed = false; // Variable, um den Durchgang abzuschließen

void setup() {
  mySoftwareSerial.begin(9600); // Initialisieren der Software Serial-Kommunikation
  Serial.begin(9600); // Initialisieren der seriellen Kommunikation

  lcd.init(); // Initialisieren des LCD-Bildschirms
  lcd.backlight(); // Einschalten der Hintergrundbeleuchtung des LCD

  // Anfangsnachricht auf dem LCD-Bildschirm
  lcd.setCursor(1, 0);
  lcd.print("S1 zum starten");
  lcd.setCursor(4, 1);
  lcd.print("druecken!");

  pinMode(buttonIncreasePin, INPUT_PULLUP); // Konfigurieren des Pin zum Erhöhen des Volumens als Eingang mit Pull-Up-Widerstand
  pinMode(buttonDecreasePin, INPUT_PULLUP); // Konfigurieren des Pin zum Verringern des Volumens als Eingang mit Pull-Up-Widerstand
  pinMode(swButtonPin, INPUT_PULLUP); // Konfigurieren des Schalter-Pins als Eingang mit Pull-Up-Widerstand

  // Warten auf das Drücken von S1 zum Starten
  while (digitalRead(swButtonPin) == HIGH) {
    // Warten, bis der Knopf gedrückt wird
  }

  if (!myDFPlayer.begin(mySoftwareSerial)) {
    // Wenn die Kommunikation mit dem DFPlayer Mini fehlschlägt, eine Fehlermeldung ausgeben
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true); // In einer Endlosschleife bleiben
  }
  myDFPlayer.volume(currentVolume); // Setzen der initiale Lautstärke
  myDFPlayer.play(1); // Starten der Audiowiedergabe mit Track 0001 (125 Hz)
  updateDisplay(); // Aktualisieren der Anzeige auf dem LCD
}

void loop() {
  // Überwachen der Tasten zum Erhöhen und Verringern des Volumens
  int increaseButtonState = digitalRead(buttonIncreasePin);
  int decreaseButtonState = digitalRead(buttonDecreasePin);

  if (increaseButtonState == LOW && currentVolume < 30) {
    currentVolume = constrain(currentVolume + 1, 0, 30); // Erhöhen der Lautstärke, aber nicht über 30
    myDFPlayer.volume(currentVolume); // Aktualisieren der Lautstärke
    updateDisplay(); // Aktualisieren der Anzeige
    delay(250); // Verzögerung zur Entprellung
  }

  if (decreaseButtonState == LOW && currentVolume > 0) {
    currentVolume = constrain(currentVolume - 1, 0, 30); // Verringern der Lautstärke, aber nicht unter 0
    myDFPlayer.volume(currentVolume); // Aktualisieren der Lautstärke
    updateDisplay(); // Aktualisieren der Anzeige
    delay(250); // Verzögerung zur Entprellung
  }

  // Überwachen des SW-Buttons, um den aktuellen Track weiterzuschalten
  if (digitalRead(swButtonPin) == LOW) {
    // Überprüfen, ob es sich nicht um die nullte Frequenz handelt, bevor Daten gespeichert werden
    if (currentFrequencyIndex > 0) {
      Serial.println(currentVolume); // Ausgabe des Lautstärkewerts in CSV-Format
    }
  
    currentFrequencyIndex++;
    if (currentFrequencyIndex >= sizeof(frequencies) / sizeof(frequencies[0])) {
      // Wenn der Durchgang abgeschlossen ist
      if (!completed) {
        completed = true;
        lcd.clear(); // Löschen des LCD-Bildschirms
        lcd.setCursor(4, 0);
        lcd.print("Ende");
        lcd.setCursor(2, 1);
        lcd.print("Durchgang");
        while (true) {
          // In einer Endlosschleife bleiben, um das Programm anzuhalten
        }
      }
      currentFrequencyIndex = 1; // Zurück auf 125 Hz, wenn das Ende erreicht ist (Index 0 überspringen)
    }
    myDFPlayer.play(currentFrequencyIndex); // Abspielen des entsprechenden Tracks
    updateDisplay(); // Aktualisieren der Anzeige
    delay(500); // Verzögerung zur Entprellung
  }
}

void updateDisplay() {
  lcd.clear(); // Löschen des LCD-Bildschirms
  lcd.setCursor(0, 0);
  lcd.print("Volume:");
  lcd.print(currentVolume); // Anzeige der aktuellen Lautstärke

  lcd.setCursor(0, 1);
  lcd.print("Frequenz:");
  lcd.print(frequencies[currentFrequencyIndex]); // Anzeige der aktuellen Frequenz
  lcd.print("Hz");
  lcd.setCursor(15, 0);
  lcd.print(char(side[currentFrequencyIndex])); // Wandeln des ASCII-Werts in ein Zeichen um und Anzeige
}
