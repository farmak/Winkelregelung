#include <RCSwitch.h>  // Bilbiothek für Funksender und Funksteckdose einbinden





//////////////////////////////////////////////////////////////////////////////////
RCSwitch schalten = RCSwitch(); // Deklaration der Funktion "schalten"

float phi_soll = 0; // Variable für den Sollwinkel
float phi_ist = 0;
float Toleranz = 0;

void setup() {

  // Starte serielle Datenverbindung  
  Serial.begin(9600);
  
  //Eingabgeaufforderung
  Serial.println("Gib den Sollwinkel ein...");

  // Datenpin des Funksenders an Pin 10 angeschlossen
  mySwitch.enableTransmit(10);

}

void loop() {
  
  // Warte auf Eingabe
  while (Serial.available() != 0) {

    // Sollwinkel = eingegebener Zahlenwert
    phi_soll = Serial.parseFloat();

    // Umrechnen auf den Bereich von 0° bis 360°
    if (phi_soll < 0) {
      phi_soll += 360;
    }
    if (phi_soll > 360) {
      phi_soll -= 360; 
    }
    
    // Ausgabe des empfangenen Sollwerts
    Serial.print("Sollwert: ");
    Serial.println(phi_soll);
  }

  // Vergleiche Soll- und Ist-Wert  
  if (phi_soll != phi_ist + Toleranz | phi_soll != phi_ist - Toleranz) { //Toleranzwerte festlegen
    
    // Schalte Steckdose ein falls Ist-Wert ungleich Soll-Wert
    schalte.switchOn("11001", "01000"); //Binärcode der Stellung der Dip-Schalter entsprechend     
  }
  else {
    
    // Schalte Steckdose aus
    schalte.switchOff("11001", "01000");
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////
