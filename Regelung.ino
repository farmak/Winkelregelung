#include <RCSwitch.h>  // Bilbiothek für Funksender und Funksteckdose einbinden
#define SCLK1 28
#define SDIO1 26  
#define NRESET1 24
#define NCS1 22 
#define SCLK2 52
#define SDIO2 50 
#define NRESET2 48
#define NCS2 46 
#define REG_PRODUCT_ID 0x00
#define REG_REVISION_ID 0x01
#define REG_MOTION 0x02
#define REG_DELTA_X 0x03
#define REG_DELTA_Y 0x04
#define REG_SQUAL 0x05

RCSwitch mySwitch = RCSwitch(); // Deklaration der Funktion "myswitch"

float x = 0;
float y = 0; 
float x1 = 0; 
float y1 = 0; 
float x2 = 0;
float y2 = 0;
float xi1 = 0;
float yi1 = 0;
float xi2 = 0;
float yi2 = 0;
float d = 859; 
float phi;
float phi_p;
float phi_pp;

float phi_soll = 0; // Variable für den Sollwinkel
float phi_ist = 0;
float Toleranz = 0;

byte motion1;  // Freezes DX and DY until they are read or MOTION is read again.
int dx1; 
int dy1; 
byte pixelsum1; 

char motion2; 
int dx2;  
int dy2; 
byte pixelsum2;

float a1 = atan(dy1/dx1);
float s1 = y1*sin(a1); 
float phi1 = (s1/d)*360;

float a2 = atan(dy2/dx2);
float s2 = y2*sin(a2); 
float phi2 = (s2/d)*360; 

float tau;
float t_aus = 3*tau;
float t_soll = (phi_soll - phi)/phi_p;
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS
/////////////////////

byte pullByte1() {
  pinMode (SDIO1, INPUT);

  delayMicroseconds(100); // tHOLD = 100us min.
  
  byte res1 = 0;
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK1, LOW);
    res1 |= i * digitalRead (SDIO1);
    delayMicroseconds(100);
    digitalWrite (SCLK1, HIGH);
  }

  return res1;
}

byte pullByte2() {
  pinMode (SDIO2, INPUT);

  delayMicroseconds(100); // tHOLD = 100us min.
  
  byte res2 = 0;
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK2, LOW);
    res2 |= i * digitalRead (SDIO2);
    delayMicroseconds(100);
    digitalWrite (SCLK2, HIGH);
  }

  return res2;
}

void pushByte1(byte data1){
  pinMode (SDIO1, OUTPUT);
  
  delayMicroseconds(100); // tHOLD = 100us min.
  
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK1, LOW);
    digitalWrite (SDIO1, (data1 & i) != 0 ? HIGH : LOW);
    delayMicroseconds(100);
    digitalWrite (SCLK1, HIGH);
    
    //Serial.print((data & i) != 0 ? HIGH : LOW, BIN);
  }
  //Serial.println("");
}

void pushByte2(byte data2){
  pinMode (SDIO2, OUTPUT);
  
  delayMicroseconds(100); // tHOLD = 100us min.
  
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK2, LOW);
    digitalWrite (SDIO2, (data2 & i) != 0 ? HIGH : LOW);
    delayMicroseconds(100);
    digitalWrite (SCLK2, HIGH);
    
    //Serial.print((data & i) != 0 ? HIGH : LOW, BIN);
  }
  //Serial.println("");
}

byte readRegister1(byte address1) {
  address1 &= 0x7F; // MSB indicates read mode: 0
  
  pushByte1(address1);
  
  byte data1 = pullByte1();
  
  return data1;  
}

byte readRegister2(byte address2) {
  address2 &= 0x7F; // MSB indicates read mode: 0
  
  pushByte2(address2);
  
  byte data2 = pullByte2();
  
  return data2;  
}


void writeRegister1(byte address1, byte data1) {
  address1 |= 0x80; // MSB indicates write mode: 1
  
  pushByte1(address1);
  
  delayMicroseconds(100);
  
  pushByte1(data1);

delayMicroseconds(100); // tSWW, tSWR = 1
}

void writeRegister2(byte address2, byte data2) {
  address2 |= 0x80; // MSB indicates write mode: 1
  
  pushByte2(address2);
  
  delayMicroseconds(100);
  
  pushByte2(data2);

delayMicroseconds(100); // tSWW, tSWR = 1
}

void reset() {
  pinMode(SCLK1, OUTPUT);
  pinMode(SDIO1, INPUT);
  pinMode(NCS1, OUTPUT);
  pinMode(NRESET1, OUTPUT);
  pinMode(SCLK2, OUTPUT);
  pinMode(SDIO2, INPUT);
  pinMode(NCS2, OUTPUT);
  pinMode(NRESET2, OUTPUT);
    
  digitalWrite(SCLK1, LOW);
  digitalWrite(NCS1, LOW);
  digitalWrite(NRESET1, HIGH);
  digitalWrite(SCLK2, LOW);
  digitalWrite(NCS2, LOW);
  digitalWrite(NRESET2, HIGH);
  delayMicroseconds(100);
  
  // Initiate chip reset
  digitalWrite(NRESET1, LOW);
  pushByte1(0xfa);
  pushByte1(0x5a);
  digitalWrite(NRESET1, HIGH);

  digitalWrite(NRESET2, LOW);
  pushByte2(0xfa);
  pushByte2(0x5a);
  digitalWrite(NRESET2, HIGH);
  
  // Set resolution to 1000cpi
  //writeRegister1(0x0d, 0x01); // Sensor1
  //writeRegister2(0x0d, 0x01); // Sensor2
}

//Lese Daten aus Sensor1

void dumpDelta1() {
  
  if (dx1 > 128) {   //Information über die Richtung
    dx1 = dx1 - 256;
  }

  if (dy1 > 128) {
    dy1 = dy1 - 256;
  }

  xi1 += dx1; //Ganze Zahlen addieren, statt Gleitkommazahlen -> weniger Genauigkeitsverluste durch Rundung
  yi1 += dy1;
  
  x1 = xi1 *0.046355; //für 500cpi - 0.046mm pro Count, für 1000cpi - 0.02247mm pro Count (auf kariertem Blockpapier)
  y1 = yi1 *0.046355;
  
  

  if (/*shutter_up > 1 | shutter_low < 10*/pixelsum1 < 40) {  //Grenzwert abhängig von der Oberfläche
    y1 = 0; 
    x1 = 0;
  }

  
}

//Lese Daten aus Sensor2
void dumpDelta2() {

  if (dx2 > 128) {   //Information über die Richtung
    dx2 = dx2 - 256;
  }

  if (dy2 > 128) {
    dy2 = dy2 - 256;
  }
 
  xi2 += dx2;
  yi2 += dy2;
  
  x2 += xi2 *0.046355; //für 500cpi - 0.046mm pro Count, für 1000cpi - 0.02247mm pro Count (auf kariertem Blockpapier)
  y2 += yi2 *0.046355;
  
  

  if (/*shutter_up > 1 | shutter_low < 10*/pixelsum2 < 40) {  //Grenzwert abhängig von der Oberfläche
    y2 = 0; 
    x2 = 0;

  }


  
}  


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP 
////////////////////

void setup() {

  //Register zuordnen
  byte motion1 = readRegister1(REG_MOTION);
  int dx1 = readRegister1(REG_DELTA_X);
  int dy1 = readRegister1(REG_DELTA_Y);
  byte pixelsum1 = readRegister1(0x09);

  char motion2 = readRegister2(REG_MOTION); 
  int dx2 = readRegister2(REG_DELTA_X);
  int dy2 = readRegister2(REG_DELTA_Y);
  byte pixelsum2 = readRegister2(0x09);

  
  //Maussensoren initialisieren
  reset();
  
  // Starte serielle Datenverbindung  
  Serial.begin(9600);

  //Initialisierungsschritt, Ermittlung der mechanischen Zeitkonstanten theta/b
  Serial.println("Ermittle Zeitkonstante theta/b");
  
  // Datenpin des Funksenders an Pin 10 angeschlossen
  mySwitch.enableTransmit(10);
  //Einschalten der Steckdose
  mySwitch.switchOn("11001", "01000");
  delay(10000);
 
  if (pixelsum1 < 40 | pixelsum2 < 40){   //Wenn die Nullmarkierung erreicht ist, dann ausschalten
    delay(1000);
    mySwitch.switchOff("11001", "01000");
    delay(100); //Verzögerung der Funksteckdose
    // Winkel und Winkelgeschwindigkeit zum Ausschaltzeitpunkt
    float phi0 = phi;           
    double t = millis();   // Timer
    float phi_p = (phi - phi0)/t ;
    float phi_p0 = phi_p;

              
    //Im Stillstand Winkelgeschwindigkeit und -beschleunigung Null setzen
    if (dx1 == 0 & dy1 == 0 & dx2 == 0 & dy2 == 0){
      phi_p = 0;
      phi_pp = 0;
      
      tau = (phi - phi0)/phi_p0; // mechanische Zeitkonstante theta/d
  
    }
  }

}

void loop() {
  
   //Eingabgeaufforderung
  Serial.println("Gib den Sollwinkel ein...");
  
  // Warte auf Eingabe
  while (Serial.available() != -1) {
  }
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
  

  // Vergleiche Soll- und Ist-Wert  
  if (phi_soll - phi_ist >= Toleranz | phi_soll - phi_ist <= Toleranz) { //Toleranzwerte festlegen
    
    // Schalte Steckdose ein, falls Ist-Wert ungleich Soll-Wert
    mySwitch.switchOn("11001", "01000"); //Binärcode der Stellung der Dip-Schalter entsprechend     
  }
    else {
    
    // Schalte Steckdose aus 
    mySwitch.switchOff("11001", "01000");
    }

  //Schalte die Steckdose aus, wenn die verbleibende Zeit bis zum Erreichen des Sollwinkels der Auslaufzeit entspricht
  if (t_soll == t_aus){
    mySwitch.switchOff("11001", "01000");
  }

}
