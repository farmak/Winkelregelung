#include <RCSwitch.h>  // Bilbiothek für Funksender und Funksteckdose einbinden
#define SCLK1 52
#define SDIO1 50  
#define NRESET1 48
#define NCS1 46 
#define SCLK2 22
#define SDIO2 24 
#define NRESET2 26
#define NCS2 28 
#define REG_PRODUCT_ID 0x00
#define REG_REVISION_ID 0x01
#define REG_MOTION 0x02
#define REG_DELTA_X 0x03
#define REG_DELTA_Y 0x04
#define REG_SQUAL 0x05
#define REG_PIXEL_SUM 0x09

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
float U = 2684.8; //mm 
float phi;
float phi_p;
float phi_pp;
float phi_p_1;
float phi_0;
float phi_1;
float phi_soll = 0.000001; // Variable für den Sollwinkel
float Toleranz = 0.15;


byte motion1; 
int dx1;
int dy1; 
byte pixelsum1; 

byte motion2;
int dx2 ;
int dy2;  
byte pixelsum2;


//float a1;
//float s1; 
//float phi1;
//
//float a2;
//float s2; 
//float phi2; 

float tau;
double t;
float t_aus;
float t_soll;
  
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
   
void Read() {
  
  byte motion1 = readRegister1(REG_MOTION); // Freezes DX and DY until they are read or MOTION is read again.
  int dx1 = readRegister1(REG_DELTA_X);
  int dy1 = readRegister1(REG_DELTA_Y);
  byte pixelsum1 = readRegister1(REG_PIXEL_SUM);

  byte motion2 = readRegister2(REG_MOTION); // Freezes DX and DY until they are read or MOTION is read again.
  int dx2 = readRegister2(REG_DELTA_X);
  int dy2 = readRegister2(REG_DELTA_Y);
  byte pixelsum2 = readRegister2(REG_PIXEL_SUM);
}



//Lese Daten aus Sensoren
void dumpDelta() {

  byte motion1 = readRegister1(REG_MOTION); // Freezes DX and DY until they are read or MOTION is read again.
  int dx1 = readRegister1(REG_DELTA_X);
  int dy1 = readRegister1(REG_DELTA_Y);
  byte pixelsum1 = readRegister1(REG_PIXEL_SUM);

  byte motion2 = readRegister2(REG_MOTION); // Freezes DX and DY until they are read or MOTION is read again.
  int dx2 = readRegister2(REG_DELTA_X);
  int dy2 = readRegister2(REG_DELTA_Y);
  byte pixelsum2 = readRegister2(REG_PIXEL_SUM);
  
  unsigned int t_1 = millis();
  unsigned int t_2 = millis()

  
  if (dx1 > 128) {   //Information über die Richtung
    dx1 = dx1 - 256;
  }

  if (dy1 > 128) {
    dy1 = dy1 - 256;
  }

  if (dx2 > 128) {   
    dx2 = dx2 - 256;
  }

  if (dy2 > 128) {
    dy2 = dy2 - 256;
  }
 
  xi1 += dx1; //Ganze Zahlen addieren, statt Gleitkommazahlen -> weniger Genauigkeitsverluste durch Rundung
  yi1 += dy1;

  xi2 += dx2;
  yi2 += dy2;
  
  x1 = xi1 *0.1; //0.046355; //für 500cpi - 0.046mm pro Count, für 1000cpi - 0.02247mm pro Count (auf kariertem Blockpapier)
  y1 = yi1 *0.1; //0.046355;

  x2 = xi2 *0.1; //0.046355; 
  y2 = yi2 *0.1; //0.046355;
  

  if (/*shutter_up > 1 | shutter_low < 10*/pixelsum1 > 140) {  //Grenzwert abhängig von der Oberfläche
    x1 = 0; 
    y1 = 0;

    x2 = 0;
    y2 = 0;
  }

  //float a1 = atan(dy1/dx1);
  //float s1 = y1*sin(a1); 
  float phi1 = -(360*y1)/U ; //(360*y1*(dy1/dx1))/(U*sqrt(((dy1/dx1)^2) + 1)); 
  
  //float a2 = atan(dy2/dx2);
  //float s2 = y2*sin(a2); 
  float phi2 = -(360*y2)/U; //(360*y2*(dy2/dx2))/(U*sqrt(((dy2/dx2)^2) + 1)); 

// Der Wert des Sensors mit dem größeren Winkel, wird als Ist-Wert genommen 
 if (phi1 > phi2) {
  phi = phi1;
 }
 else {
  phi = phi2;
 }
phi0 = phi;

phi
//double t_int = t_1 - t_2;

//phi_p = (phi - phi0)/t_int; 

//
  
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP 
////////////////////

void setup() {
  
  
  //Starte serielle Datenverbindung  
  Serial.begin(9600);
  
  //Maussensoren initialisieren
  reset();

  //Initialisierungsschritt, Ermittlung der mechanischen Zeitkonstanten theta/b
  Serial.println("Ermittle Zeitkonstante theta/b");
  
  //Datenpin des Funksenders an Pin 10 angeschlossen
  mySwitch.enableTransmit(10);
  
  //Einschalten der Steckdose
  mySwitch.switchOn("11111", "00010");
  delay(2000); // Verzögerung, um auf konstante Geschwindigkeit zu kommen
  
  byte motion1 = readRegister1(REG_MOTION);
  int dx1 = readRegister1(REG_DELTA_X);
  int dy1 = readRegister1(REG_DELTA_Y);
  byte pixelsum1 = readRegister1(0x09);

  byte motion2 = readRegister2(REG_MOTION); 
  int dx2 = readRegister2(REG_DELTA_X);
  int dy2 = readRegister2(REG_DELTA_Y);
  byte pixelsum2 = readRegister2(0x09); //Bewegungsinformationen

  while ((motion1 != 0) ) { // Während die Drehbühne sich bewegt, 
    
    byte motion1 = readRegister1(REG_MOTION);
    int dx1 = readRegister1(REG_DELTA_X);
    int dy1 = readRegister1(REG_DELTA_Y);
    byte pixelsum1 = readRegister1(0x09);
    
    byte motion2 = readRegister2(REG_MOTION); 
    int dx2 = readRegister2(REG_DELTA_X);
    int dy2 = readRegister2(REG_DELTA_Y);
    byte pixelsum2 = readRegister2(0x09);
  
    dumpDelta();                                         // Sensoren auslesen, um aktuellen Winkel bestimmen zu können
    
    //Wenn die Nullmarkierung erreicht ist, ausschalten
    if(pixelsum1 > 140){   
      
      mySwitch.switchOff("11111", "00010");
      dumpDelta();
      float phi_0 = phi;
      unsigned long t_0 = millis();  // Timer
      delay(50); // Verzögerung, bis die Funksteckdose schaltet
      dumpDelta();
      float phi_1 = phi;
      unsigned long t_1 = millis();
      unsigned long t_2 = t_1 - t_0;
      float phi_p_1 = 1000* (phi_1 - phi_0)/t_2 ;
      Serial.print("Phi_0: "); Serial.print(phi_0); Serial.print("  "); Serial.print("Phi_1: ");Serial.print(phi_1); Serial.print(" ");Serial.print("t_2: ");Serial.print(t_2);
      Serial.print("  "); Serial.print("Phi_p_1: "); Serial.print(phi_p_1);
      while ((motion1 != 0)){
         dumpDelta();
         byte motion1 = readRegister1(REG_MOTION);
         int dx1 = readRegister1(REG_DELTA_X);
         int dy1 = readRegister1(REG_DELTA_Y);
         byte pixelsum1 = readRegister1(0x09);

         byte motion2 = readRegister2(REG_MOTION); 
         int dx2 = readRegister2(REG_DELTA_X);
         int dy2 = readRegister2(REG_DELTA_Y);
         byte pixelsum2 = readRegister2(0x09);
         Serial.println("IN THE LOOP");
         if (motion1 == 0){
          Serial.println("break condition");
          break;
         }

      }
    dumpDelta();    
    tau = (phi - phi_1)/phi_p_1; // mechanische Zeitkonstante theta/d
    t_aus = 3*tau;
    Serial.print("Phi_0: "); Serial.println(phi_0);
    Serial.print("Phi_1: "); Serial.println(phi_1, DEC);
    Serial.print("Phi_p1: "); Serial.println(phi_p_1, DEC);
    Serial.print("tau: "); Serial.println(tau, DEC);

         
    }
    
      
    if (motion1 == 0){
      Serial.println("break condition no.2");
      break;         
    }
  
 }  
                
    //Im Stillstand Winkelgeschwindigkeit und -beschleunigung Null setzen
    //if (dx1 == 0 & dy1 == 0 & dx2 == 0 & dy2 == 0){
    //  phi_p = 0;
      //phi_pp = 0;
    //} 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LOOP
/////////////////////////

void loop() {

  
     //Eingabgeaufforderung
  Serial.println("Gib den Sollwinkel ein...");
  
  // Warte auf Eingabe
  while (phi_soll == 0.000001) {
    while (Serial.available() == 0){
    }
  phi_soll = Serial.parseFloat();  
  }
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
  Serial.print("Istwert");
  Serial.println(phi);
  
  // Ist-Wert des Winkels abrufen
  dumpDelta();
  
  
  // Vergleiche Soll- und Ist-Wert  
  if (phi_soll - phi > Toleranz | phi_soll - phi < Toleranz*(-1)) { //Toleranzwerte festlegen
    Serial.println("Sollwinkel wird eingesgtellt");
    // Schalte Steckdose ein, falls Ist-Wert ungleich Soll-Wert
    mySwitch.switchOn("11111", "00010"); //Binärcode der Stellung der Dip-Schalter entsprechend  
       
  }
  
  byte motion1 = readRegister1(REG_MOTION);
  byte motion2 = readRegister2(REG_MOTION);
  
  while (motion1 != 0 & motion2 != 0) { // Während die Drehbühne sich bewegt, 
    
    byte motion1 = readRegister1(REG_MOTION);
    byte motion2 = readRegister2(REG_MOTION);   //Überprüfe, ob die Drehbühne sich noch bewegt
    
    //Schalte die Steckdose aus, wenn die verbleibende Zeit bis zum Erreichen des Sollwinkels der Auslaufzeit entspricht
    dumpDelta();
    float phi0 = phi;
    unsigned long t_0 = millis();
    dumpDelta();
    unsigned long t_1 = millis();
    unsigned long t_2 = t_1 - t_0;
    phi_p = 1000*((phi - phi0)/t_2);
    dumpDelta();
    t_soll = (phi_soll - phi)/phi_p;

    Serial.print("t_soll: "); Serial.print(t_soll); Serial.print("   "); Serial.print("t_aus. "); Serial.println(t_aus);
    
    if (t_soll == t_aus){
    mySwitch.switchOff("11111", "00010");

    }  
  }
   
  
}
