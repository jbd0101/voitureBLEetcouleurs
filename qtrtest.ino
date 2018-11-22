#include <QTRSensors.h>
/* ble */

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define MODE_BLE_NAME               "voiture de jc !"  
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* end ble */

#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   13     // emitter is controlled by digital pin 2
//-- MOTEUR A --
int ENA=5; //Connecté à Arduino pin 5(sortie pwm)
int IN1=2; //Connecté à Arduino pin 2
int IN2=3; //Connecté à Arduino pin 3

//-- MOTEUR B --
int ENB=6; //Connecté à Arduino pin 6(Sortie pwm)
int IN3=4; //Connecté à Arduino pin 4
int IN4=7; //Connecté à Arduino pin 7
const int pinMODE = 0;
bool automatique = false;
bool BLEahead = false;
bool BLEleft = false;
bool BLEright = false;
bool forwardStatus = true;
bool stopBLE = false;
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A7 i.e. digital pins 14-19 in uno
 unsigned int sensors[6];
int balanceDuBlanc[] = {0,0,0,0,0,0};
int values[] = {0,0,0,0,0,0};
const int baseSpeed = 150;
boolean onBlack[] = {false,false,false,false,false,false};


/* fonction aide bluetooth */ 
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
/* fin fontions aides */
void forward_setting(bool status){
  if(status==false){
      // Direction du Moteur A
      digitalWrite(IN1,LOW); 
      digitalWrite(IN2,HIGH);
      
      // Direction du Moteur B
      // NB: en sens inverse du moteur A
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
  }else{
          // Direction du Moteur A
      digitalWrite(IN1,HIGH); 
      digitalWrite(IN2,LOW);
      
      // Direction du Moteur B
      // NB: en sens inverse du moteur A
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
  }
  forwardStatus = status;
}
void setup() {
    Serial.begin(9600);

  /* primo bluetooth */

      if ( !ble.begin(VERBOSE_MODE) )
      {
        error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
      }
      Serial.println( F("OK!") );
    
      if ( FACTORYRESET_ENABLE )
      {
        /* Perform a factory reset to make sure everything is in a known state */
        Serial.println(F("Performing a factory reset: "));
        if ( ! ble.factoryReset() ){
          error(F("Couldn't factory reset"));
        }
      }
      /* demarrage ble */
      ble.echo(false);
      ble.verbose(false);  // debug info is a little annoying after this point!
      /* parametrage ble */
        if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
      {
        // Change Mode LED Activity
        Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
        ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
        //changement de nom
        ble.sendCommandCheckOK("AT+GAPDEVNAME=" MODE_BLE_NAME);
    
      }
        ble.sendCommandCheckOK("AT+GAPDEVNAME=" MODE_BLE_NAME);
    
      // Set Bluefruit to DATA mode
      Serial.println( F("Switching to DATA mode!") );
      ble.setMode(BLUEFRUIT_MODE_DATA);
  
  // end ble 
  // end ble 
  // end ble 
  // end ble 
   int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

   // qtrrc.calibrate();
  delay(20);

  delay(500); // wait for 2s to position the bot before entering the main loop

  Serial.println("|--------- VOITURE -------|");
  Serial.println("|    calibr. terminée     |");
  Serial.println("* Placer la voiture sur une feuille blanche (1s )");
  delay(500);
  int tmp = 0;
  qtrrc.read(sensors);

  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    balanceDuBlanc[i] = sensors[i];
    if(balanceDuBlanc[i] == 0){
      balanceDuBlanc[i]++;
    }
  }
   pinMode(ENA,OUTPUT);//Configurer les broches comme sortie
 pinMode(ENB,OUTPUT);
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 digitalWrite(ENA,LOW);// Moteur A - Ne pas tourner (désactivation moteur)
 digitalWrite(ENB,LOW);// Moteur B - Ne pas tourner (désactivation moteur)

forward_setting(false);

}

void turnRight(){
  analogWrite(ENA,0);
  analogWrite(ENB,baseSpeed);
  //delay(10);
}
void turnLeft(){
  analogWrite(ENA,baseSpeed);
  analogWrite(ENB,0);
  //delay(10);
}
void straightAhead(){
  analogWrite(ENA,baseSpeed);
  analogWrite(ENB,baseSpeed);
  //delay(10);

}
void stop_motors(){
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void setBLEbutton(int buttnum,bool status){
  if(buttnum == 1){
  stopBLE = true;
  automatique = !automatique;
}else if(buttnum == 5){
  BLEahead = status;
}else if(buttnum == 6){
  forward_setting(status);
  BLEahead = status;
}else if(buttnum == 7){
  BLEleft=status;
}else if(buttnum == 8){
  BLEright = status;
}
}
void readBLE(){
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len != 0){
    if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (pressed) {
      setBLEbutton(buttnum,true);
    } else {
      if(buttnum != 1 ){
        setBLEbutton(buttnum,false);
      }
    }
  }
  } 
}
void loop() {

  if(stopBLE == false){
    readBLE();
  }
 if(automatique==true){
     qtrrc.read(sensors);
  
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
      values[i] = sensors[i]-balanceDuBlanc[i];
      if(values[i]>400){
        onBlack[i]= true;
      }else{
        onBlack[i]=false;
      }
      Serial.print(onBlack[i]);
      Serial.print("\t | \t ");
    }
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
      if(onBlack[i] == true){
        if(i==0 || i== 1){
          turnRight();
        }else if(i==2 || i == 3){
          straightAhead();
        }else if(i==4 || i == 5 ){
          turnLeft();
        }
      }
    }
    Serial.println();
 }else{

    if(BLEright == true){
      turnRight();
    }else if(BLEahead==true){
      straightAhead();
    }else if(BLEleft==true ){
      turnLeft();
    }else{
      stop_motors();
    }
 }

}
