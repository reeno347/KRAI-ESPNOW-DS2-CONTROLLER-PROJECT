/*---------------------------------------------------------------------------------------------------------*/
/*--------------------ARDUINO MEGA PARSING DATA FROM ESP32, MECANUM WHEEL WITH JOYSTICK--------------------*/
/*--------------------INCLUDING INTERNAL ENCODER FEEDBACK WITH PID CLOSED LOOP FEEDBACK--------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-24 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-24 TEAM---------------------------------*/
/*----------------------------------------------------V3.1-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 09:35:00, 16 JAN 25-----------------------------------*/

// Define DEBUG to enable debugging; comment it out to disable
// #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINT(...)    
  #define DEBUG_PRINTLN(...)  
  #define DEBUG_BEGIN(baud)   
#endif

#include "dataReadFunc.h"
#include "mecanumControl.h"
#include "shooting.h"


bool R2lastState = true;
bool L2lastState = true;
bool R1lastState = true;
bool L1lastState = true;

uint8_t relay1 = 46;
uint8_t relay2 = 48;
uint8_t relay3 = 50;
uint8_t relay4 = 52;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, HIGH);
  pinMode(relay3, OUTPUT);
  digitalWrite(relay3, LOW);
  pinMode(relay4, OUTPUT);
  digitalWrite(relay4, LOW);
  recvStart();
  PIDSetup();
  mecanumSetup();
  shootingSetup();
  
}

void loop(){
  checkData();
  delay(30);
  calcMecanum();
  relayCheck();
  shootingTask();
  // feederTask();
}

void relayCheck(){
    if (!recvData.stat[12] && R2lastState) { // R2
    digitalWrite(relay2, !digitalRead(relay2)); // Toggle Relay 2
    delay(100);
    //Mengaktifkan Relay1 Sesaat
    digitalWrite(relay1, LOW);
    delay(100);
    digitalWrite(relay1, HIGH);
    } 
    else if (!recvData.stat[12] && R2lastState) { // R2
    digitalWrite(relay2, !digitalRead(relay2)); // Toggle Relay 2

    }

  R2lastState = recvData.stat[12];

  if (!recvData.stat[2] && L2lastState) {  //L2
    digitalWrite(relay2, !digitalRead(relay2));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  L2lastState = recvData.stat[2]; 
  
  if (!recvData.stat[13] && R1lastState) { //R1
    digitalWrite(relay3, !digitalRead(relay3)); 
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  R1lastState = recvData.stat[13];

  if (!recvData.stat[3] && L1lastState) {  //L1
    digitalWrite(relay4, !digitalRead(relay4));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  }
  L1lastState = recvData.stat[3]; 
}


  // if (!recvData.stat[12] && R2lastState) { //R2
  //   digitalWrite(relay2, !digitalRead(relay2));
  //   delay(100);
  //   digitalWrite(relay1, !digitalRead(relay1)); 
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
  // }