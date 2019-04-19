/*
=====GUIDE STUFF=====
PHONE NUMERICAL KEYPAD:
1 | 2 | 3 | A
4 | 5 | 6 | B
7 | 8 | 9 | C
* | 0 | # | D

ROBOT CONTROL v1.0
1: Distance up
2: Forward
3: Distance down
4: Turn left
5: Backward
6: Turn right

(not yet done)
7: Tilt up
8: Song A
9: Song B
*: Tilt down

*/

//=====IMPORT LIBS=====
//HUD:
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//STEPPERS:
#include <AccelStepper.h>

//SONGS
#include "pitches.h"

//=====VARIABLES=====
bool FlyByWire; //if bot is in FBY or not
float StepSpeed = 512.0f;
; //the speed of the steppers
const float DFdistance = 5.0f; //inches per turn
float distance; //variable inches per turn

//=====PINOUTS=====
//lcd for HUD
LiquidCrystal_I2C lcd(0x3F,20,4);

//steppers
AccelStepper step1(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
AccelStepper step2(AccelStepper::FULL4WIRE, 2, 4, 3, 5);

//mt8870 dtmf 
byte Q1 = 18;
byte Q2 = 17;
byte Q3 = 16;
byte Q4 = 15;
byte StQ = 14;

//=====VOIDS=====
//Go with Inches
void drive2(int S1, int S2, float distIn, float diameter){
  Serial.println("distIn: ");
  Serial.println(distIn);
  Serial.println("diameter: ");
  Serial.println(diameter);
  Serial.println("denominator: ");
  Serial.println((diameter * 3.1415f));
  Serial.println("-------");
  //STEPPER 1
  if(S1 == 1){
    //drive STP1 forward
   Serial.println((int)round((distIn / (diameter * 3.1415f)) * 2048));
   step1.moveTo(step1.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * 2048)));
  }
  else if(S1 == 0){
    //neutral
  }
  else if(S1 == -1){
    //drive STP1 backward
    Serial.println("s1 backward: ");
    Serial.println(step1.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * -2048)));
    step1.moveTo(step1.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * -2048)));
  }
  //STEPPER 2
  if(S2 == 1){
    //drive STP2 forward
    step2.moveTo(step2.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * -2048)));
  }
  else if(S2 == 0){
    //neutral
  }
  else if(S2 == -1){
    //drive STP2 backward
    Serial.println("s2 backward: ");
    Serial.println(step2.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * 2048)));
    step2.moveTo(step2.currentPosition() + ((int)round((distIn / (diameter * 3.1415f)) * 2048)));
  }
}

//Rotate with angle
void rotate2(float angle, float diameter, float wheelDistance){
  //find decimal value of angle
  float calcAngle;
  calcAngle = angle / 360.0f;
  
  //find distance required to execute turn
  float calcDistance;
  calcDistance = (calcAngle * 3.1415f * wheelDistance) / 2;
  
  //find motor rotations required to execute distance
  step1.moveTo((calcDistance / (diameter * 3.1415f)) * 2048);
  step2.moveTo((calcDistance / (diameter * 3.1415f)) * -2048);
}

//=====INITIAL CODE=====
void setup() {
  //==DTMF CONTROL SETUP==
  pinMode(Q1, INPUT);
  pinMode(Q2, INPUT);
  pinMode(Q3, INPUT);
  pinMode(Q4, INPUT);
  pinMode(StQ, INPUT);
  
  //==LCD (HUD) Setup==
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2,1);
  lcd.print("CHRIS ON A STICK");
  lcd.setCursor(1,2);
  lcd.print("TELEPRESENCE ROBOT");
  //delay(2000);
  
  //==INITIALIZE MOTORS==
    step1.setMaxSpeed(StepSpeed);
    step1.setAcceleration(200.0);
    
    step2.setMaxSpeed(StepSpeed);
    step2.setAcceleration(200.0);
    
  //==PREPARE VARIABLES==
  distance = DFdistance;
  
  //==DEBUG (SERIAL)==
  Serial.begin(9600);
  /*
  long int x = 100000;
  step1.moveTo(x);
  step2.moveTo(-1 * x);
  */
}
//

void loop() {
  uint8_t number;
  
  //===DTMF DECODING AND EXECUTION===
  bool signal ;  
  signal = digitalRead(StQ);
  if(signal == HIGH)  /* If new pin pressed */
   {
    delay(250);
    number = ( 0x00 | (digitalRead(Q1)<<0) | (digitalRead(Q2)<<1) | (digitalRead(Q3)<<2) | (digitalRead(Q4)<<3) );
      switch (number)
      {
        case 0x01:
        Serial.println("Pin Pressed : 1");
        //increase distance
        distance = distance + 0.5f;
        break;
        
        case 0x02:
        Serial.println("Pin Pressed : 2");

        //go forward
        drive2(1,1,distance,8);
        break;
        
        case 0x03:
        Serial.println("Pin Pressed : 3");
        //decrease distance
        distance = distance - 0.5f;
        break;
        
        case 0x04:
        Serial.println("Pin Pressed : 4");
        //turn left
        break;
        
        case 0x05:
        Serial.println("Pin Pressed : 5");
        //backward
        drive2(-1,-1,distance,8);
        break;
        
        case 0x06:
        Serial.println("Pin Pressed : 6");
        //turn right
        break;
        
        case 0x07:
        Serial.println("Pin Pressed : 7");
        break;
        
        case 0x08:
        Serial.println("Pin Pressed : 8");
        break;
        
        case 0x09:
        Serial.println("Pin Pressed : 9");
        break;
        
        case 0x0A:
        Serial.println("Pin Pressed : 0");
        break;
        
        case 0x0B:
        Serial.println("Pin Pressed : *");
        break;
        
        case 0x0C:
        Serial.println("Pin Pressed : #");
        break;   
      }
  }
    step1.run();
    step2.run();
}




/* LEGACY (4 MOTOR) CODE:
===============================================
===============================================
void drive4(byte S1, byte S2, byte S3, byte S4){
  //STEPPER 1
  if(S1 == 1){
    //drive STP1 forward
  }
  else if(S1 == 0){
    //neutral
  }
  else if(S1 == -1){
    //drive STP1 backward
  }
  
  //===========================
  //STEPPER 2
  if(S2 == 1){
  //drive STP2 forward
  }
  else if(S2 == 0){
  //neutral
  }
  else if(S2 == -1){
  //drive STP2 backward
  }
  
  //===========================
  //STEPPER 3
  if(S3 == 1){
  //drive STP3 forward
  }
  else if(S3 == 0){
  //neutral
  }
  else if(S3 == -1){
  //drive STP3 backward
  }
  
  //=========================== 
  //STEPPER 4
  if(S4 == 1){
  //drive STP4 forward
  }
  else if(S4 == 0){
  //neutral
  }
  else if(S4 == -1){
  //drive STP4 backward
  }
  
  //=========================== 
}
//=COS1 LEGACY=
//AccelStepper step3(AccelStepper::FULL4WIRE, 50, 52, 48, 46);
//AccelStepper step4(AccelStepper::FULL4WIRE, 38, 34, 36, 32);
*/
