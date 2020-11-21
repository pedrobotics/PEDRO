/*
 Hello! Welcome to the source code of PEDRO the Quadruped Robot. The bulk of the interesting parts
 are located in the legHandler file. PEDRO has one legHandler, which contains four leg objects. You
 can position a leg in world space relative to the body, or leg space relative to itself. The legHandler
 update() funtion is where all of the walking logic is performed, and where you can add additional operating
 modes if you wish!
  Adding a mode can be achieved by adding its name to the enum Mode list in legHandler.h,
 adding a case to the main mode switch statement, and finally adding some way of entering that mode into
 the "modeChange" case. A convenient method is the "LEFT" direction on the left stick while in the mode
 change menu.
 */

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "Leg.h"
#include "vec3d.h"
#include "LegHandler.h"

LegHandler legHandler;

RF24 radio(7, 8);

byte address[6] = "00001";

struct payload{ //this struct is what's sent from the remote to the robot

  float axes[6];
  byte buttons = 0;
  payload(){}
  payload(float* axesIn, bool* buts){
    for(int i = 0; i < 6; i++) axes[i] = axesIn[i];
    if(buts[0]) buttons |= B00000001;
    if(buts[1]) buttons |= B00000010;
  }
};

void bodySetup();
void transmitterControl();
void serialCommunication();

long loopStart;


void setup() {

  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(1, address);
  
  bodySetup();
  
  loopStart = millis();
  radio.startListening();
}

long pt = millis();
int updates = 0;

long servoTimer = millis();
long serialTimer = millis();



/////     MAIN LOOP     /////

void loop() {

  transmitterControl(); 
  legHandler.update();
  serialCommunication();

}
/////     END MAIN LOOP     /////

void serialCommunication(){
  if(Serial.available()){
    switch(Serial.read()){
      case 's':
      for(byte i = 0; i < 5; i++){
        char letter = address[i];
        Serial.print(letter);
      } Serial.println();
      break;
    }
  }
}

void transmitterControl(){ //receive controller input

  static long timeAtLastPayload{};
  static bool connected{};
  
  if(radio.available()){

    timeAtLastPayload = millis();
    connected = true;
    payload data;
    radio.read(&data, sizeof(payload));

    legHandler.input(vec3d(data.axes[1], -data.axes[0], data.axes[4]), vec3d(-data.axes[3], -data.axes[4], data.axes[3]));
    legHandler.buttonInput(data.buttons);
  }

  else if(millis() - timeAtLastPayload > 250 && connected == true){ //if no data recieved in 250 ms, set inputs to 0
    connected = false;
    legHandler.input(vec3d::zero, vec3d::zero);
    legHandler.buttonInput(0);
  }
}

int trimAxis = 0;

void bodySetup(){ //retrieve robot info from EEPROM, set trims, attach servos
  legHandler.servoTrims.load();
  
  for(byte i = 0; i < 4; i++){
    for(byte j = 0; j < 3; j++){
      legHandler.leg[i].servoTrim.n[j] = legHandler.servoTrims.values[i][j];
    }
  }
  //PROTOTYPE
  legHandler.leg[0].attachServos(5, 6, 9);
  legHandler.leg[0].setLegOrigin(vec3d(0.03889, -0.03889, 0), Quaternion(vec3d::Z, -45));
  
  legHandler.leg[1].attachServos(2, 3, 4);
  legHandler.leg[1].setLegOrigin(vec3d(0.03889, 0.03889, 0), Quaternion(vec3d::Z, 45));

  legHandler.leg[2].attachServos(A3, A4, A5);
  legHandler.leg[2].setLegOrigin(vec3d(-0.03889, 0.03889, 0), Quaternion(vec3d::Z, 135));

  legHandler.leg[3].attachServos(A0, A1, A2);
  legHandler.leg[3].setLegOrigin(vec3d(-0.03889, -0.03889, 0), Quaternion(vec3d::Z, 225));
  
  legHandler.homeLegs();
}
