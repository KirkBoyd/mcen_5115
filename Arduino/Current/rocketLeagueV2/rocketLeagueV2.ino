/* Got from roboticsbackend.com/raspberry-pi-arduino-serial-communication/
.//quick paste motor commands:
<MOT|254-254-254-254-1-1-1-1>
<MOT|254-254-254-254-1-1-1-1>
<MOT|254-254-254-254-1-1-1-1>
<MOT|254-254-254-254-1-1-1-1>
 */
// Retyped by Kirk Boyd
// Last modified Nov 16, 2021
//Diverged from original file "commsRedo.ino"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int motVals[12] = {0}; //integer array to store motor values received over serial
int motValsOld[12] = {0}; //stores old values to compare later
int theta = 0;
int oldtheta = 1;
String IMUstr = "<IMU|0>"; //String to send IMU data
String sendPacket; //packet to send to pi
double magX = -1000000, magY = -1000000;
sensors_event_t magEvent, orientationData; //BNO055 magnetic data
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 20;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Pins - Motor Drivers //
/* Front Left   (FL) Motor - WHITE */
#define mFL 1 //index number for FL motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_FL 39  //mega or teensy pin //SHOULD BE WHITE //VERIFIED
#define aIn2_FL 38  //mega or teensy pin //SHOULD BE WHITE //VERIFIED
#define pwm_FL 10 //6 //10   // speed for front left motor //SHOULD BE WHITE //VERIFIED

/* Front Right  (FR) Motor - YELLOW */
#define mFR 2 //index number for FR motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_FR 17 //40//41 //40  //mega or teensy pin //SHOULD BE YELLOW //VERIFIED
#define aIn2_FR 22 //41//40 //41  //mega or teensy pin //SHOULD BE YELLOW //VERIFIED
#define pwm_FR 13 //7 //11   //speed for front right motor //SHOULD BE YELLOW //VERIFIED

/* Back Left    (BL) Motor - BLUE   */
#define mBL 3 //index number for BL motor, may be used for logic
#define aIn1_BL 16 //15  //mega or teensy pin //SHOULD BE BLUE //VERIFIED
#define aIn2_BL 15 //16  //mega or teensy pin //SHOULD BE BLUE //VERIFIED
#define pwm_BL 12 //8 //12   //speed for back left motor //SHOULD BE BLUE //VERIFIED

/* Back Right   (BR) Motor - GREEN  */
#define mBR 4 //index number for BR motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_BR 40 //17  //mega or teensy pin //SHOULD BE GREEN //VERIFIED
#define aIn2_BR 41 //22  //mega or teensy pin //SHOULD BE GREEN //VERIFIED
#define pwm_BR 11 //13 //9 //13   //speed for back right motor //SHOULD BE GREEN //VERIFIED
int analogPins[4] = {10, 13, 12, 11};
int inA1Pins[4] = {39, 17, 16, 40};
int inA2Pins[4] = {38, 22, 15, 41};
void setup(){
//  /* Pin Mode IMU Data */
    pinMode(26,OUTPUT);
    pinMode(27,OUTPUT);
    pinMode(28,OUTPUT);
    pinMode(29,OUTPUT);
    pinMode(30,OUTPUT);
    pinMode(31,OUTPUT);
    pinMode(32,OUTPUT);
    pinMode(33,OUTPUT);
    pinMode(34,OUTPUT);

    /*Pin Mode Motors*/
    pinMode(mFL, OUTPUT);
    pinMode(aIn1_FL,OUTPUT);
    pinMode(aIn2_FL, OUTPUT);
    pinMode(pwm_FL,OUTPUT);
    
    pinMode(mFR, OUTPUT);
    pinMode(aIn1_FR,OUTPUT);
    pinMode(aIn2_FR, OUTPUT);
    pinMode(pwm_FR,OUTPUT);
    
    pinMode(mBL, OUTPUT);
    pinMode(aIn1_BL,OUTPUT);
    pinMode(aIn2_BL, OUTPUT);
    pinMode(pwm_BL,OUTPUT);
    
    pinMode(mBR, OUTPUT);
    pinMode(aIn1_BR,OUTPUT);
    pinMode(aIn2_BR, OUTPUT);
    pinMode(pwm_BR,OUTPUT);

    
  /* Initialise the sensor */
  Serial.begin(9600);
  delay(1000);
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //bno.onReceive(ReceiveEvent);
}
int loopCounter = 1;
int motStep[4];
const int numSteps = 20;
int motSteps[numSteps];
int scaledSpeeds[4] = {255};
int oldScaledSpeeds[4] = {255};
float percentage = 1;
void loop(){ 
  if(Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    subdivideStr(data);
    for(int i = 0; i<4; i++){
      if (motVals[i+4] ==1){ //Going Forawrd
        scaledSpeeds[i] = 250+motVals[i];
      }
      else{
        scaledSpeeds[i] = 260-motVals[i];
      }
    }
    for(int i = 0; i<4; i++){ // set ramp vals
      motSteps[i] = (scaledSpeeds[i]-oldScaledSpeeds[i])/numSteps;
      Serial.print(motSteps[i]);
      //motSteps[i] = (motVals[i]-motValsOld[i])/numSteps;
    }
    for(int j = 1; j<numSteps+1; j++){ // Ramp Motors
      for(int i = 0; i<4; i++){  //Set motors to intended values again
        //moveMotor(i,motVals[i],motVals[i+4]);
        int goalSpeed = oldScaledSpeeds[i] + j*motSteps[i];
        if (goalSpeed > 255){
          goalSpeed -= 255;
          //Serial.println(goalSpeed);
          digitalWrite(inA1Pins[i],true);
          digitalWrite(inA2Pins[i],false);
          analogWrite(analogPins[i],goalSpeed*percentage);
        }else{
          goalSpeed = 255 - goalSpeed;
          //Serial.println(goalSpeed);
          digitalWrite(inA1Pins[i],false);
          digitalWrite(inA2Pins[i],true);
          analogWrite(analogPins[i],goalSpeed*percentage);
        }
        delay(20);
      }
    }
    for(int i =0; i<4; i++){
        motValsOld[i] = motVals[i];
        oldScaledSpeeds[i] = scaledSpeeds[i];
    }
  }
 
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  theta = int(orientationData.orientation.x);
  if(theta != oldtheta){
    sendIMU(theta);
  }
  oldtheta = theta;
  loopCounter++;  
}
