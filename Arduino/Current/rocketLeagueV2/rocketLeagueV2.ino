/* Got from roboticsbackend.com/raspberry-pi-arduino-serial-communication/
 
 */
// Retyped by Kirk Boyd
// Last modified Nov 16, 2021
//Diverged from original file "commsRedo.ino"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int motVals[12] = {0}; //integer array to store motor values received over serial
String IMUstr = "<IMU|0>"; //String to send IMU data
double magX = -1000000, magY = -1000000;
sensors_event_t magEvent; //BNO055 magnetic data
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Pins - Motor Drivers //
/* Front Left   (FL) Motor - WHITE */
#define mFL 1 //index number for FL motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_FL 39  //mega or teensy pin //SHOULD BE WHITE
#define aIn2_FL 38  //mega or teensy pin //SHOULD BE WHITE
#define pwm_FL 10   // speed for front left motor //SHOULD BE WHITE

/* Front Right  (FR) Motor - YELLOW */
#define mFR 2 //index number for FR motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_FR 40  //mega or teensy pin //SHOULD BE YELLOW
#define aIn2_FR 41  //mega or teensy pin //SHOULD BE YELLOW
#define pwm_FR 11   //speed for front right motor //SHOULD BE YELLOW

/* Back Left    (BL) Motor - BLUE   */
#define mBL 3 //index number for BL motor, may be used for logic
#define aIn1_BL 15  //mega or teensy pin //SHOULD BE BLUE
#define aIn2_BL 16  //mega or teensy pin //SHOULD BE BLUE
#define pwm_BL 12   //speed for back left motor //SHOULD BE BLUE

/* Back Right   (BR) Motor - GREEN  */
#define mBR 4 //index number for BR motor, may be used for logic
//had to flip the two below, not sure why
#define aIn1_BR 17  //mega or teensy pin //SHOULD BE GREEN
#define aIn2_BR 22  //mega or teensy pin //SHOULD BE GREEN
#define pwm_BR 13   //speed for back right motor //SHOULD BE GREEN

void setup(){
  /* Initialise the sensor */
  Serial.begin(115200);
  delay(1000);
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //bno.onReceive(ReceiveEvent);
}

void loop(){
//  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//  magX = magEvent.magnetic.x;
//  magY = magEvent.magnetic.y;
  
//  int yaw = atan2(magY, magX) * 180/3.14159+180;
//  IMUstr = String(yaw);
  

  
  if(Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    subdivideStr(data);
    for(int i = 0; i<4; i++){
      moveMotor(i,motVals[i],motVals[i+4],motVals[i+8]);
    }
  }
//  Serial.println(IMUstr);
//  if(Serial.availableForWrite() == 0){
//      Serial.println(IMUstr);
//      //Serial.println("<IMU|" + IMUstr + ">");
//  }
  
}
