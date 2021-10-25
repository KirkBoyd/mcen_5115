/** LIBRARIES **/
//#include <Wire.h>  // Library which contains functions to have I2C Communication
#include <RFM69.h>
#include <SPI.h>

/** CONSTANTS **/
// Pins - SPI //
#define MISOpin 50 //mega pin for SPI interface
#define CIPOpin 50 //mega pin for SPI interface (PC name)
#define MOSIpin 51 //mega pin for SPI interface
#define COPIpin 51 //mega pin for SPI interface (PC name)
#define SCKpin  52 //mega pin for SPI interface
#define SSpin 53 //mega pin for SPI interface

// Pins - Sensors //
#define stripeR 2 //teensy digital pin 2
#define stripeL 3 //
#define stripeMid 4 //

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




#define ledPin 13


// RFM69 SETUP //
#define NETWORKID 10 //must match all nodes
#define MYNODEID 2  //our node
#define TONODEID 1 //Reamon's output node
#define FREQUENCY RF69_915MHZ //this should not change
#define ENCRYPT false //set to "true" to encrypt signal
#define ENCRYPTKEY "youShouldKnowThisAlready" // "Use the same 16-byte key on all nodes"
#define USEACK false //set to "true" to request acknowledgements (ACKs) when packets recieved

RFM69 radio; //create the radio object

char radData[18];

char datarino;
// PIXY2 SETUP //
//#define SLAVE_ADDRESS 0x40 // Define the I2C address to Communicate to Uno
//
//byte response[2]; // this data is sent to PI
//volatile short LDR_value; // Global Declaration
//const int LDR_pin=A9; //pin to which LDR is connected A0 is analog A0 pin 

String input;


int speedPin = 0;
int ctrl_1 = 0;
int ctrl_2 = 0;
int speedRatio = 0;
boolean on = false;

void setup() {
  // Initialize Pins //
  pinMode(aIn1_FL, OUTPUT);
  pinMode(aIn2_FL, OUTPUT);
  pinMode(pwm_FL, OUTPUT);
  pinMode(aIn1_FR, OUTPUT);
  pinMode(aIn2_FR, OUTPUT);
  pinMode(pwm_FR, OUTPUT);
  pinMode(aIn1_BL, OUTPUT);
  pinMode(aIn2_BL, OUTPUT);
  pinMode(pwm_BL, OUTPUT);
  pinMode(aIn1_BR, OUTPUT);
  pinMode(aIn2_BR, OUTPUT);
  pinMode(pwm_BR, OUTPUT);
  pinMode(stripeL, INPUT);
  pinMode(stripeMid, INPUT);
  pinMode(stripeR, INPUT);

  // Initialize Serial Connection //
  Serial.begin(38400); // can change this later if need be
  Serial.println("Sandcrawler Initiated");
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");

  // Initialize Radio Module RFM69HCW //
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();
  if(ENCRYPT){radio.encrypt(ENCRYPTKEY);} // likely will not use

  // Pixy Setup //
//  Wire.begin(SLAVE_ADDRESS); // this will begin I2C Connection with 0x40 address
//  Wire.onRequest(sendData); // sendData is funtion called when Pi requests data
//  pinMode(LDR_pin,INPUT);
//  Wire.begin(0x8);                // join i2c bus with address #8
//  Wire.onReceive(receiveEvent); // register event
//  pinMode(ledPin, OUTPUT);
//  digitalWrite(ledPin, LOW); // turn it off
//  Serial.println("Awaiting Serial Input...");
}

void loop() {

  
/* Stuff below this was used in the past for debugging / lab / testing */  
  getRadData();
//  lab6radioCheck();
//    lab6radioCheckExtraData();  
//  printStripes();
//  allDirsTest();
//  north();
//  digitalWrite(aIn1_BR, HIGH);
//  digitalWrite(aIn2_BR, LOW);
//  analogWrite(pwm_BR, 255*0.25);
//  delay(1000);
//  stop();
//  delay(1000);
//  while(digitalRead(stripeMid) == LOW){
//     Serial.println("in whiel");
//    delay(1000);

    
  //}//end while(middle == 0)
  //stop();
}//end void loop
