/** LIBRARIES **/
//#include <Wire.h>  // Library which contains functions to have I2C Communication

/** CONSTANTS **/
// Pins //
#define stripeR 2 //teensy digital pin 2
#define stripeL 3 //
#define stripeMid 4 //
#define aIn1_f 39//teensy pin 39
#define aIn2_f 38//teensy pin 38
#define pwmA_f 14//teensy pin 14 // speed for front left motor
#define bIn1_f 40//teensy pin 40
#define bIn2_f 41//teensy pin 41
#define pwmB_f 37//teensy pin 37 //speed for front right motor
#define aIn1_b 15//teensy pin 15
#define aIn2_b 16//teensy pin 16
#define pwmA_b 36//teensy pin 36 //speed for back left motor
#define bIn1_b 17//teensy pin 17
#define bIn2_b 22//teensy pin 22
#define pwmB_b 33//teensy pin 33 //speed for back right motor
#define mFr 1
#define mFl 2
#define mRr 3
#define mRl 4
#define ledPin 13


// RFM69 SETUP //
#define FREQUENCY RF69_915MHZ
#define ENCRYPT false //set to "true" to encrypt signal
#define ENCRYPTKEY "youShouldKnowThisAlready" // "Use the same 16-byte key on all nodes"
#define USEACK false //set to "true" to request acknowledgements (ACKs) when packets recieved
RFM69 radio;

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
  pinMode(aIn1_f, OUTPUT);
  pinMode(aIn2_f, OUTPUT);
  pinMode(pwmA_f, OUTPUT);
  pinMode(bIn1_f, OUTPUT);
  pinMode(bIn2_f, OUTPUT);
  pinMode(pwmB_f, OUTPUT);
  pinMode(aIn1_b, OUTPUT);
  pinMode(aIn2_b, OUTPUT);
  pinMode(pwmA_b, OUTPUT);
  pinMode(bIn1_b, OUTPUT);
  pinMode(bIn2_b, OUTPUT);
  pinMode(pwmB_b, OUTPUT);
  pinMode(stripeL, INPUT);
  pinMode(stripeMid, INPUT);
  pinMode(stripeR, INPUT);
  Serial.begin(9600);
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
  printStripes();
//  allDirsTest();
        
  //while(digitalRead(stripeMid) == LOW){
          Serial.println("in whiel");
    //delay(1000);

    
  //}//end while(middle == 0)
  //stop();
}//end void loop
