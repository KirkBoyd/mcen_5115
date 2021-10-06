//#include <Wire.h>  // Library which contains functions to have I2C Communication
//#define SLAVE_ADDRESS 0x40 // Define the I2C address to Communicate to Uno
//
//byte response[2]; // this data is sent to PI
//volatile short LDR_value; // Global Declaration
//const int LDR_pin=A9; //pin to which LDR is connected A0 is analog A0 pin 

String input;

#define line1pin 2//teensy digital pin 2
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
  Serial.begin(9600);
  //Wire.begin(SLAVE_ADDRESS); // this will begin I2C Connection with 0x40 address
  //Wire.onRequest(sendData); // sendData is funtion called when Pi requests data
//  pinMode(LDR_pin,INPUT);
  //Wire.begin(0x8);                // join i2c bus with address #8
  //Wire.onReceive(receiveEvent); // register event
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // turn it off
  delay(2000);
  Serial.println("Awaiting Serial Input...");
}

void loop() {
// Serial.println(digitalRead(line1pin));
// digitalWrite(ledPin,HIGH);
// delay(250);
// digitalWrite(ledPin,LOW);
// delay(1000);
  while(middle == 0){
    if(rightStripe == 0){
      right();
    }
    else if(leftStripe == 0){
      left();
    }
    else{
      north();
    }//end else
  }//end while(middle == 0)
}//end void loop

//void sendData(){
//  LDR_value=analogRead(LDR_pin);
//  // Arduino returns 10bit data but we need to convert it to 8bit 
//  LDR_value=map(LDR_value,0,1023,0,255);
//  response[0]=(byte)LDR_value;
//  Wire.write(response,2); // return data to PI
//}
//
//void receiveEvent(int howMany) {
//  while (Wire.available()) { // loop through all but the last
//    if (Wire.read()){
//     on = !on; // receive byte as a character
//    //digitalWrite(ledPin, c);
//    }
//  }
//}
