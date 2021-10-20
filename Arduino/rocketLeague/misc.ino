
 void serialMotorLab4(){
  if(Serial.available()){
    input = Serial.readStringUntil('\n');
    //delay(1000);
    Serial.print("Recieved Input: ");
    Serial.println(input);
    if(input == '1'){
      Serial.println("Moving Motor...");
      moveMotor(1, HIGH, 0.25);
      moveMotor(2, HIGH, 0.25);
      moveMotor(3, HIGH, 0.25);
      moveMotor(4, HIGH, 0.25);
      //delay(1000);
    }
    else if(input == '2'){
      Serial.println("Stopping Motors...");
      moveMotor(1, HIGH, 0);
      moveMotor(2, HIGH, 0);
      moveMotor(3, HIGH, 0);
      moveMotor(4, HIGH, 0); 
      //delay(1000);
    }
    else{
      Serial.println("Not the right input. I shall do nothing. >:)");
      //delay(1000);
    }
  }
 }


void printStripes(){
  Serial.print("Left: ");
  Serial.print(digitalRead(stripeL));
  Serial.print("\t Mid: ");
  Serial.print(digitalRead(stripeMid));
  Serial.print("\t Right:");
  Serial.println(digitalRead(stripeR));
}
void stripeLogic(){
    if(digitalRead(stripeR) == LOW){ 
      east();
      Serial.println("tryna go east");
    }
    else if(digitalRead(stripeL) == LOW){ 
      west();
      Serial.println("tyna go weast"); 
    }
    else{ 
      north(); 
      Serial.println("tyna go north");
    }
}
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
