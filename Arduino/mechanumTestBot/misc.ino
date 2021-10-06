 void serialMotorLab4(){
  if(Serial.available()){
    input = Serial.readStringUntil('\n');
    //delay(1000);
    Serial.print("Recieved Input: ");
    Serial.println(input);
    if(input == '1'){
      Serial.println("Moving Motor...");
      moveMotor(1, HIGH, 0.75);
      moveMotor(2, HIGH, 0.75);
      moveMotor(3, HIGH, 0.75);
      moveMotor(4, HIGH, 0.75);
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
