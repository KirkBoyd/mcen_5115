void moveMotor(int motorNum, bool dir, float speedRatio){ // motorNum is which motor; dir is fwd or back; speed ratio is a number between 0 and 1 where 0 is stopped and 1 is full speed
  if(motorNum == 1){
    digitalWrite(aIn1_f, dir);
    digitalWrite(aIn2_f, !dir);
    analogWrite(pwmA_f, speedRatio*255);
  }
  else if(motorNum == 2){
    digitalWrite(bIn1_f, dir);
    digitalWrite(bIn2_f, !dir);
    analogWrite(pwmB_f, speedRatio*255);  
  }
  else if(motorNum == 3){
    digitalWrite(aIn1_b, dir);
    digitalWrite(aIn2_b, !dir);
    analogWrite(pwmA_b, speedRatio*255);
  }
  else if(motorNum == 4){
    digitalWrite(bIn1_b, dir);
    digitalWrite(bIn2_b, !dir);
    analogWrite(pwmB_b, speedRatio*255);
  }
}

void stop(){
  moveMotor(1, HIGH, 0);
  moveMotor(2, HIGH, 0);
  moveMotor(3, HIGH, 0);
  moveMotor(4, HIGH, 0);
}
void north(){
  moveMotor(1, HIGH, 0.75);
  moveMotor(2, HIGH, 0.75);
  moveMotor(3, HIGH, 0.75);
  moveMotor(4, HIGH, 0.75);
}
void south(){
  moveMotor(1, LOW, 0.75);
  moveMotor(2, LOW, 0.75);
  moveMotor(3, LOW, 0.75);
  moveMotor(4, LOW, 0.75);
}
void west(){
  moveMotor(1, LOW, 0.75);
  moveMotor(2, HIGH, 0.75);
  moveMotor(3, HIGH, 0.75);
  moveMotor(4, LOW, 0.75);
}
void east(){
  moveMotor(1, HIGH, 0.75);
  moveMotor(2, LOW, 0.75);
  moveMotor(3, LOW, 0.75);
  moveMotor(4, HIGH, 0.75);
}
void nw(){
  moveMotor(1, HIGH, 0);
  moveMotor(2, HIGH, 0.75);
  moveMotor(3, HIGH, 0.75);
  moveMotor(4, HIGH, 0);
}
void ne(){
  moveMotor(1, HIGH, 0.75);
  moveMotor(2, HIGH, 0);
  moveMotor(3, HIGH, 0);
  moveMotor(4, HIGH, 0.75);
}
void sw(){
  moveMotor(1, LOW, 0.75);
  moveMotor(2, HIGH, 0);
  moveMotor(3, HIGH, 0);
  moveMotor(4, LOW, 0.75);
}
void se(){
  moveMotor(1, HIGH, 0);
  moveMotor(2, LOW, 0.75);
  moveMotor(3, LOW, 0.75);
  moveMotor(4, HIGH, 0);
}
void allDirsTest(){
  north();
  delay(1000);
  stop();
  delay(1000);
  south();
  delay(1000);
  stop();
  delay(1000);
  east();
  delay(1000);
  stop();
  delay(1000);
  west();
  delay(1000);
  stop();
  delay(1000);
  nw();
  delay(1000);
  stop();
  delay(1000);
  se();
  delay(1000);
  stop();
  delay(1000);
  sw();
  delay(1000);
  stop();
  delay(1000);
  ne();
  delay(1000);
  stop();
  delay(1000);
}
