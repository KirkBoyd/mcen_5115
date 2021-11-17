
//order:
//fL fR bL bR
//void mm(int speeds[], bool dirs[]){
//  speeds[1] = fLspeed;
//  speeds[2] = fRspeed;
//  speeds[3] = bLspeed;
//  speeds[4] = bRspeed;
//  dirs[1] = fLdir;
//  dirs[1] = fRdir;
//  dirs[1] = bLdir;
//  dirs[1] = fLdir;
//  digitalWrite(aIn1_f, dir)
//}
void moveMotor(int motorNum, bool dir, float speedRatio){ // motorNum is which motor; dir is fwd or back; speed ratio is a number between 0 and 1 where 0 is stopped and 1 is full speed
  if(motorNum == 1){
    digitalWrite(aIn1_FL, dir);
    digitalWrite(aIn2_FL, !dir);
    analogWrite(pwm_FL, speedRatio*255);
  }
  else if(motorNum == 2){
    digitalWrite(aIn1_FR, dir);
    digitalWrite(aIn2_FR, !dir);
    analogWrite(pwm_FR, speedRatio*255);  
  }
  else if(motorNum == 3){
    digitalWrite(aIn1_BL, dir);
    digitalWrite(aIn2_BL, !dir);
    analogWrite(pwm_BL, speedRatio*255);
  }
  else if(motorNum == 4){
    digitalWrite(aIn1_BR, dir);
    digitalWrite(aIn2_BR, !dir);
    analogWrite(pwm_BR, speedRatio*255);
  }
}

void stop(){
  moveMotor(mFL, HIGH, 0);
  moveMotor(mFR, HIGH, 0);
  moveMotor(mBL, HIGH, 0);
  moveMotor(mBR, HIGH, 0);
}
void north(){
  moveMotor(mFL, HIGH, 0.5);
  moveMotor(mFR, HIGH, 0.5);
  moveMotor(mBL, HIGH, 0.5);
  moveMotor(mBR, HIGH, 0.5);
}
void south(){
  moveMotor(mFL, LOW, 0.5);
  moveMotor(mFR, LOW, 0.5);
  moveMotor(mBL, LOW, 0.5);
  moveMotor(mBR, LOW, 0.5);
}
void west(){
  moveMotor(mFL, LOW, 0.5);
  moveMotor(mFR, HIGH, 0.5);
  moveMotor(mBL, HIGH, 0.5);
  moveMotor(mBR, LOW, 0.5);
}
void east(){
  moveMotor(mFL, HIGH, 0.5);
  moveMotor(mFR, LOW, 0.5);
  moveMotor(mBL, LOW, 0.5);
  moveMotor(mBR, HIGH, 0.5);
}
void nw(){
  moveMotor(mFL, HIGH, 0);
  moveMotor(mFR, HIGH, 0.5);
  moveMotor(mBL, HIGH, 0.5);
  moveMotor(mBR, HIGH, 0);
}
void ne(){
  moveMotor(mFL, HIGH, 0.5);
  moveMotor(mFR, HIGH, 0);
  moveMotor(mBL, HIGH, 0);
  moveMotor(mBR, HIGH, 0.5);
}
void sw(){
  moveMotor(mFL, LOW, 0.5);
  moveMotor(mFR, HIGH, 0);
  moveMotor(mBL, HIGH, 0);
  moveMotor(mBR, LOW, 0.5);
}
void se(){
  moveMotor(mFL, HIGH, 0);
  moveMotor(mFR, LOW, 0.5);
  moveMotor(mBL, LOW, 0.5);
  moveMotor(mBR, HIGH, 0);
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
