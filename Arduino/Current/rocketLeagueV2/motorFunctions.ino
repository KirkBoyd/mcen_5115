//order:
//fL fR bL bR
//<MOT|255-255-255-255-1-1-1-1->
//<MOT|255-255-255-255-0-0-0-0->
void moveMotor(int motorNum, float speedRatio, bool dir){ // motorNum is which motor; dir is fwd or back; speed ratio is a number between 0 and 1 where 0 is stopped and 1 is full speed
  float percentage = .5;
  bool ina1;
  bool ina2;
  if (dir){
    ina1 = true;
    ina2 = false;
  }else{
    ina1 = false;
    ina2 = true;
  }
  if(motorNum == 0){
    digitalWrite(aIn1_FL, ina1);
    digitalWrite(aIn2_FL, ina2);
    analogWrite(pwm_FL, speedRatio*percentage);
  }
  else if(motorNum == 1){
    digitalWrite(aIn1_FR, !ina1);
    digitalWrite(aIn2_FR, !ina2);
    analogWrite(pwm_FR, speedRatio*percentage);  
  }
  else if(motorNum == 2){
    digitalWrite(aIn1_BL, ina1);
    digitalWrite(aIn2_BL, ina2);
    analogWrite(pwm_BL, speedRatio*percentage);
  }
  else if(motorNum == 3){
    digitalWrite(aIn1_BR, !ina1);
    digitalWrite(aIn2_BR, !ina2);
    analogWrite(pwm_BR, speedRatio*percentage);
  }
}

void resetMotorDrivers(){
    digitalWrite(aIn1_FL, 0);
    digitalWrite(aIn2_FL, 0);
    analogWrite(pwm_FL, 0);
    digitalWrite(aIn1_FR, 0);
    digitalWrite(aIn2_FR, 0);
    analogWrite(pwm_FR, 0);  
    digitalWrite(aIn1_BL, 0);
    digitalWrite(aIn2_BL, 0);
    analogWrite(pwm_BL, 0);
    digitalWrite(aIn1_BR, 0);
    digitalWrite(aIn2_BR, 0);
    analogWrite(pwm_BR, 0);
}

//<MOT|255-255-255-255-1-1-1-1-0-0-0-0>
//<MOT|255-255-255-255-0-0-0-0-1-1-1-1>
//<MOT|255-255-255-255-1-1-1-1-1-1-1-1>
//<MOT|255-255-255-255-0-0-0-0-1-1-1-1>
//<MOT|255-255-255-255-0-0-0-0-0-0-0-0>
