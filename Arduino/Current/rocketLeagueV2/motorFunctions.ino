//order:
//fL fR bL bR
void moveMotor(int motorNum, float speedRatio, bool ina1, bool ina2){ // motorNum is which motor; dir is fwd or back; speed ratio is a number between 0 and 1 where 0 is stopped and 1 is full speed
  float percentage = .25;
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
