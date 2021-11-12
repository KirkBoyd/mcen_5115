void moveMotor(int motorNum, bool inA1, bool inA2, float speedRatio){ // motorNum is which motor; dir is fwd or back; speed ratio is a number between 0 and 1 where 0 is stopped and 1 is full speed
  float speedVal=.25;
  if(motorNum == 0){
    digitalWrite(aIn1_f, inA1);
    digitalWrite(aIn2_f, inA2);
    analogWrite(pwmA_f, speedRatio*speedVal);
  }
  else if(motorNum == 1){
    digitalWrite(bIn1_f, inA1);
    digitalWrite(bIn2_f, inA2);
    analogWrite(pwmB_f, speedRatio*speedVal);  
  }
  else if(motorNum == 2){
    digitalWrite(aIn1_b, inA1);
    digitalWrite(aIn2_b, inA2);
    analogWrite(pwmA_b, speedRatio*speedVal);
  }
  else if(motorNum == 3){
    digitalWrite(bIn1_b, inA1);
    digitalWrite(bIn2_b, inA2);
    analogWrite(pwmB_b, speedRatio*speedVal);
  }
}
