bool isWhiteSpace(char character){
    if (character == ' ')
        return true;
    if (character == '\r')
        return true;
    if (character == '\n')
        return true;
    return false;
}
void subdivideStr(String packet){
  bool receiving = 0;
  bool cmdReceived = 0;
  bool motorCmd = 0;
  String cmdBuffer = "";
  int cmdIndex = 0;
  int arrayOfInts[16];
  int arrayOfIntsIndex = 0;
  int len = packet.length();
  char bytes[len];
  String tempNum = "";
  int tempNumInd = 0;
  int motValsInd = 0;
  
  
  packet.toCharArray(bytes, len);
  char serByte; 
  for(int i=0; i<len; i++){
    serByte = bytes[i];
    if(isWhiteSpace(serByte)){ continue; }    
    if(serByte == '<' and !receiving){
      receiving = true;
      continue;
    }
    else if(serByte == '<' and receiving){
      Serial.println("Splish Splash the Data is Trash");
      break;
    }
    if(receiving){ //Started receiving command
        if(!cmdReceived){
          if(serByte == '|'){ //Command Sperator has been found
            cmdReceived = true; //Aknowledge new command
            continue;  //Break from loop
          }
          else if(serByte == '>'){ 
            Serial.println("Received end");
            break; 
          } //End marker has been received
          else if(cmdIndex < 3){ //cmdBuffer has not been filled
            cmdBuffer = cmdBuffer + serByte; //Add the byte to the command index
            cmdIndex++; //Increment the command buffer
          }
          else{ //Command it too long
            Serial.println("Command too long");
            break;
          }
        }
        else{ //Command has been received
          if(cmdBuffer == "MOT"){ //Motor command has been received
            if(serByte == '-' or serByte == '>'){ //
              motVals[motValsInd] = tempNum.toInt();
              tempNum = "";
              tempNumInd = 0;
              motValsInd++;
              
            }
          else if(cmdBuffer == "STP"){
            Serial.println("STP Received");
            motVals[0] = 0;
            motVals[1] = 0;
            motVals[2] = 0;
            motVals[3] = 0;
            motVals[4] = 1;
            motVals[5] = 1;
            motVals[6] = 1;
            motVals[7] = 1;
            motVals[8] = 1;
            motVals[9] = 1;
            motVals[10] = 1;
            motVals[11] = 1;
            break;
          }
          else{
            Serial.println("Invalid Command");
            break;
          }
       } // Command Received
     } //End Receiving
     if(serByte == '>'){ //End marker has been received
      //Serial.println("Received end marker");
      break; 
     }
     
  } //End For Loop
} 

void sendIMU(int IMUdata){
  int numPins = 9;
  int pins[] = {26, 27, 28, 29, 30, 31, 32, 33, 34};
  int pinVal[9];
  //Serial.println(IMUdata);
  for (int i=0; i<numPins; i++){
    if( IMUdata - pow(2,numPins-i-1) >= 0){
      pinVal[i] = 1;
      IMUdata =  IMUdata - pow(2,numPins-i-1);
      digitalWrite(pins[i],HIGH);
      //Serial.print(1);
    }
    else{
      pinVal[i] = 0;
      digitalWrite(pins[i],LOW);
     //Serial.print(0);
    }
  }
  //Serial.println();
 }