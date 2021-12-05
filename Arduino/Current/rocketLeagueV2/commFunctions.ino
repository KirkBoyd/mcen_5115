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
//  Serial.println("Parsing String");
//  Serial.print("Packet Length: ");
//  Serial.println(len);
  char serByte; 
  for(int i=0; i<len; i++){
    serByte = bytes[i];
//    Serial.print("serByte = ");
//    Serial.println(serByte);
    if(isWhiteSpace(serByte)){ continue; }    
    if(serByte == '<' and !receiving){
      receiving = true;
//      Serial.println("Received start marker");
      continue;
    }
    else if(serByte == '<' and receiving){
      Serial.println("Splish Splash the Data is Trash");
      break;
    }
    if(receiving){ //Started receiving command
        if(!cmdReceived){
//          Serial.println("cmd not yet received");
          if(serByte == '|'){ //Command Sperator has been found
            cmdReceived = true; //Aknowledge new command
//            Serial.println("Command Received");
            continue;  //Break from loop
          }
          else if(serByte == '>'){ 
            Serial.println("Received end");
            break; 
          } //End marker has been received
          else if(cmdIndex < 3){ //cmdBuffer has not been filled
            cmdBuffer = cmdBuffer + serByte; //Add the byte to the command index
//            Serial.print("cmdBuffer: ");
//            Serial.println(cmdBuffer);
            cmdIndex++; //Increment the command buffer
          }
          else{ //Command it too long
            Serial.println("Command too long");
            break;
          }
        }
        else{ //Command has been received
//          Serial.println("Building numbers from string");
          if(cmdBuffer == "MOT"){ //Motor command has been received
//            Serial.println("MOT Received");
            if(serByte == '-' or serByte == '>'){ //
//              Serial.println("Value Seperator received");
//              Serial.print("Storing tempNum: ");
//              Serial.println(tempNum);
//              Serial.print("at motor array index: ");
//              Serial.println(tempNumInd);
              motVals[motValsInd] = tempNum.toInt();
              
//              Serial.print("Motor: ");
//              Serial.print(motValsInd);
//              Serial.print(" is: ");
//              Serial.println(motVals[motValsInd]);
              tempNum = "";
              tempNumInd = 0;
              motValsInd++;
              
            }
            else{
//              Serial.println("Building up tempNum");
              tempNum = tempNum + serByte;
//              String serByteStr = String(serByte);
//              strcpy(tempNum,serByte);
//              strcat(tempNum,serByteStr);
//              Serial.print("tempNum: ");
//              Serial.println(tempNum);
              tempNumInd++;
            }
            //now dig out numbers
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
  char bytes[9];
  String binary;
  char serByte;
              //digitalWrite(26, HIGH);

  binary = String(IMUdata , BIN);
  binary.toCharArray(bytes,10);
  
  for (int i = 0; i <numPins; i++){
    Serial.print("Digital Pin: ");
    Serial.print(i);
    Serial.print(" is: ");
    Serial.print(pins[i]);
    Serial.print(" with value: ");
      serByte = bytes[i];
      if(serByte == '1')
      {
              digitalWrite(pins[i], HIGH);
              Serial.println(1);
              
        
      }
      else if(serByte =='0')
      {
             digitalWrite(pins[i], LOW);
                  Serial.println(0);
      }
      else{Serial.println();}
    } 
}
