/* Got from roboticsbackend.com/raspberry-pi-arduino-serial-communication/
 
 */
// Retyped by Kirk Boyd
// Last modified Nov 16, 2021

void setup(){
  Serial.begin(9600);
}

void loop(){
  if(Serial.available() > 0){
      
    String data = Serial.readStringUntil('\n');
    subdivideStr(data);
//    Serial.print("You sent me: ");
//    Serial.println(data);
  }
  
}

void subdivideStr(String packet){
  bool receiving = 0;
  bool cmdReceived = 0;
  bool motorCmd = 0;
  char cmdBuffer[3];
  int cmdIndex = 0;
  int arrayOfInts[16];
  int arrayOfIntsIndex = 0;
  int len = packet.length();
  char bytes[len];
  packet.toCharArray(bytes, len);
  Serial.println("Parsing String");
  Serial.print("Packet Length: ");
  Serial.println(len);
  char serByte; 
  for(int i=0; i<len; i++){
    serByte = bytes[i];
    Serial.print("serByte = ");
    Serial.println(serByte);
    if(isWhiteSpace(serByte)){ continue; }
    
    if(serByte == '<' and !receiving){
      receiving = true;
      Serial.println("Received start marker");
      continue;
    }
    else if(serByte == '<' and receiving){
      Serial.println("Splish Splash the Data is Trash");
      return;
    }
    if(serByte == '>'){ //End marker has been received
      Serial.println("Received end marker");
      return; 
    } 
    if(receiving){ //Started receiving command
        if(!cmdReceived){
          if(serByte == '|'){ //Command Sperator has been found
            cmdReceived = true; //Aknowledge new command
            continue;  //Break from loop
          }
          else if(serByte == '>'){ return; } //End marker has been received
          else if(cmdIndex < 3){ //cmdBuffer has not been filled
            cmdBuffer[cmdIndex] = serByte; //Add the byte to the command index
            cmdIndex++; //Increment the command buffer
          }
          else{ //Command it too long
            Serial.println("Command to long");
            return;
          }
        }
        else{ //Command has been received
          if(cmdBuffer == "MOT"){ //Motor command has been received
            Serial.println("MOT Received");
            if(serByte == '-'){ //
              Serial.println("Value Seperator received");
            }
            //now dig out numbers
          }
          else if(cmdBuffer == "STP"){
            Serial.println("STP Received");
            //Stop motors
            return;
          }
          else{
            Serial.println("Invalid Command");
            return;
          }
       } // Command Received
     } //End Receiving
  } //End For Loop
}
