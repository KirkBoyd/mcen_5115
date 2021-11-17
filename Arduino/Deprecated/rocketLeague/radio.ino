void getRadData(){
  if(radio.receiveDone()){
    Serial.print("got data: ");
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", message [");
    for (byte i=0; i<radio.DATALEN; i++){ 
      Serial.print((char)radio.DATA[i]);
       radData[i] = {(char)radio.DATA[i]};
       datarino = datarino + radData[i];
//       Serial.print("radData[");
//       Serial.print(i);
//       Serial.print("] is ");
//       Serial.print(radData[i]);
//       Serial.print("|");
    }
      Serial.print("datarino: ");
      Serial.println(datarino);
//    Serial.println(radData[]);
//    Serial.print("], RSSI ");
//    Serial.println(radio.RSSI);
  }
}

void lab6radioCheckExtraData(){
  if(radio.receiveDone()){
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", message [");
    for (byte i=0; i<radio.DATALEN; i++){ Serial.print((char)radio.DATA[i]); }
    Serial.print("], RSSI [");
    Serial.print(radio.RSSI);
    Serial.print("], DATALEN [");
    Serial.println(radio.DATALEN);
  }
}
void lab6radioCheck(){
  if(radio.receiveDone()){
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(", message [");
    for (byte i=0; i<radio.DATALEN; i++){ Serial.print((char)radio.DATA[i]); }
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);
  }
}
