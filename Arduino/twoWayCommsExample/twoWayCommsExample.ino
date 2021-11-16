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
    Serial.print("You sent me: ");
    Serial.println(data);
  }
}
 
