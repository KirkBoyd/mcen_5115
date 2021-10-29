char dataString[50] = {0};
int a =0; 
char test[] = "<MOT-CWO|50>";

void setup() {
Serial.begin(9600);              //Starting serial communication
}
  
void loop() {
  a++;                          // a value increase every loop
  sprintf(dataString,"%02X",a); // convert a value to hexa 
  Serial.println(test);   // send the data
  delay(1000);                  // give the loop some break
}
