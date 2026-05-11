char myArray[20];
void setup(){ 
 Serial.begin(9600); 
}
void loop(){ 
while(Serial.available() > 0){ 
  byte m =Serial.readBytesUntil('\n', myArray, 20);
  myArray[m] = '\0';
  int x = atoi(myArray);
  Serial.println(x, DEC);
  Serial.println(x, HEX);
  Serial.println(x, BIN);
 }
}
