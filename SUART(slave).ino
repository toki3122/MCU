/*Create sketch to send Voltage and Current to NANO using SUART Port in this format:
<x.xx,y.yy> where, ‘<’ is the beginning mark and ‘>’ is the ending mark.*/

#include<SoftwareSerial.h>
SoftwareSerial SUART(5, 6); //SRX = 5, STX = 6
char arr[20];
char *token;
void setup(){
Serial.begin(9600);
SUART.begin(9600);
}
void loop(){
if (SUART.available()!= 0){
char y = SUART.read();
 if (y == '<'){ //beginning mark is found
 byte m = SUART.readBytesUntil('>', arr, 20);
 arr[m]='\0';
 Serial.println(arr);
 token = strtok(arr, ",");
 float V = atof(token);
 Serial.print(“Voltage: “);
Serial.print(V, 2); //2-digit after decimal point
Serial.println(“ Volt”);
float I = atof(strtok(NULL, ","));
Serial.print(“Current: “);
Serial.print(I, 2);
Serial.println(“ mA”);
}}}
