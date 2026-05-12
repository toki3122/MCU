/*Create sketch to send Voltage and Current to NANO using SUART Port in this format:
<x.xx,y.yy> where, ‘<’ is the beginning mark and ‘>’ is the ending mark.*/
#include <SoftwareSerial.h>
SoftwareSerial SUART(5, 6); //SRX = 5, STX = 6
float V,I;
void setup(){
SUART.begin(9600);
float V1=(5.0/1023)*analogRead(A0);
float V2=(5.0/1023)*analogRead(A1);
V = V2-V1;
I = V/2.2; //unit mA 
}
void loop(){
SUART.print('<');
SUART.print(V,2);
SUART.print(',');
SUART.print(I,2);
SUART.print(‘>’);
delay(1000);
}
