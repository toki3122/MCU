#include<Arduino_RouterBrige.h>
void setup() {
  Bridge.begin();
  Monitor.begin();
  analogReadResolution(10);
}
void loop() {
  int adc=analogRead(A0);
  Bridge.notify("adc_data",adc);
  Monitor.print("sending data to mpu");
  Monitor.println(adc,HEX);
  delay(1000);
}
