#include<Arduino_RouterBrige.h>
void setup() {
  Bridge.begin();
  Monitor.begin(9600);
}
void loop() {
  Monitor.println("hello");
  delay(1000);
}
