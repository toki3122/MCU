#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  int n = WiFi.scanNetworks();
  Serial.println("Scan done");
  if (n == 0) {
    Serial.println("NO NETWORKS FOUND");
  } 
  else {
    for (int i = 0; i < n; i++) {
      Serial.print(WiFi.SSID(i));
      Serial.print("  RSSI=");
      Serial.println(WiFi.RSSI(i));
    }
  }
}
void loop() {}
