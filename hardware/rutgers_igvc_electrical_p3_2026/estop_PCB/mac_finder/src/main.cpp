#include <esp_now.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.println(WiFi.macAddress());
}