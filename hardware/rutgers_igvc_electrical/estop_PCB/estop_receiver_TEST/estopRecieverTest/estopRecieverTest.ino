#include <esp_now.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "ADS1X15.h"

ADS1115 ADS(0x48);
// reciever
// this esp32 will recieve a string that will contain "true" for triggering the estop or "false" for not.
// using strings instead of booleans because I don't know if the protocol automatically implements CRCs or anything and a string wont suffer from a bit flip.

// STRINGS MIGHT NOT WORK, USE CHAR ARRAY INSTEAD FOR FIXED SIZE


String estopSignal; // needed for the callback function to know what to expect

bool estop = false; // actual state, as controlled by gpio pins
bool prevState = false;

int relayPin = 33; // right above gnd on my board, LIKELY DIFFERENT ON OTHERS
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.print("Recieving. This ESP32's MAC adress is: ");
  Serial.println(WiFi.macAddress());
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); 

  //ADS1115 (Current Reader Stuff)
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  ADS.begin();
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info* mac, const uint8_t *incomingData, int len) {
  memcpy(&estopSignal, incomingData, sizeof(estopSignal));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Should the estop be activated?: ");
  Serial.println(estopSignal);

  if (estopSignal == "true") {
      estop = true;
  } else if (estopSignal == "false") {
      estop = false;
  } else {
      Serial.println("Recieved invalid data. Should only recieve \"true\" or \"false\" right now");
  }
}
 
void loop() {
    if (estop == prevState)
        return;

    if (estop == false)
        digitalWrite(relayPin,  HIGH);
  
    if (estop == true)
        digitalWrite(relayPin, LOW);

    prevState = estop;
    Serial.print("Changed pin state");

    //ADS1115 (THIS HAS NOT BEEN TESTED YET!)
    

    int16_t val = ADS.readADC(3);

    float f = ADS.toVoltage(2);
    Serial.print("\tAnalog 3: "); 
    Serial.print(val); 
    Serial.print('\t'); 
    Serial.println(val * f, 3);
    delay(250);
  
}

// does the callback function still work when a loop is running?
