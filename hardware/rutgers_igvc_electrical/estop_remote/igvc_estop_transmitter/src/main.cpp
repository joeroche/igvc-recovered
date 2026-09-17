#include "WiFi.h"
#include "esp_now.h"
#include "Arduino.h"

// 30:C6:F7:22:D6:58

// uint8_t broadcastAddress[] = {0xC8, 0x2E, 0x18, 0x5F, 0xD5, 0x24}; // mac adress of test receiver
// uint8_t broadcastAddress[] = {0xC8, 0x2E, 0x18, 0x5F, 0xD5, 0x8C}; // mac adress of real reciever (My ESP32)

uint8_t broadcastAddress[] = {0xC8, 0x2E, 0x18, 0x5F, 0xD5, 0x24};

//uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x22, 0xD6, 0x58};

// need to rewrite this to constantly ping the reciever with some ignored info and make sure they are in range and 
// communicating successfully. should work as is but wont know if they are out of range

int stopLower = 1;
int gentleLower = 5;
int unstopLower = 4;

int stopPin = 23; 
int gentlePin = 19;
int unstopPin = 15;

int stopLed = 22;
int gentleLed = 18;
int unstopLed = 2;

int errorPin = 2; // i think the onboard led is on gpio 2
String state = "";

unsigned long timeOfLast = 0; // time since last button press, used for debouncing
bool estop = false;

String estopSignal; // data to send 
char otherEstopSignal[10]; // use char array if string doesnt work

esp_now_peer_info_t peerInfo;

void activateLed(int num);
void sendSignal();


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // replacement for ground
  pinMode(stopLower, OUTPUT);
  pinMode(gentleLower, OUTPUT);
  pinMode(unstopLower, OUTPUT);
  digitalWrite(stopLower, HIGH);
  digitalWrite(gentleLower, HIGH);
  digitalWrite(unstopLower, HIGH);

  pinMode(stopPin, INPUT_PULLDOWN);
  pinMode(gentlePin, INPUT_PULLDOWN);
  pinMode(unstopPin, INPUT_PULLDOWN);

  pinMode(stopLed, OUTPUT);
  pinMode(gentleLed, OUTPUT);
  //pinMode(unstopLed, OUTPUT);  

  ledcSetup(0, 5000, 8);
  ledcAttachPin(unstopLed, 0);
  
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


}
 
void loop() {
  // Set values to send
  //strcpy(otherEstopSignal, "t/f");

  if ( digitalRead(stopPin) ) {
    activateLed(stopLed);
    Serial.println("Stop button pressed");
    unsigned long currentTime = millis();
    if (currentTime - timeOfLast > 200) { // will have to tune this time
        estopSignal = "true";
    }
    sendSignal();
    timeOfLast = currentTime;
  }

  if ( digitalRead(gentlePin) ) {
    activateLed(gentleLed);
    Serial.println("Gentle estop pressed");
    unsigned long currentTime = millis();
    if (currentTime - timeOfLast > 200) { // will have to tune this time
        estopSignal = "gentle";
    }
    sendSignal();
    timeOfLast = currentTime;
  }

  if ( digitalRead(unstopPin) ) {
    activateLed(unstopLed);
    Serial.println("Unstop buttton pressed");
    unsigned long currentTime = millis();
    if (currentTime - timeOfLast > 200) {
        estopSignal = "false";
    }
    sendSignal();
    timeOfLast = currentTime;
  }
  
  delay(50); // should this be based on interrupts or will that break ESP-Now?
}

// sends the signal. if the signal doesnt make it, led goes on and retries until it makes it
void sendSignal() { 

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &estopSignal, sizeof(estopSignal));
    if (result == ESP_OK) {
        //digitalWrite(errorPin, LOW);
    } else {
        //digitalWrite(errorPin, HIGH);
        while (true) {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &estopSignal, sizeof(estopSignal));
            if (result == ESP_OK) {
                //digitalWrite(errorPin, LOW);
                break;
            }
            delay(1000);
        }
        return;
    }
}

void activateLed(int num) {

  if( num != unstopLed ) {
    digitalWrite(num, HIGH);
  } else {
    ledcWrite(0, 25);
  }
  
  if( num != stopLed) {
    digitalWrite(stopLed, LOW);
  }

  if( num != gentleLed) {
    digitalWrite(gentleLed, LOW);
  }

  if( num != unstopLed) {
    //digitalWrite(unstopLed, LOW);
    ledcWrite(0, 0);
  }
}
