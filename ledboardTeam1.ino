/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
const long interval = 50;           // interval at which to blink (milliseconds)

const int motionSensor = 27;
//int isObstacle = HIGH;  // HIGH MEANS NO OBSTACLE
int counter=0;
//int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
//int IR1_out = HIGH; /* Avoiding initial false detections.    */
boolean state = true;
unsigned long currentMillis=0; 
unsigned long previousMillis = 0;        // will store last time LED was updated

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x15, 0xDB, 0x50};
//B4:E6:2D:BF:5E:51
uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0xBF, 0x5E, 0x51};

// Define variables to store BME280 readings to be sent
float team1_score=0;
float team2_score=0;
//float pressure;

// Define variables to store incoming readings
float incoming_team1_score;
float incoming_team2_score;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float t1;
    float t2;
//    float pres;
} struct_message;

struct irsensor {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

irsensor irsensor1 = {27, 0, false};

// Create a struct_message called BME280Readings to hold sensor readings
struct_message sensorReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";

  }
  else{
    success = "Delivery Fail :(";
  }
}
float getscore1(){
  return team1_score;
}
void setscore1(){
  team1_score+=1;
}


// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incoming_team1_score = incomingReadings.t1;
  incoming_team2_score = incomingReadings.t2;
  Serial.println(incoming_team1_score);
  Serial.println(incoming_team2_score);
  
}
volatile unsigned long DebounceTimer;
volatile int ButtonPressed;
volatile unsigned int delayTime = 100;
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Setting up Cornholed ESP-NOW");
   // IR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
//  attachInterrupt(irsensor1.PIN, detectsMovement, RISING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), [] {if (ButtonPressed+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);
 
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
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);



}

void getIRdata(){

if (ButtonPressed> 0) {Serial.println("Pressed");
  ButtonPressed = 0; // Must clear
  team1_score+=1;
}
  
}


void doespnowstuff(){
  // Set values to send
  sensorReadings.t1 = team1_score;
  sensorReadings.t2 = team2_score;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorReadings, sizeof(sensorReadings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
}
void loop() {

  getIRdata();

  
  doespnowstuff();
  updateDisplay();
  
  delay(1000);
}
void getReadings(){

//team1_score=random(0, 21);
team2_score=random(0, 21);
//getIRdata();

}

void updateDisplay(){
  Serial.println("INCOMING READINGS");
  Serial.print("t1: ");
  Serial.print(incomingReadings.t1);
  Serial.println(" ºC");
  Serial.print("t2: ");
  Serial.print(incomingReadings.t2);

}
