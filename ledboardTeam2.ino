/*
03-23-2020 - Zombo
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/////////////   ____                 _           _     _____ ____  /////////////
/////////////  / ___|___  _ __ _ __ | |__   ___ | |   | ____|  _ \ /////////////
///////////// | |   / _ \| '__| '_ \| '_ \ / _ \| |   |  _| | | | |/////////////
///////////// | |__| (_) | |  | | | | | | | (_) | |___| |___| |_| |/////////////
/////////////  \____\___/|_|  |_| |_|_| |_|\___/|_____|_____|____/ /////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

This is the inital start of CornhoLED using the ESP32 Microcontroller;

This system was designed with the purpose of creating a Smart Cornhole.

Using the new ESP-NOW protocol, we can implement a system that communicates with another microcontroller.

This can be done cheaper by just adding buttons,
    making the users click each point to match the lights.
    
Instead this systems allows full control of both boards through a either board!

Their data should always display the same as the other! one point is added or removed from one side?

it shows up in the code and the LEDS. 

Inital testing with IR is okay, 
  but I think it might be better to just have buttons and no IR sensor.
      Because players will be adding points themselves and removing. 
  

*/

#include "FastLED.h"

FASTLED_USING_NAMESPACE


#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

////////////FASTLED DEFINITIONS////////////
///////////////////////////////////
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    144
CRGB team1Leds[NUM_LEDS];
CRGB team2Leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120
/////////////////////////////////////


#include <esp_now.h>
#include <WiFi.h>
const long interval = 50;           // interval at which to blink (milliseconds)


//int isObstacle = HIGH;  // HIGH MEANS NO OBSTACLE
int counter=0;
//int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
//int IR1_out = HIGH; /* Avoiding initial false detections.    */
boolean state = true;
unsigned long currentMillis=0; 
unsigned long previousMillis = 0;        // will store last time LED was updated

// REPLACE WITH THE MAC Address of your receiver 
//B4:E6:2D:BF:5E:51 32 s
//uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x15, 0xDB, 0x50}; //devkit IP
uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0xBF, 0x5E, 0x51}; //32s IP

// Define variables to store BME280 readings to be sent
float team1_score=0;
float team2_score=0;
//float pressure;

// Define variables to store incoming readings
float incoming_team1_score;
float incoming_team2_score;

// Variable to store if sending data was successful
String success;

// button debounce for ir led
volatile unsigned long DebounceTimer;
volatile int irPressed;
volatile int piezoHit;
volatile unsigned int delayTime = 100;
volatile unsigned int delayTimeButton = 10;

// sensors

const int motionSensor = 14;
//const int piezoSensor = 14;
//team1
const int team1_addButton = 27;
const int team1_removeButton = 26;
#define TEAM1_LED_PIN    22
volatile int team1_addButton_ButtonPressed;
volatile int team1_removeButton_ButtonPressed;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


//team2
const int team2_addButton = 33;
const int team2_removeButton = 32;
#define TEAM2_LED_PIN    23
volatile int team2_addButton_ButtonPressed;
volatile int team2_removeButton_ButtonPressed;



//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float t1;
    float t2;
//    float pres;
} struct_message;

//struct irsensor {
//  const uint8_t PIN;
//  uint32_t numberKeyPresses;
//  bool pressed;
//};

//irsensor irsensor1 = {27, 0, false};

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



// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incoming_team1_score = incomingReadings.t1;
  incoming_team2_score = incomingReadings.t2;
  Serial.println(incoming_team1_score);
  Serial.println(incoming_team2_score);
  team1_score=incoming_team1_score;
  team2_score=incoming_team2_score;
  
}


float getscore2(){
  return team2_score;
}
void setscore2(){
  team2_score+=1;
}


void ledTasks(void * parameter){
//  ledWork();
  for(;;){ // infinite loop

    // Turn the LED on
//    digitalWrite(led1, HIGH);
  Serial.print("Team 1 Score: ");
  Serial.print(team1_score);
  Serial.println();
  Serial.print("Team 2 Score: ");
  Serial.print(team2_score);
  Serial.println();
  
    

    // Pause the task for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Turn the LED off
//    digitalWrite(led1, LOW);

    // Pause the task again for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void ledTasks2(void * parameter){
//  ledWork();
  for(;;){ // infinite loop

    // Turn the LED on
//    digitalWrite(led1, HIGH);
  // Move a single white led 
//    pride();
//    FastLED.show();
    int team1=team1_score;
    int team2=team2_score;
    if(0>team1){
      team1=0;
    }
    for(int i=0;i<team1;i++){
    team1Leds[i].setRGB(255,0,0);
//    leds[i].fadeLightBy(brightness);
    }
    for(int i=team1+1;i<NUM_LEDS;i++){
    team1Leds[i].setRGB(0,0,0);
//    leds[i].fadeLightBy(brightness);
    }
    if(0>team1){
      team1=0;
    }
    for(int i=0;i<team2;i++){
    team2Leds[i].setRGB(0,0,255);
//    leds[i].fadeLightBy(brightness);
    }
    for(int i=team1+1;i<NUM_LEDS;i++){
    team2Leds[i].setRGB(0,0,0);
//    leds[i].fadeLightBy(brightness);
    }
    FastLED.show();
    
    // Pause the task for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Turn the LED off
//    digitalWrite(led1, LOW);

    // Pause the task again for 500ms
//    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void ledInit(){

    FastLED.addLeds<LED_TYPE,TEAM1_LED_PIN,COLOR_ORDER>(team1Leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE,TEAM2_LED_PIN,COLOR_ORDER>(team2Leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
   


}
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Setting up Cornholed ESP-NOW");
  ledInit();
  
   // IR Motion Sensor mode INPUT_PULLUP
    Serial.println("Setting up LED Task");
   xTaskCreate(
    ledTasks,    // Function that should be called
    "LED Tasks",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );
    xTaskCreate(
    ledTasks2,    // Function that should be called
    "LED Tasks2",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    2,               // Task priority
    NULL             // Task handle
  );
   
   
  pinMode(motionSensor, INPUT_PULLUP);
  //team1
  pinMode(team1_removeButton, INPUT_PULLUP);
  pinMode(team1_addButton, INPUT_PULLUP);

  //team2
  pinMode(team2_removeButton, INPUT_PULLUP);
  pinMode(team2_addButton, INPUT_PULLUP);
  
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
//  attachInterrupt(irsensor1.PIN, detectsMovement, RISING);
  attachInterrupt(digitalPinToInterrupt(motionSensor), [] {if (irPressed+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);

  // piezo sensor
//  attachInterrupt(digitalPinToInterrupt(piezoSensor), [] {if (piezoHit+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);

  // Team 1
  attachInterrupt(digitalPinToInterrupt(team1_addButton), [] {if (team1_addButton_ButtonPressed+= (millis() - DebounceTimer) >= (delayTimeButton )) DebounceTimer = millis();}, RISING);
  attachInterrupt(digitalPinToInterrupt(team1_removeButton), [] {if (team1_removeButton_ButtonPressed+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);

  //Team 2
  attachInterrupt(digitalPinToInterrupt(team2_addButton), [] {if (team2_addButton_ButtonPressed+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);
  attachInterrupt(digitalPinToInterrupt(team2_removeButton), [] {if (team2_removeButton_ButtonPressed+= (millis() - DebounceTimer) >= (delayTime )) DebounceTimer = millis();}, RISING);
 
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

if (irPressed> 0) {
  Serial.println("Bag through Hole");
  irPressed = 0; // Must clear
  team2_score+=1;
//  team1_score+=1;
}
  
}

void getButtonData(){

////get the bag
//if (irPressed> 0) {
//  Serial.println("Bag through Hole");
//  irPressed = 0; // Must clear
//  team2_score+=1;
////  team1_score+=1;
//}

//if (piezoHit> 0) {
//  Serial.println("Piezo Hit");
//  piezoHit = 0; // Must clear
////  do an led thing!
//}




//adders
if (team1_addButton_ButtonPressed> 0) {
  Serial.println("Team 1 Point Added");
  team1_addButton_ButtonPressed = 0; // Must clear
  if (team1_score+1==22){
    // win situation?
    team1_score=22;
  }
  else{
  //  team1_score+=1;
  team1_score+=1;
  }

}

if (team2_addButton_ButtonPressed> 0) {
Serial.println("Team 2 Point Added");
  team2_addButton_ButtonPressed = 0; // Must clear
  if (team2_score+1==22){
    // win situation?
    team2_score=22;
  }
  else{
  //  team1_score+=1;
  team2_score+=1;
  }

}


// Removers
if (team1_removeButton_ButtonPressed> 0) {
  Serial.println("Team 1 Point Removed");
  team1_removeButton_ButtonPressed = 0; // Must clear
  if ((team1_score-1)==0){
    team1_score=0;
  }
  else{
  team1_score-=1;
  }

}
  


if (team2_removeButton_ButtonPressed> 0) {
  Serial.println("Team 2 Point Removed");
  team2_removeButton_ButtonPressed = 0; // Must clear
  if ((team2_score-1)==0){
    team2_score=0;
  }
  else{
  team2_score-=1;
  }

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



void ledWork(){

  //getScores(); // maybe return an array of team 1 and team 2 points.
//  set team1 score leds
//set team2 score leds
 pride();
 FastLED.show();  
FastLED.delay(1000/FRAMES_PER_SECOND); 


}


void loop() {

  getIRdata();
  getButtonData();
//  ledWork();
  

  
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
//  team1_score=incomingReadings.t1;
//   team2_score=incomingReadings.t2;
  Serial.print("t2: ");
  Serial.print(incomingReadings.t2);

}

void pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( team1Leds[pixelnumber], newcolor, 64);
    nblend( team2Leds[pixelnumber], newcolor, 64);
  }
}
