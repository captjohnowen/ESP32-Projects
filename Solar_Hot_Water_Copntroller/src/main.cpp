

/*
 * This module is to montior the Solar Hot Water System
 *  Key components:
 *    - 3 Temperature monitors
 *      - at the Solar Panels
 *      - feed temperature
 *      - return temperature
 *    - System fluid pressure
 *    - System fluid flow
 *  Also monitored components controlled by Solar Controller:
 *    - System pump, running status, On or OFF
 *    - Normally Open Solinoid Status
 *    - Normally closed Solinoid status
 * Unit status also reports out via MQTT
 * 
 * John Owen
 * March 2002
 */

#include "Arduino.h"
#include "WiFi.h"
#include "AsyncMqttClient.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "EEPROM.h"


// these are for the OTA programming

#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "AsyncElegantOTA.h"

AsyncWebServer server(80);


#include <SPI.h>
//#include "Wire.h"
//#include "Adafruit_GFX.h"
//#include "Adafruit_SSD1306.h"
#include <U8g2lib.h>
//#include <Adafruit_BusIO_Register.h>
//#include "images.h"

// U8g2 Contructor
U8G2_SSD1309_128X64_NONAME2_1_4W_HW_SPI u8g2(U8G2_R0, 5, 16, 17); // cs= 5, dc= 16, reset = 17

//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 32 // OLED display height, in pixels


//#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



// Change the credentials below, so your ESP32 connects to your router
#define WIFI_SSID "Foxgrove"
#define WIFI_PASSWORD "S2i0s1k5o"

// Change the MQTT_HOST variable to your Raspberry Pi IP address, 
// so it connects to your Mosquitto MQTT broker
#define MQTT_HOST IPAddress(10, 0, 1, 214)
#define MQTT_PORT 1883

#define EEPROM_SIZE 1  // EEPROM memory for set point temperature


// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

String ipAddress;
String wifissid = WIFI_SSID;

// GPIO for controlling circulation pump, tank 1 solinoid, tank 2 solinoid
const int cirPump = 33;
const int tank1valve = 26;
const int tank2valve = 27;

// Set Point and Range pots

const int diffSetPointPin = 34;
const int diffRangePin = 35;
int diffSetPoint = 0;
int diffRange = 0;

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Debounce Timer

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 1000;



/** 
 * 
 * set global variables
 * 
 **/
 
// Set String variables

String tank1TempString = "";  // tank 1 temperature
String tank2TempString = "";  // tank 2 temperature
String feedTempString = "";   // fluid temperature feeding to solar panels
String returnTempString = "";  // fluid temperature returning from solar panels
String collectorTempString = "";  // solar panel fluid temperature
String flowRateString = "";             // fluid flow rate
String differentialSetPointString = ""; // temperature differential between tank temp and collector temp
String differentialRangeString = ""; // temperature range for starting and stopping
String systemPressureString = ""; // system fluid pressure in PSI
String cirPumpString = "";  // circulation pump running or not
String systemStatus = "";
String tank1ValveString = "";
String tank2ValveString = "";

// Set Int variables
int tank1TempInt = 0;  // tank 1 temperature
int tank2TempInt = 0;  // tank 2 temperature
int feedTempInt = 0;  // fluid temperature feeding to solar panels
int returnTempInt = 0;  // fluid temperature returning from solar panels
int collectorTempInt = 0;  // solar panel fluid temperature
float flowRate = 0.0, l_minute;  // fluid flow rate
int differentialSetPointInt = 12;   // temperature differential between tank temp and collector temp
int differentialRangeInt = 8; // temperature range for starting and stopping
int systemPressureInt = 0; // system fluid pressure in PSI
int cirPumpInt = 0;  // circulation pump running or not
int cirPumpTimer = 0;
int cirPumpDelay = 30000;  // 2.5 delay to make sure the pump doesn't over cycle
int ADCValue = 0; // reading from Analog to Digital Converter
int pageNumber = 0; // screen display page number
int buttonState;
int lastButtonState = LOW;
int lastButtonPushTime = 0;




//variables for averaging collector temps
const int numReadings = 10;
int collectorTempReading[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

// Set boolean flags

bool heatTank1Flag = false;
bool heatTank2Flag = false;

//GPIO for ADC input for pressure sensor
const int presSensorInput = 39;

// GPIO where the DS18B20(s) is connected to
const int oneWireBus = 32;

//GPIO for flow sensor
const int flowSensor = 25;
volatile int flowFrequency;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);


// Device addresses - use "DallasOneWire Address Finder" to determine addresses before installation of probe
DeviceAddress tempSensorFeed = { 0x28, 0xB8, 0xB8, 0xFA, 0x58, 0x20, 0x1, 0x62 }; //  ID Number: 1, 50mm
DeviceAddress tempSensorReturn = { 0x28, 0xD1, 0x39, 0x27, 0x57, 0x20, 0x1, 0xF2 }; // ID Number: 2, 50mm
DeviceAddress tempSensorCollector = { 0x28, 0x46, 0x6C, 0x2C, 0xD, 0x0, 0x0, 0x9B }; // ID Number: 5, 100mm
DeviceAddress tempSensorTank1 = { 0x28, 0x1F, 0x3F, 0xD2, 0xB1, 0x21, 0x6, 0x43 }; // ID Number: 11, 400mm
DeviceAddress tempSensorTank2 = { 0x28, 0xEB, 0x1A, 0x14, 0xB2, 0x21, 0x6, 0x13 }; // ID Number: 12 400mm

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;         // interval at which to publish sensor readings

const int screenButtonPin = 36;

// button debounce

long lastDebounceTimeButton = 0;
long debounceDelayButton = 50;

/**
 * Intro / Startup screen
 * 
 */
void refreshDisplayScreen0() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_unifont_tf);
    u8g2.setCursor(2,18);
    u8g2.println("Foxgrove");
    u8g2.setCursor(2,30);
    u8g2.println("Solar Hot Water");
    u8g2.setCursor(2,42);
    u8g2.println("Controller");
    u8g2.setCursor(0,62);
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
   
}

/**
 * Default screen
 * 
 */
void refreshDisplayScreen1() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,10);
    u8g2.println("Solar Hot Water Controller");
    u8g2.setCursor(0,20);
    u8g2.println("Feed: ");
    u8g2.setCursor(45,20);
    u8g2.println(feedTempString);
    u8g2.setCursor(65,20);
    u8g2.println("Return: ");
    u8g2.setCursor(105,20);
    u8g2.println(returnTempString);
    u8g2.setCursor(0,30);
    u8g2.println ("Tank 1: ");
    u8g2.setCursor(45,30);
    u8g2.println (tank1TempString);
    u8g2.setCursor(65,30);
    u8g2.println ("Tank 2: ");
    u8g2.setCursor(105,30);
    u8g2.println (tank2TempString);
    u8g2.setCursor(0,40);
    u8g2.println("Collector: ");
    u8g2.setCursor(45,40);
    u8g2.println(collectorTempString);
    u8g2.setCursor(65,40);
    u8g2.println ("Cir Pump: ");
    u8g2.setCursor(105,40);
    u8g2.println (cirPumpString);
    u8g2.setCursor(0,50);
    u8g2.println ("Pressure: ");
    u8g2.setCursor(45,50);
    u8g2.println (systemPressureString);
    u8g2.setCursor(65,50);
    u8g2.println ("Flow: ");
    u8g2.setCursor(105,50);
    u8g2.println (flowRateString);
    u8g2.setCursor(0,62);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
   
}

/**
 * Default screen
 * 
 */
void refreshDisplayScreen2() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,10);
    u8g2.println("Solar Hot Water Controller");
    u8g2.setFont(u8g2_font_profont10_tf );
    u8g2.setCursor(0,28);
    u8g2.println("Tank 1 Temp: ");
    u8g2.setCursor(80,28);
    u8g2.println(tank1TempString);
    u8g2.setCursor(0,46);
    u8g2.println("Tank 1 Valve: ");
    u8g2.setCursor(80,46);
    if (tank1valve == 1){
      u8g2.println("OPEN");
    } else {
      u8g2.println("CLOSED");
    }
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,62);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
}

/**
 * Default screen
 * 
 */
void refreshDisplayScreen3() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,10);
    u8g2.println("Solar Hot Water Controller");
    u8g2.setFont(u8g2_font_profont10_tf );
    u8g2.setCursor(0,28);
    u8g2.println("Tank 2 Temp: ");
    u8g2.setCursor(80,28);
    u8g2.println(tank2TempString);
    u8g2.setCursor(0,46);
    u8g2.println("Tank 2 Valve: ");
    u8g2.setCursor(80,46);
    if (tank2valve == 1){
      u8g2.println("OPEN");
    } else {
      u8g2.println("CLOSED");
    }
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,62);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
}

void refreshDisplayScreen4() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,10);
    u8g2.println("Solar Hot Water Controller");
    u8g2.setFont(u8g2_font_profont10_tf );
    u8g2.setCursor(0,28);
    u8g2.println("Network: ");
    u8g2.setCursor(50,28);
    u8g2.println(wifissid);
    u8g2.setCursor(0,46);
    u8g2.println("IP Addr: ");
    u8g2.setCursor(50,46);
    u8g2.println(ipAddress);
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,62);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
}

void refreshDisplayScreen5() {  //refreshs the SSD 1309 OLED display
   
   u8g2.firstPage();
   do {
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,10);
    u8g2.println("Solar Hot Water Controller");
    u8g2.setFont(u8g2_font_profont10_tf );
    u8g2.setCursor(0,20);
    u8g2.println("Set Point: ");
    u8g2.setCursor(50,20);
    u8g2.println(differentialSetPointString);
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,30);
    u8g2.println("Minimum: 8 - Maximum: 16");
    u8g2.setFont(u8g2_font_profont10_tf );
    u8g2.setCursor(0,40);
    u8g2.println("Range: ");
    u8g2.setCursor(50,40);
    u8g2.println(differentialRangeString);
    u8g2.setFont(u8g2_font_smallsimple_tr);
    u8g2.setCursor(0,50);
    u8g2.println("Minimum: 2 - Maximum: 6");
    u8g2.setCursor(0,62);
    u8g2.println ("Status: " + systemStatus);
   } while ( u8g2.nextPage() );
}

void refreshDisplay() {  //refreshs the SSD 1309 OLED display wiith selected screen

  switch (pageNumber)
  {
  case 0:
    refreshDisplayScreen0();
    break;
  case 1:
    refreshDisplayScreen1();
    break;
  case 2:
    refreshDisplayScreen2();
    break;
  case 3:
    refreshDisplayScreen3();
    break;
  case 4:
    refreshDisplayScreen4();
    break;
  case 5:
    refreshDisplayScreen5();
    break;
  default:
    refreshDisplayScreen1();
    break;
  }

}  

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(100);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// Function for checking connection to WiFi and then, if connected, connect to MQTT
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      ipAddress = WiFi.localIP().toString().c_str();
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// Add more topics that want your ESP32 to be subscribed to
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // Topics for subscribing to MQTT topics
  // outputA  Upper Socket
  // outputB  Lower Socket
  // ESP32 subscribed to Solar Hot Water heartbeat topic
  uint16_t packetIdSubHB = mqttClient.subscribe("SolarHW/heartbeat", 0);
  Serial.print("Heartbeat - Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSubHB);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

// You can modify this function to handle what happens when you receive a certain message in a specific topic
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  messageTemp.trim();
  String messageTopic = topic;
  Serial.println("Received topic: " + messageTopic);
  // Response to heartbeat check
  if (strcmp(topic, "SolarHW/heartbeat") == 0) { 
    // If the LED is off turn it on (and vice-versa)
    messageTemp.trim();
    Serial.println("Message Temp: " + messageTemp);
    if (messageTemp == "checkPulse") {
      //delay (2000);
      uint16_t packetIdPubHB = mqttClient.publish("SolarHW/pulse", 2, true, "strongPulse");
      Serial.print("Publishing on topic SolarHW/heartbeat at QoS 2, packetId: ");
      Serial.println(packetIdPubHB);
    }
  }
}

void circulationPump() {
  if ( heatTank1Flag == true ) {
    digitalWrite ( cirPump, HIGH );
    cirPumpString = "ON";
    uint16_t packetIdPubCPS = mqttClient.publish("SolarHW/cirPumpState", 2, true, "ON");
    Serial.println (packetIdPubCPS);
    Serial.println ( "Cir Pump: ON, Heating Tank 1");
    Serial.println ( "Cir Pump Delay: " + String(cirPumpTimer));
  } else if ( heatTank2Flag == true ) {
    digitalWrite ( cirPump, HIGH );
    cirPumpString = "ON";
    uint16_t packetIdPubCPS = mqttClient.publish("SolarHW/cirPumpState", 2, true, "ON");
    Serial.println (packetIdPubCPS);
    Serial.println ( "Cir Pump: ON, min cycle" );
    Serial.println ( "Cir Pump Delay: " + String(cirPumpTimer));
  } else if ( heatTank1Flag == false || heatTank2Flag == false) {
    digitalWrite ( cirPump, LOW);
    cirPumpString = "OFF";
    uint16_t packetIdPubCPS = mqttClient.publish("SolarHW/cirPumpState", 2, true, "OFF");
    Serial.println (packetIdPubCPS);
    cirPumpTimer = 0;
    Serial.println ( "Cir Pump: OFF" );
    Serial.println ( "Cir Pump Delay: " + String(cirPumpTimer));
  }
}

void heatTanks () {
  if ( collectorTempInt >= ( tank1TempInt + differentialSetPointInt )) {  // Start heating Tank 1 when delta is greater than or equal to tank 1 temp plus set point
    // Start heating tank 1, when tank reaches 180F OR collector is within differential range
    if ( collectorTempInt >= (tank1TempInt + ( differentialSetPointInt - differentialRangeInt ))) {
      heatTank1Flag = true;
      circulationPump(); // tank 1 valve is normally open so remains in LOW state
      Serial.println ("Tank 1: Heating");
      systemStatus = "Tank 1 Heating";
      uint16_t packetIdPubT1V = mqttClient.publish("SolarHW/tank1valveState", 2, true, "OPEN");
      Serial.println ("Tank 1 Valve: OPEN");
      Serial.println (packetIdPubT1V);
      // If tank 1 is at 180F or within differential, stop heating tank 1 and start heating tank 2
    } else if ( tank1TempInt >= 180 || collectorTempInt <= ( tank1TempInt + ( differentialSetPointInt - differentialRangeInt ))){  
      digitalWrite ( tank1valve, HIGH); // closes tank 1 valve, stop heating Tank 1
      heatTank1Flag = false;
      circulationPump();
      Serial.println ("Tank 1: Heating Finished");
      uint16_t packetIdPubT1V = mqttClient.publish("SolarHW/tank1valveState", 2, true, "CLOSED");
      Serial.println ("Tank 1 Valve: CLOSED");
      Serial.println (packetIdPubT1V);
    }
  } else if ( collectorTempInt >= ( tank2TempInt + differentialSetPointInt )) { 
      if ( collectorTempInt >= ( tank2TempInt + (differentialSetPointInt - differentialRangeInt))) {  
        digitalWrite ( tank1valve, HIGH);
        digitalWrite ( tank2valve, HIGH);
        heatTank2Flag = true;
        Serial.println ("Tank 2: Heating");
        systemStatus = "Tank 2 Heating";
        circulationPump();
        uint16_t packetIdPubCPS = mqttClient.publish("SolarHW/cirPumpState", 2, true, "ON");
        Serial.println (packetIdPubCPS);
        uint16_t packetIdPubT1V = mqttClient.publish("SolarHW/tank1valveState", 2, true, "CLOSED");
        Serial.println ("Tank 1 Valve: CLOSED");
        Serial.println (packetIdPubT1V);
        uint16_t packetIdPubT2V = mqttClient.publish("SolarHW/tank2valveState", 2, true, "OPEN");
        Serial.println ("Tank 2 Valve: OPEN");
        Serial.println (packetIdPubT2V);

      } else if ( tank2TempInt >= 180 || collectorTempInt <= ( tank2TempInt + ( differentialSetPointInt - differentialRangeInt ))) {
        digitalWrite ( tank1valve, LOW );
        digitalWrite ( tank2valve, LOW );
        Serial.println ("Tank 2: Heating Finished");
        uint16_t packetIdPubT1V = mqttClient.publish("SolarHW/tank1valveState", 2, true, "OPEN");
        Serial.println ("Tank 1 Valve: OPEN");
        Serial.println (packetIdPubT1V);
        uint16_t packetIdPubT2V = mqttClient.publish("SolarHW/tank2valveState", 2, true, "CLOSED");
        Serial.println ("Tank 2 Valve: CLOSED");
        Serial.println (packetIdPubT2V);
        heatTank2Flag = false;
        circulationPump();
        
      }
  } else {
    digitalWrite ( tank1valve, LOW);
    digitalWrite ( tank2valve, LOW);
    uint16_t packetIdPubT1V = mqttClient.publish("SolarHW/tank1valveState", 2, true, "OPEN");
    Serial.println ("Tank 1 Valve: OPEN");
    Serial.println (packetIdPubT1V);
    uint16_t packetIdPubT2V = mqttClient.publish("SolarHW/tank2valveState", 2, true, "CLOSED");
    Serial.println ("Tank 2 Valve: CLOSED");
    heatTank1Flag = false;
    heatTank2Flag = false;
    circulationPump();
    systemStatus = "Heating Complete";
    Serial.println ("Tank Heating Complete");
  }
}



void flow () {  // Interrupt Function
  flowFrequency++;
}

  

void setup() {
  Serial.begin(115200);

  u8g2.begin();
  
  u8g2.setFont(u8g2_font_smallsimple_tr);
  u8g2.setFontMode(0);


  // Start the DS18B20 sensor
  sensors.begin();
  sensors.setResolution(tempSensorCollector, 9);
  sensors.setResolution(tempSensorTank1, 9);
  sensors.setResolution(tempSensorTank2, 9);
  sensors.setResolution(tempSensorFeed, 9);
  sensors.setResolution(tempSensorReturn, 9);

  for ( int thisReading = 0; thisReading < numReadings; thisReading++) {
    collectorTempReading[thisReading] = 0;
  }


  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM Begin");
  
  //temperatureIntSet = EEPROM.read(0);
  //temperatureIntRange = EEPROM.read(1);


  //define output pin and set to LOW
  pinMode (cirPump, OUTPUT );
  pinMode (tank1valve, OUTPUT );
  pinMode (tank2valve, OUTPUT );
  pinMode (flowSensor, INPUT); // for flowmeter
  pinMode ( screenButtonPin, INPUT);
  
  digitalWrite (cirPump, LOW );
  digitalWrite( tank1valve, LOW );
  digitalWrite( tank2valve, LOW );
  digitalWrite (flowSensor, HIGH); // for flowmeter

  

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  // setup for OTA
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);
  server.begin();

  attachInterrupt(digitalPinToInterrupt(flowSensor), flow, RISING);  // Sets up interrupt

}

void loop() {

  int reading = digitalRead (screenButtonPin);

  if ( reading != lastButtonState ) {  // resets the debounce timer
    lastDebounceTimeButton = millis();
  }

  if ( (millis() - lastDebounceTimeButton ) >= debounceDelayButton ){
    if ( reading != buttonState ) {
      buttonState = reading;
      u8g2.setPowerSave(0);
      lastButtonPushTime = 0;
      Serial.println (" Button State: " + String(buttonState));
      if ( buttonState == HIGH){
        pageNumber = pageNumber + 1;
        if ( pageNumber > 5) {
        pageNumber = 0;
        }
      }
    }
  }
  lastButtonState = reading;
  refreshDisplay();

  

  diffSetPoint = analogRead(diffSetPointPin);
  diffRange = analogRead(diffRangePin);
  
  int differentialSetPointIntNew = floatMap(diffSetPoint, 0, 4095, 8, 16);
  if (differentialSetPointIntNew != differentialSetPointInt){
    differentialSetPointInt = differentialSetPointIntNew;
    differentialSetPointString = String(differentialSetPointInt);
    Serial.println(" Differnetial Set Point: " + differentialSetPointString);
  };
  
  int differentialRangeIntNew = floatMap(diffRange, 0, 4095, 2, 6);
  if (differentialRangeIntNew != differentialRangeInt){
    differentialRangeInt = differentialRangeIntNew;
    differentialRangeString = String(differentialRangeInt);
    Serial.println(" Differential Range: " + differentialSetPointString);
  };



  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 5 seconds) 
  // it publishes a new MQTT message on topic esp32/temperature
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New temperature readings
    sensors.requestTemperatures();

    feedTempInt = sensors.getTempF(tempSensorFeed);
    returnTempInt = sensors.getTempF(tempSensorReturn);
    tank1TempInt = sensors.getTempF(tempSensorTank1);
    tank2TempInt = sensors.getTempF(tempSensorTank2);

    /**
     * This section provides an average collector temperature value
     */

    
    total = total - collectorTempReading[readIndex];  // subtract the last reading
    collectorTempReading[readIndex] = sensors.getTempF(tempSensorCollector);  // read the value from the collector
    total = total + collectorTempReading[readIndex];  // totals the readings in the array
    readIndex = readIndex + 1;  // move to the next reading
    if ( readIndex >= numReadings) {  // if at the end of the array
      readIndex = 0;
    }
    // Calcualte the average
    collectorTempInt = total / numReadings;

    /**
     * Reads the Analog Input and converts to PSI
     */
    ADCValue = analogRead(presSensorInput);  // ADC input from pressure sensor 
    systemPressureInt = (ADCValue * 10 ) / (4095);

    /**
     * Counts Hall Effect sensor pulse from the flow meter and converts to GPM
     */

    if (flowFrequency != 0) {
    l_minute = (flowFrequency / 7.5); // (Pulse frequency x 60 min ) / 7.5Q = flow rate in L/hour
    }

    feedTempString = String (feedTempInt);
    uint16_t packetIdPubFT = mqttClient.publish("SolarHW/feedTemperature", 2, true, feedTempString.c_str());
    Serial.println ("Feed Temp: " + feedTempString);
    Serial.println (packetIdPubFT);
    
    returnTempString = String (returnTempInt);
    uint16_t packetIdPubRT = mqttClient.publish("SolarHW/returnTemperature", 2, true, returnTempString.c_str());
    Serial.println ("Return Temp: " + returnTempString);
    Serial.println (packetIdPubRT);
    
    collectorTempString = String(collectorTempInt);
    uint16_t packetIdPubCT = mqttClient.publish("SolarHW/collectorTemperature", 2, true, collectorTempString.c_str());
    Serial.println ("Collector Temp: " + collectorTempString);
    Serial.println (packetIdPubCT);

    tank1TempString = String(tank1TempInt);
    uint16_t packetIdPubT1 = mqttClient.publish("SolarHW/tank1temperature", 2, true, tank1TempString.c_str());
    Serial.println ("Tank1 Temp: " + tank1TempString);
    Serial.println (packetIdPubT1);

    tank2TempString = String(tank2TempInt);
    uint16_t packetIdPubT2= mqttClient.publish("SolarHW/tank2temperature", 2, true, tank2TempString.c_str());
    Serial.println ("Tank2 Temp: " + tank2TempString);
    Serial.println (packetIdPubT2);

    systemPressureString = String (systemPressureInt);
    uint16_t packetIdPubSysP = mqttClient.publish("SolarHW/systemPressure", 2, true, systemPressureString.c_str());
    Serial.println ("System Pressure: " + systemPressureString);
    Serial.println (packetIdPubSysP);

    flowRateString = String (flowRate);
    uint16_t packetIdPubFR = mqttClient.publish("SolarHW/flowRate", 2, true, flowRateString.c_str());
    Serial.println ("Flow Rate (GPM): " + flowRateString);
    Serial.println (packetIdPubFR);

    


  
  heatTanks();  // runs the circulation pump, heat tank 1 - this tank will get heated first
  cirPumpTimer = cirPumpTimer + interval;
  Serial.println ("Page Number: " + String(pageNumber));
  lastButtonPushTime = lastButtonPushTime + interval;
  Serial.println (" Display Sleep Timer: " + String(lastButtonPushTime));
  if (lastButtonPushTime > 300000){
    u8g2.setPowerSave(1);
  }
  //refreshDisplay();

  

  }

}



    