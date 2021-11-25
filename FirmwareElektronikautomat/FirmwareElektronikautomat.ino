#include "config.h"

#include <Arduino.h>
#include <FastLED.h>

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>

// PIN Definitions
const int pin_IRSensor_SDO = 19;
const int pin_IRSensor_CLK = 18;
const int pin_IRSensor_Latch = 5;
const int pin_IRSensor_Value = 34; // GPIO32-GPIO39 für analogeingänge
const int pin_WS2812 = 23;

// LED Definitions
const int numLEDs = 60;
const int ledBrightness = 50;
const int ledMaxCurrent = 1000; // max Strom in mA
const int numSlotStatusLEDs = 3;
const int slotLEDIndices[] = { 3, 8, 13, 21 };

// IR Sensor Definitions
const int IRSensorInterval = 500;
const int IRSensorChannels = 16;
const int IRMeasurementTimePerSlot = 10;
const int ShiftOutUs = 10;
const int EmptySlotTreshold = 1000; // Analog value

volatile bool slotEmpty[10] = { false };

CRGB leds[numLEDs];

// Task Definitions
void TaskIRSensors( void *pvParameters );
void TaskWS2812( void *pvParameters );
void TaskWiFi( void *pvParameters );
void TaskDiagnostics( void *pvParameters );

// Task Handles
TaskHandle_t hTaskIRSensors, hTaskWS2812, hTaskWiFi;

// MQTT Classes
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  // Task Setup
  xTaskCreate(
    TaskIRSensors
    ,  "TaskIRSensors"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  1 // Priority
    ,  &hTaskIRSensors); // Handle

  xTaskCreate(
    TaskWS2812
    ,  "TaskWS2812"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  2 // Priority
    ,  &hTaskWS2812); // Handle

  xTaskCreate(
    TaskWiFi
    ,  "TaskWiFi"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  2 // Priority
    ,  &hTaskWiFi); // Handle

  xTaskCreate(
    TaskDiagnostics
    ,  "TaskDiagnostics"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  0 // Priority
    ,  NULL); // Handle
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskIRSensors(void *pvParameters)
{
  (void) pvParameters;

  int backgroundValue, sensorValue;

  // Pin Modes for IR Sensor communication
  pinMode(pin_IRSensor_SDO, OUTPUT);
  pinMode(pin_IRSensor_CLK, OUTPUT);
  pinMode(pin_IRSensor_Latch, OUTPUT);
  pinMode(pin_IRSensor_Value, INPUT);

  // set pins low
  digitalWrite(pin_IRSensor_CLK, LOW);
  digitalWrite(pin_IRSensor_SDO, LOW);
  digitalWrite(pin_IRSensor_Latch, LOW);

  for (;;) // A Task shall never return or exit.
  {
    // Read background value
    backgroundValue = analogRead(pin_IRSensor_Value);
    
    // Datenpin high setzen
    digitalWrite(pin_IRSensor_SDO, HIGH);
    delayMicroseconds(ShiftOutUs);
    digitalWrite(pin_IRSensor_CLK, HIGH);
    delayMicroseconds(ShiftOutUs);
    digitalWrite(pin_IRSensor_CLK, LOW);
    digitalWrite(pin_IRSensor_Latch, HIGH);
    delayMicroseconds(ShiftOutUs);
    digitalWrite(pin_IRSensor_Latch, LOW);
    digitalWrite(pin_IRSensor_SDO, LOW);

    for (int i = 0; i < IRSensorChannels; i++)
    {
      // Samplezeit warten
      vTaskDelay(IRMeasurementTimePerSlot / portTICK_PERIOD_MS);
  
      // Messwert lesen
      sensorValue = analogRead(pin_IRSensor_Value);
      slotEmpty[i] = (sensorValue - backgroundValue) < EmptySlotTreshold;

      // Nächster Kanal
      digitalWrite(pin_IRSensor_CLK, HIGH);
      delayMicroseconds(ShiftOutUs);
      digitalWrite(pin_IRSensor_CLK, LOW);
      digitalWrite(pin_IRSensor_Latch, HIGH);
      delayMicroseconds(ShiftOutUs);
      digitalWrite(pin_IRSensor_Latch, LOW);
    }

    /*for (int i = 0; i < 16; i++)
    {
      Serial.print(sensorValues[i] ? "1" : "0");
    }
    Serial.println();*/
    
    vTaskDelay(IRSensorInterval / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskWS2812(void *pvParameters)
{
  (void) pvParameters;

  pinMode(pin_WS2812, OUTPUT);

  FastLED.addLeds<WS2812B,pin_WS2812>(leds, numLEDs).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(ledBrightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,ledMaxCurrent); 
  uint8_t startingHue = 0;

  for (;;)
  {
    fill_rainbow(leds, 
                 numLEDs/*led count*/, 
                 startingHue /*starting hue*/);

    for (int i = 0; i < sizeof(slotLEDIndices)/sizeof(slotLEDIndices[0]); i++)
    {
      for (int j = 0; j < numSlotStatusLEDs; j++)
      {
        leds[slotLEDIndices[i]+j] = slotEmpty[i] ? CRGB::Red : CRGB::Green;
      }
    }

    FastLED.show();
    startingHue++;

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskWiFi(void *pvParameters)
{
  (void) pvParameters;
  char buf[32];
  bool firstRun = false;
  bool previousSlotValues[10] = { false };

  // Connect to Wifi
  WiFi.begin(wifiSSID, wifiPSK);
  while (WiFi.status() != WL_CONNECTED)
    vTaskDelay(500 / portTICK_PERIOD_MS);
  Serial.print("WiFi connected: ");
  Serial.println(WiFi.localIP());

  // Initialize MQTT Client
  mqttClient.setServer(MQTT_Server, MQTT_Port);

  for (;;)
  {
    // reconnect if not connected
    while (!mqttClient.connected()) {
      Serial.println("Reconnecting MQTT...");
      if (mqttClient.connect("ESP32Client")) {
        //mqttClient.subscribe("esp32/output");
      } else {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    }

    // publish values if changed
    for (int i = 0; i < 10; i++)
    {
      if ((firstRun || (slotEmpty[i] != previousSlotValues[i])) &&
        snprintf(buf, sizeof(buf)/sizeof(char), "elektronikautomat/slot%dEmpty", i+1))
      {
        mqttClient.publish(buf, slotEmpty[i] ? "1" : "0");
        previousSlotValues[i] = slotEmpty[i];
      }
    }
    firstRun = false;

    vTaskDelay(MQTT_PushInterval / portTICK_PERIOD_MS);
  }
}

void TaskDiagnostics(void *pvParameters)
{
  (void) pvParameters;
  char buf[1024];

  for (;;)
  {
    Serial.print("Task Highwater Marks: TaskIRSensors: ");
    Serial.print(uxTaskGetStackHighWaterMark(hTaskIRSensors));
    Serial.print(", TaskWS2812: ");
    Serial.print(uxTaskGetStackHighWaterMark(hTaskWS2812));
    Serial.print(", TaskWiFi: ");
    Serial.print(uxTaskGetStackHighWaterMark(hTaskWiFi));
    Serial.print(", Diagnostics: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}
