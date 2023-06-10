#include "config.h"

#include <Arduino.h>
#include <FastLED.h>

//#define IR_SENSOR_NOT_IMPLEMENTED

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>

// PIN Definitions
const int pin_IRSensor_SDO = 19;
const int pin_IRSensor_CLK = 18
;
const int pin_IRSensor_Latch = 5;
const int pin_IRSensor_Value = 34; // GPIO32-GPIO39 für analogeingänge
const int pin_WS2812_Front = 23;
const int pin_WS2812_Bottom = 22;

// LED Definitions
const int numLEDsFront = 127;
const int ledBrightnessFront = 100;
const int ledMaxCurrent = 4000; // max Strom in mA
const int numLEDsBottom = 123;
const int ledBrightnessBottom = 255;

const int LED_ANIMATION_FPS = 60;

// IR Sensor Definitions
const float IRSensorFilterCoeff = 0.1f;
const int IRSensorInterval = 500;
const int IRSensorChannels = 10;
const int IRMeasurementTimePerSlot = 20;
const int ShiftOutUs = 20;
const int EmptySlotTreshold = 200; // Analog value

// Slot Sensors
const int slotLEDIndices[IRSensorChannels] = { 59, 55, 51, 47, 43, 39, 35, 31, 27, 23 };
const int numSlotStatusLEDs = 4;
volatile bool slotEmpty[IRSensorChannels] = { false };
float IRSensorState[IRSensorChannels] = { 0.0f };

CRGB ledsFront[numLEDsFront];
CRGB ledsBottom[numLEDsBottom];
CLEDController *frontController, *bottomController;

// Task Definitions
void TaskIRSensors( void *pvParameters );
void TaskWS2812( void *pvParameters );
//void TaskWiFi( void *pvParameters );
void TaskDiagnostics( void *pvParameters );

// Task Handles
TaskHandle_t hTaskIRSensors, hTaskWS2812, hTaskWiFi;

// MQTT Classes
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient wifiClient;
PubSubClient mqttClient(MQTT_Server, MQTT_Port, callback, wifiClient);

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

#include "animations.h" // TODO: better implementation

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Initialize LEDS
  pinMode(pin_WS2812_Front, OUTPUT);
  pinMode(pin_WS2812_Bottom, OUTPUT);
  frontController = &FastLED.addLeds<WS2812B,pin_WS2812_Front,GRB>(ledsFront, numLEDsFront).setCorrection(TypicalLEDStrip);
  bottomController = &FastLED.addLeds<WS2812B,pin_WS2812_Bottom,GRB>(ledsBottom, numLEDsBottom).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,ledMaxCurrent);

  // Task Setup
  xTaskCreate(
    TaskIRSensors
    ,  "TaskIRSensors"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  1 // Priority
    ,  &hTaskIRSensors); // Handle

  xTaskCreate(
    TaskWS2812
    ,  "TaskWS2812"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  2 // Priority
    ,  &hTaskWS2812); // Handle

  xTaskCreate(
    TaskWiFi
    ,  "TaskWiFi"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL // priority
    ,  2 // Priority
    ,  &hTaskWiFi); // Handle

  xTaskCreate(
    TaskDiagnostics
    ,  "TaskDiagnostics"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
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

    // Debugdaten
    Serial.print("Background: ");
    Serial.print(backgroundValue);
    Serial.print(" Sensors: ");

    for (int i = 0; i < 5; i++) // TODO: Fix
    {
      // Samplezeit warten
      vTaskDelay(IRMeasurementTimePerSlot / portTICK_PERIOD_MS);
  
      // Messwert lesen
      sensorValue = analogRead(pin_IRSensor_Value);
      IRSensorState[i] = IRSensorState[i] * (1.0f - IRSensorFilterCoeff) + IRSensorFilterCoeff * (sensorValue - backgroundValue);
      slotEmpty[i] = IRSensorState[i] < EmptySlotTreshold;

      // nochmal warten
      vTaskDelay(IRMeasurementTimePerSlot / portTICK_PERIOD_MS);

      // Nächster Kanal
      digitalWrite(pin_IRSensor_CLK, HIGH);
      delayMicroseconds(ShiftOutUs);
      digitalWrite(pin_IRSensor_CLK, LOW);
      digitalWrite(pin_IRSensor_Latch, HIGH);
      delayMicroseconds(ShiftOutUs);
      digitalWrite(pin_IRSensor_Latch, LOW);

      // Debugdaten
      Serial.print(IRSensorState[i]);
      Serial.print(", ");
    }

    slotEmpty[5] = false;
    slotEmpty[6] = false;
    slotEmpty[7] = true;
    slotEmpty[8] = true;
    slotEmpty[9] = true;

    
    /*Serial.print(" Digital: ");
    for (int i = 0; i < IRSensorChannels; i++)
    {
      Serial.print(slotEmpty[i] ? "1" : "0");
    }*/
    Serial.println();
    
    vTaskDelay(IRSensorInterval / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskWS2812(void *pvParameters)
{
  (void) pvParameters;
  uint8_t startingHue = 0;

  for (;;)
  {
    fill_rainbow(ledsFront, 
                 numLEDsFront/*led count*/, 
                 startingHue /*starting hue*/);

    // Bottom Fire animation
    Fire();

    
    for (int i = 0; i < sizeof(slotLEDIndices)/sizeof(slotLEDIndices[0]); i++)
    {
      for (int j = 0; j < numSlotStatusLEDs; j++)
      {
#ifdef IR_SENSOR_NOT_IMPLEMENTED
        ledsFront[slotLEDIndices[i]+j] = CRGB::White;
#else
        ledsFront[slotLEDIndices[i]+j] = (slotEmpty[i] ? CRGB::Red : CRGB::White);
#endif
      }
    }

    frontController->showLeds(ledBrightnessFront);
    bottomController->showLeds(ledBrightnessBottom);

    startingHue++;

    vTaskDelay(1000 / LED_ANIMATION_FPS / portTICK_PERIOD_MS);
  }
}


void TaskWiFi(void *pvParameters)
{
  (void) pvParameters;
  char buf[32];
  bool firstRun = false;
  bool previousSlotValues[IRSensorChannels] = { false };

  // Connect to Wifi
  WiFi.begin(wifiSSID, wifiPSK);
  while (WiFi.status() != WL_CONNECTED)
    vTaskDelay(500 / portTICK_PERIOD_MS);
  Serial.print("WiFi connected: ");
  Serial.println(WiFi.localIP());


  for (;;)
  {
    // reconnect if not connected
    while (!mqttClient.connected()) {
      Serial.println("Reconnecting MQTT...");
      if (mqttClient.connect("esp32", MQTT_User, MQTT_Pass)) {
        //mqttClient.subscribe("esp32/output");
        Serial.println("MQTT connected!");
      } else {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    }

    // publish values if changed
    for (int i = 0; i < IRSensorChannels; i++)
    {
      if ((firstRun || (slotEmpty[i] != previousSlotValues[i])) &&
        snprintf(buf, sizeof(buf)/sizeof(char), "elektronikautomat/slot%dEmpty", i+1))
      {
        Serial.print("Publish to: ");
        Serial.println(buf);
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

  for (;;)
  {
    Serial.print("Task Highwater Marks: ");
    Serial.print("TaskIRSensors: ");
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
