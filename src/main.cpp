/*
Copyright Gabriele Benati


*/
#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <TinyGPS++.h>


#define WIFI_SSID "VODAFONE-C946"
#define WIFI_PASSWORD "GxKKbsGA3csMmdbL" 

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 18)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "esp/bme680/temperature"
#define MQTT_PUB_HUM  "esp/bme680/humidity"
#define MQTT_PUB_PRES "esp/bme680/pressure"
#define MQTT_PUB_GAS  "esp/bme680/gas"
#define MQTT_PUB_TIME "esp/bme680/time"
#define MQTT_PUB_LAT  "esp/gps/latitude"
#define MQTT_PUB_LONG "esp/gps/longitude"

#define RXD2 16
#define TXD2 17

/*#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Variables to hold sensor readings
float temperature;
float humidity;
float pressure;
float gasResistance;

RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t packetIdPub1;
RTC_DATA_ATTR uint16_t packetIdPub2;
RTC_DATA_ATTR uint16_t packetIdPub3;
RTC_DATA_ATTR uint16_t packetIdPub4;
RTC_DATA_ATTR uint16_t packetIdPub5;
RTC_DATA_ATTR uint16_t packetIdPub6;
RTC_DATA_ATTR uint16_t packetIdPub7;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TinyGPSPlus gps;
double latitude = 0;
double longitude = 0;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

// put function declarations here:

void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading BME680"));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading BME680"));
    return;
  }
  temperature = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
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
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

#define GPS_BAUDRATE 9600

void GPS_Read(void) 
{  
  char received;
  Serial.println("Reading GPS");
  while (Serial2.available() > 0) 
  { 
    received = (char)Serial2.read();
    Serial.print(received);
    if (gps.encode(received)) 
    {
      if (gps.location.isValid()) 
      {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat());
        latitude = gps.location.lat();

        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng());
        longitude = gps.location.lng();

        Serial.print(F("- altitude: "));
        if (gps.altitude.isValid())
          Serial.println(gps.altitude.meters());
        else
          Serial.println(F("INVALID"));
      } else {
        Serial.println(F("- location: INVALID"));
      }

      Serial.print(F("- speed: "));
      if (gps.speed.isValid()) {
        Serial.print(gps.speed.kmph());
        Serial.println(F(" km/h"));
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F(" - Date: INVALID"));
      }

      Serial.println();
    } 
    #if 0
    else
    {
      Serial.println(F("GPS serial encode failed"));
     

    }
    #endif
  }
  #if 0
  else /*if (millis() > 5000 && gps.charsProcessed() < 10)*/
  {
    Serial.println(F("No GPS data received: check wiring"));
  }
  #endif
  Serial.println("\nEnd reading GPS");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("Initialised gps MODULE"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    Serial.println(F("Continuing anyway"));
    //while (1);
  }
  
  Serial.println("Number of boots");
  Serial.println(bootCount);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(20000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  esp_sleep_enable_timer_wakeup(300000000ULL);
  Serial.printf("Going to sleep every 300 seconds \n");

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("gbenati", "Hotelier%65");
  connectToWifi();
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() 
{

  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) 
  {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    GPS_Read() ;
    getBME680Readings();
    Serial.println();
    Serial.printf("Temperature = %.2f ºC \n", temperature);
    Serial.printf("Humidity = %.2f % \n", humidity);
    Serial.printf("Pressure = %.2f hPa \n", pressure);
    Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);
    
    // Publish an MQTT message on topic esp/bme680/temperature
    packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature);

    // Publish an MQTT message on topic esp/bme680/humidity
    packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity);

    // Publish an MQTT message on topic esp/bme680/pressure
    packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(pressure).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PRES, packetIdPub3);
    Serial.printf("Message: %.2f \n", pressure);

    // Publish an MQTT message on topic esp/bme680/gas
    packetIdPub4 = mqttClient.publish(MQTT_PUB_GAS, 1, true, String(gasResistance).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_GAS, packetIdPub4);
    Serial.printf("Message: %.2f \n", gasResistance);

    // Publish an MQTT message on topic esp/bme680/time
    packetIdPub5 = mqttClient.publish(MQTT_PUB_TIME, 1, true, String(bootCount).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TIME, packetIdPub5);
    Serial.printf("Message: %i \n", bootCount++);

    // Publish an MQTT message on topic esp/gps/latitude
    packetIdPub6 = mqttClient.publish(MQTT_PUB_LAT, 1, true, String(latitude).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LAT, packetIdPub6);
    Serial.printf("Message: %f \n", latitude);

    // Publish an MQTT message on topic esp/gps/longitude
    packetIdPub7 = mqttClient.publish(MQTT_PUB_LONG, 1, true, String(longitude).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LONG, packetIdPub7);
    Serial.printf("Message: %f \n", longitude);

    delay(1000);
   

    delay(5000);
    Serial.println("Going to sleep now");
   
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
  #if 0
  while (Serial2.available() > 0)
  {
    char gpsData = Serial2.read();
    Serial.print(gpsData);
  }
  delay(2000);
  Serial.println("loop");
  #endif
}

