/*
  --------------------------------
  ESPConnect - AutoConnect Example
  --------------------------------

  Initializes ESPConnect and attaches itself to AsyncWebServer.

  Github & WiKi: https://github.com/ayushsharma82/ESPConnect
  Works with both ESP8266 & ESP32
*/
#define MQTT_CLIENTID "esp32-mqtt-deepsleep"
#define MQTT_HOST "192.168.0.23"
#define MQTT_PORT 1883

#undef MQTT_USEAUTH
#define MQTT_USER "pi"
#define MQTT_PASSWORD "xyzzy13"

#include <Arduino.h>
#if defined(ESP8266)
/* ESP8266 Dependencies */
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#elif defined(ESP32)
/* ESP32 Dependencies */
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif

#include <ESPConnect.h>
#include <Adafruit_BME280.h> /// for the sensors
#include <PubSubClient.h>
String clientId;
WiFiClient wifiClient;

PubSubClient client(wifiClient);
time_t startTime = millis();
struct weatherData
{
  float temperature;
  float pressure;
  float humidity;
};
WiFiClient espClient;
PubSubClient mqttClient(espClient);

AsyncWebServer server(80);

struct weatherData readSensor(uint8_t bmeAddress = 0x76)
{
  Adafruit_BME280 bme; // I2C

  while (!bme.begin(bmeAddress))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1000);
  }

  struct weatherData measurement;
  // So if you take the pascal value of say 100734 and divide by 3386.39 you'll get 29.72 inches-Hg.
  // value = temp *9/5 + 32;//convert to Fahrenheit
  measurement.temperature = bme.readTemperature() * 9 / 5 + 32;
  measurement.pressure = bme.readPressure() / 3386.39;
  measurement.humidity = bme.readHumidity();
  return measurement;
}

void printValues(struct weatherData dataToPrint)
{
  Serial.print("Temperature = ");
  Serial.print(dataToPrint.temperature);
  Serial.println(" *F");

  Serial.print("Pressure = ");
  Serial.print(dataToPrint.pressure);
  Serial.println(" Inch Hg");
  Serial.print("Humidity = ");
  Serial.print(dataToPrint.humidity);
  Serial.println(" %");

  Serial.println();
}
void setup()
{
  Serial.begin(115200);
  Serial.println();

  /*
    AutoConnect AP
    Configure SSID and password for Captive Portal
  */
  ESPConnect.autoConnect("ESPConfig");

  /*
    Begin connecting to previous WiFi
    or start autoConnect AP if unable to connect
  */
  if (ESPConnect.begin(&server))
  {
    Serial.println("Connected to WiFi");
    Serial.println("IPAddress: " + WiFi.localIP().toString());
  }
  else
  {
    Serial.println("Failed to connect to WiFi");
  }
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
}
// the wifi connection is WiFi and is connected at this point????

void loop()
{
  Serial.println("Start loop()");
  // Loop until we're reconnected

  while (!mqttClient.connected())
  {
    Serial.print(".");
    if (WiFi.isConnected())
    {
      Serial.println("WiFi is Connected");
    }
    else
    {
      Serial.println("WiFi not Connected");
    }

    

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    if (mqttClient.connect(MQTT_CLIENTID))//, MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println("MQTT connected.");
    }
    else{
      Serial.println("MQTT not connected.");

    }
    delay(1000);
  }
  delay(5000);
}