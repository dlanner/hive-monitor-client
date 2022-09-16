#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "HX711.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>

#define LOADCELL_DOUT_PIN  23
#define LOADCELL_SCK_PIN 22

#define BME_CS 26
#define BME_MOSI 18
#define BME_MISO 19
#define BME_SCK 5

#define BATTERY_VOLTAGE_PIN A13

#define SEALEVELPRESSURE_HPA (1019.8)

HX711 scale;
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

char weightString[10], temperatureString[10], humidityString[10], pressureString[10], gasResistanceString[10], batteryVoltageString[10];

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  float calibration_factor = 13097;
  float offset = 281811;
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.set_offset(offset);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  const char* ssid = "barbarian";
  const char* password = "gazebobossfight!";

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

    if (!bme.performReading()) {
    Serial.println("Failed to perform BME 680 reading");
    return;
  }

  int number_of_scale_readings = 20;
  float rawWeight = scale.get_units(number_of_scale_readings);

  float weight = fmaxf(rawWeight, 0.0); // lbs
  float temperature = (bme.temperature * 9/5) + 32; // Fahrenheit
  float pressure = bme.pressure / 3386.39; // inHg
  float humidity = bme.humidity; // Percent
  float gasResistance = bme.gas_resistance; // Ohms
  float batteryVoltage = (float(analogRead(BATTERY_VOLTAGE_PIN)) * 2) / 1000; // Volts

  dtostrf(weight, 6, 2, weightString);
  dtostrf(temperature, 6, 2, temperatureString);
  dtostrf(pressure, 6, 2, pressureString);
  dtostrf(humidity, 6, 2, humidityString);
  dtostrf(gasResistance, 6, 2, gasResistanceString);
  dtostrf(batteryVoltage, 6, 2, batteryVoltageString);

  if (WiFi.status()== WL_CONNECTED ) {
    String json;
    StaticJsonDocument<192> doc;
    doc["hiveId"] = "test";
    doc["weight"] = weightString;
    doc["temperature"] = temperatureString;
    doc["humidity"] = humidityString;
    doc["pressure"] = pressureString;
    doc["gasResistance"] = gasResistanceString;
    doc["batteryVoltage"] = batteryVoltageString;
    serializeJson(doc, json);
    Serial.print("JSON: ");
    Serial.println(json);
    HTTPClient http;
    WiFiClient client;
    String hiveMonitorServerUrl = "http://10.0.0.1:8080/data";
    http.begin(client, hiveMonitorServerUrl.c_str());
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(json);
    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String response = http.getString();
        Serial.print("Response: ");
        Serial.println(response);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
  
  scale.power_down();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush();

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_OFF);

  esp_deep_sleep_start();
}

void loop() {
}
