
/* Soil Moisture Sensor for OpenHAB2

  GPLv3

  based on
  http://www.qsl.net/v/ve2cuz//garden/
  https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/


  ///////// Pin Assigment ///////

  A0  Input Soil Moisture and Battery
  GPIO4   SDA for tmp112
  GPIO5   SCL for tmp112
  GPIO12  Button S1 (Active Access Point for Config Sensor)
  GPIO13  LED
  GPIO14  Clock output for soil moisture sensor
  GPIO15  SWITCH for measuring Soil Moisture or Battery Voltage

  /////////////////////////////////////////
*/

#include <ESP8266WiFi.h>
#include <MQTT.h>

#include <Wire.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

const int FW_VERSION = 13;
String fwUrlBase = "http://shibuya:8080/static/";
const char* mqttBroker = "shibuya";

// I2C address for temperature sensor
const int TMP_ADDR  = 0x48;

uint64_t sleepTime = 1800e6;

float temp;

String ssid = "cputoasters";
String pass = "kaeferkaefer";

// Soil Moisture Sensor
const int PIN_CLK   = 14; // or D5
const int PIN_SOIL  = A0;
int Soil = 0;
int soil_hum = 0;
int SoilTrig;
int SoilValue;

// Push Button
const int ConfigPin = 12; // D6
const int Led = 13; // D7

// Electronic Switch Soil-Moisture & Battery Voltage
const int Switch = 15; // D8
int batt; // Numeric Value
int Batt; // % Value

WiFiClient wifiClient;
HTTPClient httpClient;
MQTTClient mqttClient(256);

void setup() {

  Serial.begin(115200);
  Serial.println("starting setup");

  // I2C BUS
  Wire.begin();

  // PIN I/O Setup
  pinMode(Led, OUTPUT);
  digitalWrite(Led, HIGH);

  pinMode(ConfigPin, INPUT_PULLUP);

  pinMode (Switch, OUTPUT); // LOW=Battery Voltage , HIGH=Soil Moisture
  digitalWrite (Switch, HIGH); // Soil-Moisture Selected

  // Soil Moisture
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_SOIL, INPUT);
  analogWriteFreq(40000);
  analogWrite(PIN_CLK, 400);
  delay(500);

  // device address is specified in datasheet
  Wire.beginTransmission(TMP_ADDR); // transmit to device #44 (0x2c)
  Wire.write(byte(0x01));            // sends instruction byte
  Wire.write(0x60);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting

  
  Serial.println("starting WiFi");
  WiFi.mode(WIFI_STA); // Set to Station
  delay(200);

  WiFi.begin(ssid.c_str(), pass.c_str());
  bool state = HIGH;
  int s = 0;
  while (s < 120) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("starting WiFi");
      break;
    }
    Serial.println("retry WiFi");
    delay(500);
    s++;
    state = (state == HIGH) ? LOW : HIGH;
    digitalWrite (Led, state);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("disconnecting WiFi");
    WiFi.disconnect();
    delay(1000);
    ESP.deepSleep(sleepTime, WAKE_RF_DEFAULT);
    delay(10000);
  }

  Serial.println("setup done");
}

void loop() {
  Serial.println("start main loop");
  ReadSensor();
  PutResult();
  checkForUpdates();

  WiFi.disconnect();
  Serial.println("http disconnected");
  delay(1000);
  ESP.deepSleep(sleepTime, WAKE_RF_DEFAULT);
  delay(10000);
}


void ReadSensor()
{

  Serial.println("start sensor read");

  temp = readTemperature(); // Real Temps in Celcius

  Serial.println("temp: " + String(temp));

  // Battery Voltage
  digitalWrite (Switch, LOW); // Battery Voltage Selected
  delay(200);
  batt = readBatt();
  Serial.println("batt: " + String(batt));

  // Get Soil Moisture
  delay(100);
  digitalWrite (Switch, HIGH); // Soil Moisture Selected
  delay(200);
  soil_hum = readSoilSensor();
  Serial.println("soil: " + String(soil_hum));
}

void PutResult()
{
  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("starting MQTT send");
    
    mqttClient.begin(mqttBroker, wifiClient);
    Serial.print("\nconnecting...");
    while (!mqttClient.connect("ESP8266", "openhab", "openhab")) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("\nconnected!");
    mqttClient.publish("tele/aprilbrothers-" + getMAC() + "/SENSOR", "{\"Temperature\":" + String(temp)
      + ",\"BatteryRaw\":" + batt + ",\"SoilRaw\":" + soil_hum + "}");
    mqttClient.disconnect();
    Serial.println("MQTT sent");

  }
  digitalWrite (Led, HIGH);
  
  
}

void checkForUpdates() {
  String mac = getMAC();
  String fwURL = String( fwUrlBase );
  fwURL.concat( mac );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( "Checking for firmware updates." );
  Serial.print( "MAC address: " );
  Serial.println( mac );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();

    if( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );

      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_OK");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}

float readTemperature() {

  // Begin transmission
  Wire.beginTransmission(TMP_ADDR);
  // Select Data Registers
  Wire.write(0X00);
  // End Transmission
  Wire.endTransmission();

  delay(500);

  // Request 2 bytes , Msb first
  Wire.requestFrom(TMP_ADDR, 2 );

  // Read temperature as Celsius (the default)
  while (Wire.available()) {
    int msb = Wire.read();
    int lsb = Wire.read();
    Wire.endTransmission();

    int temp = (msb * 256 + lsb) / 16;
		if(temp > 2047)
      temp -= 4096;

    return temp * 0.0625;
  }
  return -200;
}

float readSoilSensor() {
  float tmp = 0;
  float total = 0;
  float rawVal = 0;
  int sampleCount = 3;

  for (int i = 0; i < sampleCount; i++) {
    rawVal = analogRead(PIN_SOIL);
    total += rawVal;
  }

  tmp = total / sampleCount;
  return tmp;
}

float readBatt() {
  float tmp = 0;
  float total = 0;
  float rawVal = 0;
  int sampleCount = 3;

  for (int i = 0; i < sampleCount; i++) {
    rawVal = analogRead(PIN_SOIL);
    total += rawVal;
  }
  tmp = total / sampleCount;
  return tmp;
}

String getMAC()
{
  uint8_t mac[6];
  char result[14];
  WiFi.macAddress( mac );
  snprintf( result, sizeof( result ), "%02x%02x%02x%02x%02x%02x", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ] );

  return String( result );
}
