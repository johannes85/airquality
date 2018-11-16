#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <pms.h>

#include "settings.h"

// Network
const char* ssid = C_WIFI_SSID;
const char* password = C_WIFI_PASSWORD;
WiFiClient espClient;

// MQTT
const char* mqttServerHost = C_MQTT_HOST;
const char* mqttUsername = C_MQTT_USERNAME;
const char* mqttPassword = C_MQTT_PASSWORD;
PubSubClient mqttClient(espClient);
long lastMqttMsg= 0;
char mqttMsgBuffer[120];
boolean mqttResult;

// BME280
#define BME_I2C_ADDR 0x76
#define BME_SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
float bmeTemp= 0;
float bmeHum= 0;
float bmePres= 0;
long lastCCSSensorRead= 0;

// CCS811
Adafruit_CCS811 ccs;
uint16_t ccsCO2= 0;
uint16_t ccsTVOC= 0;
long lastBMESensorRead= 0;

// PMS
Pmsx003 pms(D5, D6);
uint16_t pms1= 0;
uint16_t pms25= 0;
uint16_t pms10= 0;
long lastPMSSensorOutput= 0;

void connectMQTT() {
  Serial.print("Attempting MQTT connection...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str(), mqttUsername, mqttPassword)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}

void connectWiFi(bool reconnect) {
  bool successful = false;
  bool waitForConnect;
  uint8_t status;
  do {
    if (reconnect) {
      Serial.println("Reconnecting to WLAN...");
      WiFi.reconnect();
    } else {
      Serial.println("Connecting to WLAN...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
    }

    waitForConnect= true;
    do {
      status= WiFi.status();
      switch (status) {
        case WL_NO_SSID_AVAIL:
          Serial.println("Could not found ssid, waiting and retrying...");
          successful= false;
          waitForConnect= false;
          delay(5000);
          break;
        case WL_CONNECT_FAILED:
          Serial.println("Connection failed, waiting and retrying...");
          successful= false;
          waitForConnect= false;
          delay(5000);
          break;
        case WL_CONNECTED:
          successful= true;
          waitForConnect= false;
          break;
        default:
          Serial.print('.');
          delay(500);
          break;
      }
    } while (waitForConnect);
  } while (!successful);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  bool initialized;
  delay(10);
  Serial.begin(9600);

  // Init Ethernet
  connectWiFi(false);

  // Init MQTT
  mqttClient.setServer(mqttServerHost, C_MQTT_PORT);
  connectMQTT();

  // Init sensors 
  do {
    delay(1000);
    initialized= true;
    
    if (!bme.begin(BME_I2C_ADDR)) {
      Serial.println("Sensor error: BME280"); 
      initialized= false;
    }

    if(!ccs.begin()){
      Serial.println("Sensor error: CCS811"); 
      initialized= false;
    }

    pms.begin();
    pms.waitForData(Pmsx003::wakeupTime);
    pms.write(Pmsx003::cmdModeActive);
  } while (initialized == false);

  // Wait for sensors
  do {
    delay(1000);
    initialized= true;

    if(!ccs.available()){
      Serial.println("Waiting for sensor: CCS811"); 
      initialized= false;
    }
  } while (initialized == false);

  // Configure sensors (recommended settings for weather monitoring)
  bme.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1, // temperature
    Adafruit_BME280::SAMPLING_X1, // pressure
    Adafruit_BME280::SAMPLING_X1, // humidity
    Adafruit_BME280::FILTER_OFF  
  );
}

void loop() {
  long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(true);
  }
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  // BME280 read interval = 30sek (recommended 60sek for weather monitoring)
  if (now - lastBMESensorRead > 30000 || lastBMESensorRead == 0) {
    lastBMESensorRead = now;
    
    bme.takeForcedMeasurement();
    bmeTemp= bme.readTemperature() - C_BME280_TEMP_OFFSET;
    bmePres= bme.readPressure() / 100.0F;
    bmeHum= bme.readHumidity();
  
    Serial.println("BME280");
    Serial.print("Temperature = ");
    Serial.print(bmeTemp);
    Serial.print(" *C");
    
    Serial.print(" Pressure = ");
    Serial.print(bmePres);
    Serial.print(" hPa");
    
    Serial.print(" Humidity = ");
    Serial.print(bmeHum);
    Serial.println(" %");
  }

  // CCS811
  if (now - lastCCSSensorRead > 3000 || lastCCSSensorRead == 0) {
    lastCCSSensorRead = now;
  
    ccs.setEnvironmentalData((int)bmeHum, bmeTemp);
    Serial.println("CCS811");
    if (ccs.available()) {
      if(!ccs.readData()){
        ccsCO2= ccs.geteCO2();
        ccsTVOC= ccs.getTVOC();
        float temp = ccs.calculateTemperature();
        Serial.print("CO2: ");
        Serial.print(ccsCO2);
        Serial.print("ppm, TVOC: ");
        Serial.print(ccsTVOC);
      } else {
        Serial.print("Read error");
      }
    } else {
      Serial.print("Not ready");
    }
    Serial.println();
  }

  // PMS
  const auto n = Pmsx003::Reserved;
  Pmsx003::pmsData data[n];
	Pmsx003::PmsStatus status = pms.read(data, n);
  switch (status) {
		case Pmsx003::OK:
		{
      pms1= data[Pmsx003::PM1dot0];
      pms25= data[Pmsx003::PM2dot5];
      pms10= data[Pmsx003::PM10dot0];

      //lastPMSSensorOutput
      if (now - lastPMSSensorOutput > 3000 || lastPMSSensorOutput == 0) {
        lastPMSSensorOutput = now;
        Serial.println("PMS");
        Serial.print("PM1.0: ");
        Serial.print(pms1);
        Serial.print(" ");
        Serial.print(Pmsx003::metrics[Pmsx003::PM1dot0]);
        Serial.print(" PM2.5: ");
        Serial.print(pms25);
        Serial.print(" ");
        Serial.print(Pmsx003::metrics[Pmsx003::PM2dot5]);
        Serial.print(" PM10.0: ");
        Serial.print(pms10);
        Serial.print(" ");
        Serial.println(Pmsx003::metrics[Pmsx003::PM10dot0]);
      }
			break;
		}
		case Pmsx003::noData:
			break;
		default:
			Serial.print("PMS Error: ");
			Serial.println(Pmsx003::errorMsg[status]);
	};

  // MQTT
  if (now - lastMqttMsg > 30000 || lastMqttMsg == 0) {
    lastMqttMsg = now;

    Serial.println("Sending MQTT message 1...");
    snprintf (
      mqttMsgBuffer, 120, 
      "{\"temp\": \"%2.2f\", \"hum\": \"%2.2f\", \"pres\": \"%2.2f\"}", 
      bmeTemp,
      bmeHum,
      bmePres
    );
    Serial.println(mqttMsgBuffer);
    mqttResult = mqttClient.publish(C_MQTT_TOPIC1, mqttMsgBuffer);
    Serial.print("Result: ");
    Serial.println(mqttResult);
    Serial.print("State: ");
    Serial.println(mqttClient.state());

    Serial.println("Sending MQTT message 2...");
    snprintf (
      mqttMsgBuffer, 120, 
      "{\"co2\": \"%" PRIu16 "\", \"tvoc\": \"%" PRIu16 "\", \"pm25\": \"%" PRIu16 "\", \"pm10\": \"%" PRIu16 "\"}", 
      ccsCO2,
      ccsTVOC,
      pms25,
      pms10
    );
    Serial.println(mqttMsgBuffer);
    mqttResult = mqttClient.publish(C_MQTT_TOPIC2, mqttMsgBuffer);
    Serial.print("Result: ");
    Serial.println(mqttResult);
    Serial.print("State: ");
    Serial.println(mqttClient.state());
  }
}
