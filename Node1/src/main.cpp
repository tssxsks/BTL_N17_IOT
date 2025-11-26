// NODE 1: SENSOR → MQTT JSON

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

// WiFi
const char* ssid     = "TANG 2";
const char* password = "123456789@";

// MQTT broker
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Sensors 
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define MQ135_PIN 34
#define LDR_PIN   35

// Hiệu chuẩn MQ135 (tuyến tính)
const float MQ_RAW1 = 850.0f;
const float MQ_AQI1 = 38.0f;
const float MQ_RAW2 = 1100.0f;
const float MQ_AQI2 = 120.0f;

const float MQ_AQI_M = (MQ_AQI2 - MQ_AQI1) / (MQ_RAW2 - MQ_RAW1);
const float MQ_AQI_B = MQ_AQI1 - MQ_AQI_M * MQ_RAW1;

unsigned long lastSend = 0;

// LDR -> % sáng
float getLightPercent() {
  int raw = analogRead(LDR_PIN);
  float darkness = (raw / 4095.0f) * 100.0f;
  return 100.0f - darkness;
}

// MQ135 -> AQI tuyến tính
float getAQI() {
  int mqRaw = analogRead(MQ135_PIN);
  float aqi = MQ_AQI_M * mqRaw + MQ_AQI_B;
  if (aqi < 0) aqi = 0;
  if (aqi > 500) aqi = 500;
  return aqi;
}

// MQTT reconnect
void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("MQTT reconnect...");
    String clientId = "ESP32-NODE1-";
    clientId += String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("OK");
    } else {
      Serial.print("Fail rc=");
      Serial.print(mqtt.state());
      Serial.println(" → retry");
      delay(2000);
    }
  }
}

// SETUP
void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");

  mqtt.setServer(mqtt_server, 1883);

  dht.begin();
  analogReadResolution(12);

  Serial.println("Node 1 Started...");
}

// LOOP
void loop() {

  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  if (millis() - lastSend > 3000) {
    lastSend = millis();

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float light = getLightPercent();
    float aqi   = getAQI();

    // JSON Payload
    String payload = "{";
    payload += "\"temperature\":" + String(t) + ",";
    payload += "\"humidity\":"    + String(h) + ",";
    payload += "\"light\":"       + String(light) + ",";
    payload += "\"aqi\":"         + String(aqi);
    payload += "}";

    mqtt.publish("home/nhom17/data", payload.c_str());

    Serial.println("MQTT → " + payload);
  }
  delay(4000);
}
