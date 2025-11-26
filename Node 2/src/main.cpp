// NODE 2 – MQTT + LCD + FAN + THINGSBOARD

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// WiFi
const char* ssid = "TANG 2";
const char* password = "123456789@";

// Mosquitto
const char* mosq_host = "test.mosquitto.org";
const int   mosq_port = 1883;

// ThingsBoard
const char* tb_host  = "mqtt.eu.thingsboard.cloud";
const int   tb_port  = 1883;
const char* tb_token = "CUk5ru2wTngU9aGEcAj0";

// MQTT Client cho Mosquitto
WiFiClient espMosq;
PubSubClient mosq(espMosq);

// MQTT Client cho ThingsBoard
WiFiClient espTB;
PubSubClient tb(espTB);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// FAN
#define FAN_PIN 26

// Sensor Values
float g_temp  = 0;
float g_hum   = 0;
float g_light = 0;
float g_aqi   = 0;

// Control
bool fanState = false;
bool autoMode = false;

// APPLY FAN LOGIC
void applyFanLogic() {

    bool warnTemp = (g_temp >= 27.0f);
    bool warnAQI  = (g_aqi  >= 120.0f);

    if (autoMode) {
        fanState = (warnTemp || warnAQI);
    }

    digitalWrite(FAN_PIN, fanState);
}

// UPDATE LCD
void updateLCD() {

    static unsigned long lastSwitch = 0;
    static bool phase = false;

    lcd.clear();

    bool warnTemp = (g_temp >= 27.0f);
    bool warnAQI  = (g_aqi  >= 120.0f);

    //  CẢ 2 CẢNH BÁO
    if (warnTemp && warnAQI) {

        lcd.setCursor(0, 0);
        lcd.print("TEMP & AQI CAO!");

        lcd.setCursor(0, 1);
        lcd.printf("T:%.1f AQI:%.0f", g_temp, g_aqi);
        return;
    }

    //  1 CẢNH BÁO
    if (warnTemp || warnAQI) {

        lcd.setCursor(0, 0);
        if (warnTemp) lcd.print("NHIET DO CAO!");
        else          lcd.print("AQI CAO!");

        lcd.setCursor(0, 1);
        if (warnTemp)
            lcd.printf("T:%.1f H:%.1f", g_temp, g_hum);
        else
            lcd.printf("AQI:%.0f", g_aqi);

        return;
    }

    //  KHÔNG CẢNH BÁO
    lcd.setCursor(0, 0);
    lcd.printf("T:%.1f H:%.1f", g_temp, g_hum);

    lcd.setCursor(0, 1);
    lcd.printf("L:%.0f%% AQI:%.0f", g_light, g_aqi);
}

// SEND TELEMETRY
void sendTelemetry() {

    String payload = "{";
    payload += "\"temperature\":" + String(g_temp) + ",";
    payload += "\"humidity\":" + String(g_hum) + ",";
    payload += "\"light\":" + String(g_light) + ",";
    payload += "\"aqi\":" + String(g_aqi) + ",";
    payload += "\"fan\":" + String(fanState ? "true" : "false") + ",";
    payload += "\"mode\":" + String(autoMode ? "true" : "false");
    payload += "}";

    tb.publish("v1/devices/me/telemetry", payload.c_str());
    Serial.println("TB ← " + payload);
}

// RPC CALLBACK (ThingsBoard)
void mqttCallback_tb(char* topic, byte* payload, unsigned int len) {

    String msg = "";
    for (int i = 0; i < len; i++) msg += (char)payload[i];

    Serial.println("RPC → " + msg);

    if (msg.indexOf("\"method\":\"FAN\"") != -1) {
        if (!autoMode) {
            fanState = msg.indexOf("true") != -1;
            digitalWrite(FAN_PIN, fanState);
        }
    }

    if (msg.indexOf("\"method\":\"MODE\"") != -1) {
        autoMode = msg.indexOf("true") != -1;
        applyFanLogic();
    }

    updateLCD();
}

// MOSQUITTO CALLBACK (DATA TỪ NODE 1)
void mqttCallback_mosq(char* topic, byte* payload, unsigned int len) {

    String json = "";
    for (int i = 0; i < len; i++) json += (char)payload[i];

    Serial.println("MQTT(JSON) ← " + json);

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.print("JSON ERROR: ");
        Serial.println(err.c_str());
        return;
    }

    g_temp  = doc["temperature"];
    g_hum   = doc["humidity"];
    g_light = doc["light"];
    g_aqi   = doc["aqi"];

    applyFanLogic();
    updateLCD();
    sendTelemetry();
}

// Mosquitto reconnect
void mosqReconnect() {
    while (!mosq.connected()) {
        Serial.print("Mosquitto reconnect...");
        if (mosq.connect("Node2-MOSQ")) {
            Serial.println("OK");
            mosq.subscribe("home/nhom17/data");
        } else {
            Serial.print("Fail rc=");
            Serial.println(mosq.state());
            delay(2000);
        }
    }
}

// ThingsBoard reconnect
void tbReconnect() {
    while (!tb.connected()) {
        Serial.print("TB reconnect...");
        if (tb.connect("Node2-TB", tb_token, NULL)) {
            Serial.println("OK");
            tb.subscribe("v1/devices/me/rpc/request/+");
        } else {
            Serial.print("Fail rc=");
            Serial.println(tb.state());
            delay(2000);
        }
    }
}

// SETUP
void setup() {
    Serial.begin(9600);

    pinMode(FAN_PIN, OUTPUT);

    Wire.begin(21,22);
    lcd.init();
    lcd.backlight();
    lcd.print("Node 2 Starting...");
    delay(1000);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(200);

    // Mosquitto (Nhận JSON từ Node 1)
    mosq.setServer(mosq_host, mosq_port);
    mosq.setCallback(mqttCallback_mosq);

    // ThingsBoard (RPC + Telemetry)
    tb.setServer(tb_host, tb_port);
    tb.setCallback(mqttCallback_tb);

    lcd.clear();
    lcd.print("Ready");
}

// LOOP
void loop() {

    if (!mosq.connected()) mosqReconnect();
    mosq.loop();

    if (!tb.connected()) tbReconnect();
    tb.loop();
}