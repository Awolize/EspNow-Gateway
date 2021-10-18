#include <Arduino.h>
#include "PubSubClient.h"
#include "SoftwareSerial.h"
#include "ArduinoJson.h"
#include "time.h"

#include "secrets.h"

#ifdef ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

// Uncomment to view debug prints
#define DEBUG_FLAG

// WIFI Settings
#define WIFI_SSID SECRET_SSID
#define WIFI_PASSWORD SECRET_PASSWORD

// MQTT
#define mqttServer IPAddress(192, 168, 1, 50)
#define mqttPort 1883

#define MSG_TOPIC_SIZE 50   // not sure
#define MSG_BUFFER_SIZE 256 // avail: 0 -> 256 gotta match esp-now receiver
char mqttTopic[MSG_TOPIC_SIZE];
char mqtt_msg[MSG_BUFFER_SIZE];

// Serial
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_RX_PIN 14 // D5
#define SERIAL_TX_PIN 12 // D6
char end_char = '\0';

// Initialization
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
StaticJsonDocument<512> doc;                          // larger than it needs to be (Serial buffer size)
SoftwareSerial SSerial(SERIAL_RX_PIN, SERIAL_TX_PIN); // D5 and D6 on Wemos D1 mini

/* ############################ MQTT ############################################# */

void onMqttMessage(char *topic, byte *payload, unsigned int length)
{
    // currently not listening to anything
}

void mqttReconnect()
{

    if (!mqttClient.connected())
    {
        mqttClient.setServer(mqttServer, mqttPort);
        mqttClient.setCallback(onMqttMessage);
        if (mqttClient.connect("ESP-Now Sensors"))
        {
            mqttClient.subscribe("cmnd/sensor");
            mqttClient.publish("ESP-Now/sensor/status", "online");
#ifdef DEBUG_FLAG
            Serial.print('\n');
            Serial.println(F("ESP-Now/sensor/status  online"));
            Serial.println();
            Serial.println(F("Connected to MQTT"));
#endif
        }
    }
}

void mqttPublish(char macAdress[], char topic[], String payload)
{

    strcpy(mqttTopic, "ESP-Now/sensor_");
    strcat(mqttTopic, macAdress);
    strcat(mqttTopic, "/");
    strcat(mqttTopic, topic);
#ifdef DEBUG_FLAG
    Serial.print("mqttTopic: ");
    Serial.println(mqttTopic);
#endif
    mqttClient.publish(mqttTopic, payload.c_str());
}

void sensorMessageReceived()
{
    digitalWrite(LED_BUILTIN, LOW);

    /*
    JsonObject dataJson = doc["data"];
    dataJson["state"] = doc["moisture"];
    dataJson.remove("moisture");
    dataJson["uptime"] = doc["uptime"];
    doc["data"] = dataJson;*/

    String payload;
    serializeJson(doc["data"], payload);
    String macAddr = doc["mac"];
    mqttPublish((char *)macAddr.c_str(), "status", payload);

#ifdef DEBUG_FLAG
    Serial.print("Sending payload: ");
    Serial.println(payload.c_str());
#endif
    digitalWrite(LED_BUILTIN, HIGH);
}

/* ##### WifiManager ##### */

void wifiConnected()
{
    mqttReconnect();
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG_FLAG
    Serial.println(F("\nConnection established!"));
    Serial.print(F("IP address:\t"));
    Serial.println(WiFi.localIP());
    Serial.println(F("\nGateway ready!"));
#endif
}

/* ############################ Setup ############################################# */

void setup()
{
    Serial.begin(115200);
    SSerial.begin(9600);
#ifdef DEBUG_FLAG
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F("  ESP-Now Sensors"));
    Serial.println(F("=============================="));
    Serial.println();
#endif

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    Serial.println(" ...");

    int i = 0;
    while (WiFi.status() != WL_CONNECTED)
    { // Wait for the Wi-Fi to connect
        delay(1000);
        Serial.print(++i);
        Serial.print(' ');
    }

    wifiConnected();
}

void loop()
{
    mqttReconnect();
    mqttClient.loop();

    if (SSerial.available())
    {
        char data[SERIAL_BUFFER_SIZE];
        SSerial.readBytesUntil(end_char, data, SERIAL_BUFFER_SIZE);

        // Decode message
        char *buffer = (char *)data;
        String msg_raw = String(buffer);

        deserializeJson(doc, msg_raw);
        sensorMessageReceived();
    }
}