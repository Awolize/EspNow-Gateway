#include <Arduino.h>
#define MQTT_MAX_PACKET_SIZE 1024
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

#define MSG_TOPIC_SIZE 200 // not sure 50 was too short
char mqttTopic[MSG_TOPIC_SIZE];

// Serial
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_RX_PIN D5
#define SERIAL_TX_PIN D6
char end_char = '\0';

// Sensors
struct SensorData
{
    String id = ""; // mac-addr example: "B4E62D693C11",
    String name = "";
    String data = ""; // "data" : "{"battery" : 3.261719, "moisture" : 40, "uptime" : 131}"
};

// Initialization
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
SoftwareSerial SSerial(SERIAL_RX_PIN, SERIAL_TX_PIN); // D5 and D6 on Wemos D1 mini
SensorData sensors[10];
size_t numberOfSensor = 0;

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

void mqttPublishState(char macAdress[], char topic[], String payload)
{
    strcpy(mqttTopic, "ESP-Now/sensor_");
    strcat(mqttTopic, macAdress);
    strcat(mqttTopic, "/");
    strcat(mqttTopic, topic);

#ifdef DEBUG_FLAG
    Serial.println("");
    Serial.println("Sending State!");
    Serial.print("State Topic: ");
    Serial.println(mqttTopic);
    Serial.print("State Payload: ");
    Serial.println(payload.c_str());
    Serial.println("");
#endif

    mqttClient.publish(mqttTopic, payload.c_str());
}
void mqttPublishConfig(char macAdress[], char topic[], String payload)
{
    strcpy(mqttTopic, "homeassistant/sensor/espnow-");
    strcat(mqttTopic, macAdress);
    strcat(mqttTopic, "/");
    strcat(mqttTopic, topic);

#ifdef DEBUG_FLAG
    Serial.println("");
    Serial.println("Sending Config!");
    Serial.print("Config Topic: ");
    Serial.println(mqttTopic);
    Serial.print("Config Payload: ");
    Serial.println(payload.c_str());
    Serial.println("");
#endif

    mqttClient.publish(mqttTopic, payload.c_str(), true);
}

void sendMQTTDiscoveryMsg(SensorData &sensor)
{
    {
        DynamicJsonDocument doc(1024);
        doc["device_class"] = "humidity";
        doc["state_class"] = "measurement";
        //doc["icon"] = "mdi:water";
        doc["unit_of_measurement"] = "%";
        doc["name"] = "Soil Moisture";
        doc["value_template"] = "{{value_json.moisture}}";
        doc["unique_id"] = "ESP-Now-" + sensor.id + "_moisture";
        doc["state_topic"] = "ESP-Now/sensor_" + sensor.id + "/state";
        doc["availability_topic"] = "ESP-Now/sensor_" + sensor.id + "/status";
        JsonObject device = doc.createNestedObject("device");
        device["identifiers"] = sensor.id;
        device["name"] = "sensor_" + sensor.id;
        device["sw_version"] = "2021.10.0";
        device["model"] = "d1_mini";
        device["manufacturer"] = "espressif";

        String payload;
        serializeJson(doc, payload);
        mqttPublishConfig((char *)sensor.id.c_str(), "moisture/config", payload);
    }
    {
        DynamicJsonDocument doc(1024);
        doc["device_class"] = "voltage";
        doc["state_class"] = "measurement";
        doc["icon"] = "mdi:battery";
        doc["unit_of_measurement"] = "V";
        doc["name"] = "Battery Voltage";
        doc["value_template"] = "{{value_json.battery}}";
        doc["unique_id"] = "ESP-Now-" + sensor.id + "_battery";
        doc["state_topic"] = "ESP-Now/sensor_" + sensor.id + "/state";
        doc["availability_topic"] = "ESP-Now/sensor_" + sensor.id + "/status";
        JsonObject device = doc.createNestedObject("device");
        device["identifiers"] = sensor.id;
        device["name"] = "sensor_" + sensor.id;
        device["sw_version"] = "2021.10.0";
        device["model"] = "d1_mini";
        device["manufacturer"] = "espressif";

        String payload;
        serializeJson(doc, payload);
        mqttPublishConfig((char *)sensor.id.c_str(), "battery/config", payload);
    }
}

void publishState(StaticJsonDocument<256> &doc)
{
    String payload;
    serializeJson(doc["data"], payload);
    String macAddr = doc["mac"];
    mqttPublishState((char *)macAddr.c_str(), "state", payload);
    mqttPublishState((char *)macAddr.c_str(), "status", "online");
}

bool checkIfNewSensor(String id)
{
    bool found = false;
    for (size_t i = 0; i < numberOfSensor; i++)
    {
        if (sensors[i].id == id)
        {
            found = true;
            break;
        }
    }
    return !found;
};

void sensorMessageReceived(StaticJsonDocument<256> &doc)
{
    digitalWrite(LED_BUILTIN, LOW);

    JsonObject data = doc["data"].as<JsonObject>();
    data["uptime"] = doc["uptime"];

    SensorData sensor;
    sensor.id = doc["mac"].as<String>();
    sensor.name = "Sensor " + doc["mac"].as<String>();

    if (checkIfNewSensor(sensor.id))
    {
        sendMQTTDiscoveryMsg(sensor);
        numberOfSensor++;
    }

    serializeJson(doc["data"], sensor.data);
    publishState(doc);
    digitalWrite(LED_BUILTIN, HIGH);
}

/* ##### WifiManager ##### */

void wifiConnected()
{
    mqttReconnect();
    configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
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
        StaticJsonDocument<256> doc; // larger than it needs to be (Serial buffer size)

        char data[SERIAL_BUFFER_SIZE];
        SSerial.readBytesUntil(end_char, data, SERIAL_BUFFER_SIZE);

        // Decode message
        char *buffer = (char *)data;
        String msg_raw = String(buffer);

        deserializeJson(doc, msg_raw);

#ifdef DEBUG_FLAG
        Serial.println();
        Serial.println("New Message: ");
        Serial.print("Received: ");
        serializeJson(doc, Serial);
        Serial.println();
        Serial.println();
#endif

        sensorMessageReceived(doc);
    }
}