#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "realme GT 2";
const char* password = "12345678";

// MQTT broker details
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* topic = "esp32/coordinates";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Setup MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  // Connect to MQTT broker
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ESP32_Subscriber")) {
      Serial.println("connected");
      mqttClient.subscribe(topic); // Subscribe to the topic
    } else {
      Serial.print("failed with state ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void loop() {
  if (!mqttClient.connected()) {
    // Reconnect if the MQTT connection is lost
    while (!mqttClient.connected()) {
      Serial.print("Reconnecting to MQTT...");
      if (mqttClient.connect("ESP32_Subscriber")) {
        Serial.println("connected");
        mqttClient.subscribe(topic);
      } else {
        Serial.print("failed with state ");
        Serial.println(mqttClient.state());
        delay(2000);
      }
    }
  }
  mqttClient.loop();
}

// Callback function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("Message received on topic: " + String(topic));
  Serial.println("Payload: " + message);

  // Parse JSON payload
  StaticJsonDocument<256> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, message);

  if (error) {
    Serial.println("Failed to parse MQTT message");
    return;
  }

  // Extract X and Y coordinates
  int x = jsonDoc["x"];
  int y = jsonDoc["y"];

  // Print coordinates to Serial Monitor
  Serial.print("Joystick Coordinates - X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.println(y);
}
