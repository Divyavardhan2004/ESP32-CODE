#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "realme GT 2";
const char* password = "12345678";

AsyncWebServer server(80); // Web server on port 80
AsyncWebSocket ws("/ws");  // WebSocket server on path "/ws"

// Variables for joystick data
int joystickX = 0;
int joystickY = 0;

// Function prototypes
void handleReceivedData(uint8_t* data, size_t length);
int convertSpeed(float speed);
void setMotor(int dirPin, int speedPin, int direction, int speed);

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // WebSocket event handler
  ws.onEvent(webSocketEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  // Motor pins
  pinMode(22, OUTPUT); // Motor 1 speed
  pinMode(23, OUTPUT);  // Motor 1 direction
  pinMode(19, OUTPUT);  // Motor 2 speed
  pinMode(21, OUTPUT); // Motor 2 direction
  pinMode(15, OUTPUT); // Motor 3 speed
  pinMode(2, OUTPUT); // Motor 3 direction
//  pinMode(22, OUTPUT); // Motor 4 speed
//  pinMode(23, OUTPUT); // Motor 4 direction
}

void loop() {
  // Handle WebSocket communication
  ws.cleanupClients();

  // Check if joystick data is significant
  if (abs(joystickX) > 10 && abs(joystickY) > 10) {
    // Calculate motor speeds using velocity equation
//    float motorSpeed1 = 0.3536 * (joystickX + joystickY);
//    float motorSpeed2 = 0.3536 * (joystickX - joystickY);
//    float motorSpeed3 = 0.3536 * (-joystickX - joystickY);
//    float motorSpeed4 = 0.3536 * (-joystickX + joystickY);

    float motorSpeed1 = -0.67 * joystickX ;
    float motorSpeed2 = 0.33 * joystickX + 0.57 * joystickY;
    float motorSpeed3 =  0.31 * joystickX - 0.57 * joystickY;

    // Convert motor speeds to the 0-255 range
    int speed1 = convertSpeed(motorSpeed1);
    int speed2 = convertSpeed(motorSpeed2);
    int speed3 = convertSpeed(motorSpeed3);
//    int speed4 = convertSpeed(motorSpeed4);

    // Set motor 1 direction and speed
    digitalWrite(23, motorSpeed1 >= 0 ? HIGH : LOW);
    analogWrite(22, abs(speed1));

    // Set motor 2 direction and speed
    digitalWrite(21, motorSpeed2 >= 0 ? HIGH : LOW);
    analogWrite(19, abs(speed2));

    // Set motor 3 direction and speed
    digitalWrite(15, motorSpeed3 >= 0 ? HIGH : LOW);
    analogWrite(2, abs(speed3));

//    // Set motor 4 direction and speed
//    digitalWrite(23, motorSpeed4 >= 0 ? HIGH : LOW);
//    analogWrite(22, abs(speed4));
  } 
  else {
    // Stop all motors if joystick is near neutral
    analogWrite(22, 0);
    analogWrite(2, 0);
    analogWrite(19, 0);
//    analogWrite(22, 0);
  }
}

// WebSocket event handler
void webSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, 
                    void* arg, uint8_t* data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("Client connected");
      break;
    case WS_EVT_DISCONNECT:
      Serial.println("Client disconnected");
      break;
    case WS_EVT_ERROR:
      Serial.println("Error");
      break;
    case WS_EVT_DATA:
      // Received data: parse it as JSON
      handleReceivedData(data, length);
      break;
  }
}

// Function to handle received joystick and button data
void handleReceivedData(uint8_t* data, size_t length) {
  // Convert data to a String
  String msg;
  for (size_t i = 0; i < length; i++) {
    msg += (char)data[i];
  }
  Serial.println("Received data: " + msg);

  // Parse the JSON data
  StaticJsonDocument<256> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, msg);

  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }

  // Handle joystick data
  if (jsonDoc.containsKey("type") && strcmp(jsonDoc["type"], "joystick") == 0) {
    joystickX = jsonDoc["x"];
    joystickY = jsonDoc["y"];
    Serial.print("Joystick - X: ");
    Serial.print(joystickX);
    Serial.print(", Y: ");
    Serial.println(joystickY);
  }
}

// Speed conversion function
int convertSpeed(float speed) {
  // Map speed to range 0-255
  return (int)(abs(speed) * 2.5);  // Scale speed accordingly
}

// Helper function to set motor direction and speed
void setMotor(int dirPin, int speedPin, int direction, int speed) {
  digitalWrite(dirPin, direction);
  analogWrite(speedPin,speed);
}
