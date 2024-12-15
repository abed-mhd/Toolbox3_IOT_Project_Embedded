#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

// Constants
#define ROOM 2  // Room number

// OLED configuration
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Button Configuration
#define BUTTON_PIN 15  
bool buttonPressed = false;

// DHT Sensor Configuration
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// BME280 Sensor Configuration
Adafruit_BME280 bme;

// TEMT6000 Luminosity Sensor Configuration
#define TEMT6000_PIN A0

// Wi-Fi and MQTT Broker Configuration
const char* ssid = "Holmes";
const char* password = "aptx4869";
const char* mqtt_server = "broker.hivemq.com";
const char* unique_identifier = "clientId-drs69JNJJC";
const int mqtt_port = 1883;

// MQTT Topics
const char* auth_request_topic = "rooms/auth/request";
const char* sensor_data_topic = "rooms/sensors/data";
const char* auth_response_topic = "rooms/auth/response";
const char* message_topic = "rooms/message";  
const char* cancel_auth_topic = "rooms/auth/cancel"; 

WiFiClient espClient;
PubSubClient client(espClient);

// Variables
bool authorized = false;  // To track authorization status
String incomingMessage = "";  // Variable to store incoming message
bool waitingForMessage = false;  // Flag to know if we are displaying a message
unsigned long previousMillis = 0; 
const unsigned long sensorInterval = 5000;


void setup_wifi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Room %d\n", ROOM);
  display.println("Connecting to Wi-Fi...");
  display.display();

  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Room %d\n", ROOM);
  display.println("Wi-Fi Connected!");
  display.display();
  delay(2000);
}

void reconnect() {
  while (!client.connected()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Room %d\n", ROOM);
    display.println("Connecting to MQTT...");
    display.display();

    Serial.print("Connecting to MQTT broker...");
    if (client.connect("ESP32_Room2")) {
      Serial.println("connected!");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.printf("Room %d\n", ROOM);
      display.println("MQTT Connected!");
      display.display();
      delay(2000);

      client.subscribe(auth_response_topic);  // Subscribe to authorization response
      client.subscribe(message_topic);  // Subscribe to messages
      client.subscribe(cancel_auth_topic);  // Subscribe to cancel auth topic
      Serial.println("Subscribed to topics.");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds...");
      delay(2000);
    }
  }
}

void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
  Serial.println("Button pressed interrupt triggered.");
}

void displaySensorData(float temperature, float humidity, float pressure, float lux) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Room %d\n", ROOM);
  display.printf("Temp: %.1f C\n", temperature);
  display.printf("Humidity: %.1f %%\n", humidity);
  display.printf("Pressure: %.1f hPa\n", pressure);
  display.printf("Luminosity: %.2f lux\n", lux);
  display.display();
}

void readAndPublishSensorData() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float pressure = bme.readPressure() / 100.0F; 
  int rawValue = analogRead(TEMT6000_PIN);
  float lux = (rawValue * 5.0 / 1023.0) * 2000;

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  displaySensorData(temperature, humidity, pressure, lux);

  char json_message[256];
  snprintf(json_message, sizeof(json_message),
           "{\"room\":%d,\"temperature\":%.1f,\"humidity\":%.1f,\"pressure\":%.1f,\"luminosity\":%.2f}",
           ROOM, temperature, humidity, pressure, lux);
  client.publish(sensor_data_topic, json_message);
  Serial.println("Sensor data published.");
}

void handleAuthorization() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Requesting Auth...");
  display.display();

  char json_message[64];
  snprintf(json_message, sizeof(json_message), "{\"room\":%d}", ROOM);
  client.publish(auth_request_topic, json_message);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Auth Request Sent");
  display.display();
  delay(1000);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Callback received. Topic: ");
  Serial.println(topic);
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  int room = doc["room"] | -1;
  if (room != ROOM) {
    Serial.println("Message not for this room.");
    return;
  }

  if (String(topic) == auth_response_topic) {
    bool auth = doc["auth"] | false;
    if (auth) {
      Serial.println("Authorization accepted.");
      authorized = true;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Authorization Accepted!");
      display.display();
      delay(2000);
    } else {
      Serial.println("Authorization rejected.");
      authorized = false;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Authorization Rejected!");
      display.display();
      delay(2000);
    }
  } else if (String(topic) == message_topic) {
    String incoming = doc["message"] | "";
    Serial.println("Message received to display.");
    incomingMessage = incoming;
    waitingForMessage = true;
  } else if (String(topic) == cancel_auth_topic) {
    Serial.println("Authorization canceled.");
    authorized = false;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Authorization Canceled!");
    display.display();
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);

  // OLED Setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED initialization failed!");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Show Logo and Room Info
  display.setCursor(0, 0);
  display.println("** Welcome **");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Room %d", ROOM);
  display.display();

  // Wi-Fi and MQTT Setup
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Button Setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  // Sensor Setup
  dht.begin();
  if (!bme.begin(0x76)) { 
    Serial.println("Could not find a valid BME280 sensor!");
    while (1);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (buttonPressed) {
    buttonPressed = false;
    Serial.println("Button press processed in loop.");

    if (waitingForMessage) {
      Serial.println("Clearing message display.");
      waitingForMessage = false;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.printf("Room %d - Sensor Data", ROOM);
      display.display();
    } else if (!authorized) {
      Serial.println("Requesting authorization.");
      handleAuthorization();
    }
  }

  if (waitingForMessage) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Message Received:");
    display.println(incomingMessage);
    display.display();
  } else if (authorized) {
    if (currentMillis - previousMillis >= sensorInterval) {
      Serial.println("Reading and displaying sensor data.");
      previousMillis = currentMillis;
      readAndPublishSensorData();
    }
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Press Button for Auth");
    display.display();
  }
}
