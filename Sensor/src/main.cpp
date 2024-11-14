#include <Arduino.h>
#include <WiFi.h>
#include "bme68xLibrary.h" // Climate sensor library
#include "Wire.h"
#include <Adafruit_GFX.h> // OLED display support library
#include <Adafruit_SSD1306.h> // OLED display library

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button and server definitions
#define REDButton 12
#define BLACKButton 13
#define REMOTE_IP "192.168.10.1"
#define REMOTE_PORT 5263
#define API_SERVER_IP "192.168.1.1"
#define API_SERVER_PORT 5000

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

// Modes
enum SENSOR_MODE {
  ENVIRONMENT,
  OBJECT,
  CUP,
  ACTUATOR
};
SENSOR_MODE mode = ENVIRONMENT;

const char* ssid = "Group6BaseStation";
const char* password = "group6best";

// Initialize sensor, WiFi clients, and functions
Bme68x bme;
WiFiClient baseStationClient; // client for base station
WiFiClient apiClient; // client for API server

void sendClimateData();
void sendObjectData();
void sendCupData();
void redButtonPressed();
void blackButtonPressed();
void changeMode();

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true); // halt on error
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  delay(6);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Group6Station");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    display.print(".");
    display.display();
    delay(200);
  }

  Serial.println("WiFi connected");
  display.println("WiFi connected");
  display.println("IP address: ");
  display.println(WiFi.localIP());
  display.display();
  delay(500);

  // Connect to base station
  while (!baseStationClient.connect(REMOTE_IP, REMOTE_PORT)) {
    Serial.println("Connection to base station failed, retrying...");
    delay(500);
  }

  // Connect to API server
  while (!apiClient.connect(API_SERVER_IP, API_SERVER_PORT)) {
    Serial.println("Connection to API server failed, retrying...");
    delay(500);
  }

  // Initialize climate sensor
  bme.begin(0x76, Wire);
  if (bme.checkStatus()) {
    if (bme.checkStatus() == BME68X_ERROR) {
      Serial.println("Sensor error: " + bme.statusString());
      return;
    } else if (bme.checkStatus() == BME68X_WARNING) {
      Serial.println("Sensor warning: " + bme.statusString());
    }
  }
  bme.setTPH();
  uint16_t tempProf[10] = {100, 200, 320};
  uint16_t durProf[10] = {150, 150, 150};
  bme.setSeqSleep(BME68X_ODR_250_MS);
  bme.setHeaterProf(tempProf, durProf, 3);
  bme.setOpMode(BME68X_SEQUENTIAL_MODE);

  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);
  attachInterrupt(REDButton, redButtonPressed, FALLING);
  attachInterrupt(BLACKButton, blackButtonPressed, FALLING);
}

void loop() {
  display.clearDisplay();
  display.setCursor(0, 0);

  if (baseStationClient.available() > 0) {
    delay(20);
    String line = baseStationClient.readString();
    display.println(REMOTE_IP + String(":") + line);
    Serial.println(REMOTE_IP + String(":") + line);
  }
  if (Serial.available() > 0) {
    delay(20);
    String line = Serial.readString();
    baseStationClient.print(line);
  }

  if (!baseStationClient.connected()) {
    baseStationClient.stop();
    WiFi.disconnect();
  }

  switch (mode) {
  case ENVIRONMENT:
    sendClimateData();
    break;
  case OBJECT:
    sendObjectData();
    break;
  case CUP:
    sendCupData();
    break;
  case ACTUATOR:
    break;
  default:
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor mode error");
    display.display();
    delay(300);
  }
}

void sendClimateData() {
  bme68xData data;
  uint8_t nFieldsLeft = 0;
  delay(150);

  if (bme.fetchData()) {
    do {
      nFieldsLeft = bme.getData(data);
      String climateData = String(data.temperature - 4.49) + ", " + String(data.humidity) + ", " + String(data.pressure);

      // Send data to base station (no changes needed here)
      baseStationClient.print(climateData + '\n'); 

      // Create the JSON payload for API
      String jsonData = "{\"sensor_mode\": \"ENVIRONMENT\", \"data\": \"" + climateData + "\"}";

      // Send POST request with JSON data to API
      apiClient.print("POST /store_data HTTP/1.1\r\n");
      apiClient.print("Host: " + String(API_SERVER_IP) + "\r\n");
      apiClient.print("Content-Type: application/json\r\n");
      apiClient.print("Content-Length: " + String(jsonData.length()) + "\r\n");
      apiClient.print("\r\n" + jsonData + "\r\n");

      if (data.gas_index == 2) delay(250);
    } while (nFieldsLeft);
  }
}

void sendObjectData() {
  // Implement object data sending
}

void sendCupData() {
  // Implement cup data sending
}

void redButtonPressed() {
  changeMode();
}

void blackButtonPressed() {
  // Black button logic per mode
}

void changeMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  switch (mode) {
  case ENVIRONMENT:
    mode = OBJECT;
    display.println("object mode");
    break;
  case OBJECT:
    mode = CUP;
    display.println("cup mode");
    break;
  case CUP:
    mode = ACTUATOR;
    display.println("actuator mode");
    break;
  case ACTUATOR:
    mode = ENVIRONMENT;
    display.println("environment mode");
    break;
  default:
    display.println("mode error");
  }
  display.display();
  delay(300);
}
