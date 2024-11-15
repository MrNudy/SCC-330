#include <Arduino.h>
#include <WiFi.h>
#include "BH1745NUC.h"          // Light measurement sensor library
#include "Wire.h"
#include <Adafruit_GFX.h>       // OLED display support library
#include <Adafruit_SSD1306.h>   // OLED display library
#include "bme68xLibrary.h"      // Climate sensor library

// Wi-Fi Access Point configuration
const char* ap_ssid = "Group6BaseStation";
const char* ap_password = "group6best";
IPAddress ap_IP(192, 168, 10, 1);
IPAddress ap_gateway(192, 168, 10, 1);
IPAddress ap_subnet(255, 255, 255, 0);
WiFiServer server(5263);

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button and sensor setup
#define REDButton 12
#define BLACKButton 13
#define detector LED_BUILTIN
#define sensorPIN 11
int pirState = LOW;
int value = 0;

// Light sensor setup
#define BH1745NUC_DEVICE_ADDRESS_38 0x38
BH1745NUC sensor = BH1745NUC();

// Climate sensor setup
Bme68x bme;

// Mode tracking
enum Mode {MOTION, LIGHT, CLIMATE};
volatile Mode currentMode = MOTION;

// Function declarations
void redPressed();
void blackPressed();
void turnLED_ON();
void turnLED_OFF();
void motionSensor();
void readLightMeasurements();
void readClimateValues();
void sendMotionDataHTTP(WiFiClient& client);
void sendLightDataHTTP(WiFiClient& client);
void sendClimateDataHTTP(WiFiClient& client);

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Setup pins
  pinMode(detector, OUTPUT);
  pinMode(sensorPIN, INPUT);
  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);

  // Set up button interrupts
  attachInterrupt(digitalPinToInterrupt(REDButton), redPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BLACKButton), blackPressed, FALLING);

  // Start OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();

  // Set up light sensor
  Wire.begin();
  sensor.begin(BH1745NUC_DEVICE_ADDRESS_38);
  sensor.startMeasurement();

  // Set up climate sensor
  bme.begin(0x76, Wire);
  bme.setTPH();
  uint16_t tempProf[10] = { 100, 200, 320 };
  uint16_t durProf[10] = { 150, 150, 150 };
  bme.setSeqSleep(BME68X_ODR_250_MS);
  bme.setHeaterProf(tempProf, durProf, 3);
  bme.setOpMode(BME68X_SEQUENTIAL_MODE);

  // Configure Wi-Fi as Access Point
  WiFi.mode(WIFI_AP);
  WiFi.setHostname("BaseStation");
  WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet);
  if (WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("Access Point created successfully!");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to create Access Point.");
  }

  // Start server
  server.begin();
  Serial.println("Server started and awaiting connections.");
}

void loop() {
  WiFiClient client = server.accept();
  if (client) {
    Serial.println("Client connected.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Client connected.");
    display.display();

    if (currentMode == MOTION) {
      sendMotionDataHTTP(client);
    } else if (currentMode == LIGHT) {
      sendLightDataHTTP(client);
    } else if (currentMode == CLIMATE) {
      sendClimateDataHTTP(client);
    }

    client.stop();
    Serial.println("Client disconnected.");
  }
  delay(5000);
}

// Button press functions
void redPressed() {
  currentMode = static_cast<Mode>((currentMode + 1) % 3);
  if (currentMode == MOTION) {
    Serial.println("Switched to Motion Sensor Mode.");
    display.clearDisplay();
    display.println("Motion Sensor Mode:");
  } else if (currentMode == LIGHT) {
    Serial.println("Switched to Light Sensor Mode.");
    display.clearDisplay();
    display.println("Light Sensor Mode:");
  } else if (currentMode == CLIMATE) {
    Serial.println("Switched to Climate Sensor Mode.");
    display.clearDisplay();
    display.println("Climate Sensor Mode:");
  }
  display.display();
}

void blackPressed() {
  currentMode = static_cast<Mode>((currentMode + 2) % 3);
  if (currentMode == MOTION) {
    Serial.println("Switched to Motion Sensor Mode.");
    display.clearDisplay();
    display.println("Motion Sensor Mode:");
  } else if (currentMode == LIGHT) {
    Serial.println("Switched to Light Sensor Mode.");
    display.clearDisplay();
    display.println("Light Sensor Mode:");
  } else if (currentMode == CLIMATE) {
    Serial.println("Switched to Climate Sensor Mode.");
    display.clearDisplay();
    display.println("Climate Sensor Mode:");
  }
  display.display();
}

// LED functions
void turnLED_ON() {
  digitalWrite(detector, HIGH);
}

void turnLED_OFF() {
  digitalWrite(detector, LOW);
}

// Motion sensor function
void motionSensor() {
  value = digitalRead(sensorPIN);
  if (value == HIGH) {
    turnLED_ON();
    if (pirState == LOW) {
      Serial.println("Motion Detected!");
      display.clearDisplay();
      display.println("Motion Detected!");
      pirState = HIGH;
      display.display();
    }
  } else {
    turnLED_OFF();
    if (pirState == HIGH) {
      Serial.println("Motion stopped!");
      display.clearDisplay();
      display.println("Motion stopped!");
      pirState = LOW;
      display.display();
    }
  }
}

// Light sensor function
void readLightMeasurements() {
  if (!sensor.read()) {
    Serial.println("Light sensor failed to read measurement values!");
    return;
  }

  unsigned short rgbc[4] = { sensor.red, sensor.green, sensor.blue, sensor.clear };

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Light Measurements:");
  display.print("Red: "); display.println(rgbc[0]);
  display.print("Green: "); display.println(rgbc[1]);
  display.print("Blue: "); display.println(rgbc[2]);
  display.print("Clear: "); display.println(rgbc[3]);
  display.display();
}

// Climate sensor function
void readClimateValues() {
  bme68xData data;
  uint8_t nFieldsLeft = bme.getData(data);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(data.temperature - 4.49);
  display.println(" 'C");
  display.print("Humidity: ");
  display.print(data.humidity);
  display.println(" %");
  display.print("Pressure: ");
  display.print(data.pressure);
  display.println(" Pa");
  display.display();
}

// Send motion data to client
void sendMotionDataHTTP(WiFiClient& client) {
  motionSensor();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML><html><head><title>Motion Sensor</title></head><body><h1>Motion Sensor Data</h1>");
  client.println(pirState == HIGH ? "<p>Motion Detected!</p>" : "<p>No Motion</p>");
  client.println("</body></html>");
}

// Send light data to client
void sendLightDataHTTP(WiFiClient& client) {
  readLightMeasurements();
  unsigned short rgbc[4] = { sensor.red, sensor.green, sensor.blue, sensor.clear };

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML><html><head><title>Light Measurements</title><meta http-equiv=\"refresh\" content=\"10\"></head><body><h1>Light Measurements</h1>");
  client.printf("<p>Red: %d</p>", rgbc[0]);
  client.printf("<p>Green: %d</p>", rgbc[1]);
  client.printf("<p>Blue: %d</p>", rgbc[2]);
  client.printf("<p>Clear: %d</p>", rgbc[3]);
  client.println("</body></html>");
}

// Send climate data to client
void sendClimateDataHTTP(WiFiClient& client) {
  readClimateValues();
  bme68xData data;
  uint8_t nFieldsLeft = bme.getData(data);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE HTML><html><head><title>Climate Data</title><meta http-equiv=\"refresh\" content=\"10\"></head><body><h1>Climate Sensor Data</h1>");
  client.printf("<p>Temperature: %.2f 'C</p>", data.temperature - 4.49);
  client.printf("<p>Humidity: %.2f %%</p>", data.humidity);
  client.printf("<p>Pressure: %.2f Pa</p>", data.pressure);
  client.println("</body></html>");
}
