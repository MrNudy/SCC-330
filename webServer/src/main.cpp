#include <Arduino.h>
#include <WiFi.h>
#include "BH1745NUC.h"          // Light measurement sensor library
#include "Wire.h"
#include <Adafruit_GFX.h>       // OLED display support library
#include <Adafruit_SSD1306.h>   // OLED display library
#include "bme68xLibrary.h"      // Climate sensor library
#include <HTTPClient.h>

// Wi-Fi Network configuration
const char* ssid = "KuwshPixel";        // Network name
const char* password = "kuwshphone";    // Network password
WiFiClient client;
WiFiServer server(5263);

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button and sensor setuprea
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
void sendMotionDataToFlask();
void sendLightDataToFlask();
void sendClimateDataToFlask();

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

  // Connect to the Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

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
      sendMotionDataToFlask();
    } else if (currentMode == LIGHT) {
      sendLightDataToFlask();
    } else if (currentMode == CLIMATE) {
      //sendClimateDataToFlask();
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

// Send motion data to Flask
void sendMotionDataToFlask() {
  motionSensor();
  HTTPClient http;
  String serverUrl = "http://192.168.246.190:5000/sensor_data?zone=lab&motion=" + String(pirState == HIGH ? "Motion Detected" : "No Motion");
  http.begin(serverUrl);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("HTTP GET request sent to Flask server, response code: %d\n", httpCode);
  } else {
    Serial.printf("Failed to send data to Flask server, error code: %d\n", httpCode);
  }
  http.end();
}

// Send light data to Flask
void sendLightDataToFlask() {
  readLightMeasurements();
  unsigned short rgbc[4] = { sensor.red, sensor.green, sensor.blue, sensor.clear };
  HTTPClient http;
  String serverUrl = "http://192.168.246.190:5000/sensor_data?zone=lab&light_red=" + String(rgbc[0]) +
                      "&light_green=" + String(rgbc[1]) +
                      "&light_blue=" + String(rgbc[2]) +
                      "&light_clear=" + String(rgbc[3]);
  http.begin(serverUrl);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("HTTP GET request sent to Flask server, response code: %d\n", httpCode);
  } else {
    Serial.printf("Failed to send data to Flask server, error code: %d\n", httpCode);
  }
  http.end();
}

// Send climate data to Flask
/*void sendClimateDataToFlask() {
  readClimateValues();
  HTTPClient http;
  String serverUrl = "http://127.0.0.1:5000/sensor_data?zone=lab&temperature=" + String(data.temperature - 4.49) +
                      "&humidity=" + String(data.humidity) +
                      "&pressure=" + String(data.pressure);
  http.begin(serverUrl);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("HTTP GET request sent to Flask server, response code: %d\n", httpCode);
  } else {
    Serial.printf("Failed to send data to Flask server, error code: %d\n", httpCode);
  }
  http.end();
}*/
