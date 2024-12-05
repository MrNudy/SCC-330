#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BH1745NUC.h"          // Light sensor library

// Screen and light sensor setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define BH1745NUC_DEVICE_ADDRESS_38 0x38

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BH1745NUC sensor = BH1745NUC();

// WiFi credentials and server setup
const char* ssid = "KuwshPixel";
const char* password = "kuwshphone";
WiFiServer server(80);

// Function to display messages on OLED
void displayMessage(const String &message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(message);
  display.display();
}

// Function to read light measurements
String getLightMeasurements() {
  if (!sensor.read()) {
    Serial.println("Light sensor failed to read values!");
    return "Error reading light sensor";
  }
  
  unsigned short rgbc[4] = { sensor.red, sensor.green, sensor.blue, sensor.clear };
  String result = "Light measurements:<br>";
  result += "Red: " + String(rgbc[0]) + "<br>";
  result += "Green: " + String(rgbc[1]) + "<br>";
  result += "Blue: " + String(rgbc[2]) + "<br>";
  result += "Clear: " + String(rgbc[3]) + "<br>";
  return result;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize light sensor
  sensor.begin(BH1745NUC_DEVICE_ADDRESS_38);
  sensor.startMeasurement();

  // Start WiFi connection
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("PicoW_Server");
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi '%s'...\n", ssid);
  displayMessage("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected to WiFi");
  displayMessage("Connected to WiFi");

  Serial.printf("Server IP: %s, Port: %d\n", WiFi.localIP().toString().c_str(), 80);
  displayMessage("Server IP: " + WiFi.localIP().toString());

  server.begin();
}

void loop() {
  WiFiClient client = server.accept();
  if (client) {
    Serial.println("Client connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // Send HTTP response with light readings
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    
    // HTML content
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<head><title>Pico-W Light Sensor</title></head>");
    client.println("<body>");
    client.println("<h1>Light Sensor Readings</h1>");
    client.println(getLightMeasurements());  // Add light sensor data
    client.println("</body></html>");
    client.stop();
    Serial.println("Client disconnected");
  }
}
