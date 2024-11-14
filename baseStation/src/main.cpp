#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h> // Include HTTP client for sending data to API
#include "Wire.h"
#include "hardware/watchdog.h"

void software_reset();
void sendDataToAPI(const String& data, const String& sensor_mode); // Add sensor_mode as a parameter

String status = "Connected";

const char* ap_ssid = "Group6BaseStation";    // Access Point SSID
const char* ap_password = "group6best";       // Access Point Password
const char* api_url = "http://192.168.1.1:5000/store_data"; // Updated API URL

uint8_t max_connections = 5; // Maximum Connection Limit for AP
int current_stations = 0, new_stations = 0;

// Local Network Configuration
IPAddress local_IP(192, 168, 1, 2);
IPAddress local_gateway(192, 168, 1, 1);
IPAddress local_subnet(255, 255, 255, 0);
int local_port = 80;

// Hotspot Configuration
IPAddress ap_IP(192, 168, 10, 2);
IPAddress ap_gateway(192, 168, 10, 1);
IPAddress ap_subnet(255, 255, 255, 0);
int ap_port = 5263;

IPAddress IP;
WiFiServer server(5263); // Webserver instance on port 5263

void setup() {
  // Start serial communication
  Serial.begin(115200);
  while (!Serial);
  sleep_ms(6);

  WiFi.mode(WIFI_AP);
  WiFi.setHostname("BaseStation");
  WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet); // Configure static IP for AP mode

  if (WiFi.softAP(ap_ssid, ap_password, 1, false, max_connections)) {
    Serial.print("Access Point Created with SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Max Connections Allowed: ");
    Serial.println(max_connections);
    Serial.print("Access Point IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Unable to Create Access Point");
  }
  
  server.begin();
  Serial.println("Base Station Started\n");
}

void loop() {
  WiFiClient client = server.available(); // Listen for incoming clients
  if (client) {
    Serial.println("Client Connected...");
    String data = client.readStringUntil('\n'); // Read data from client
    Serial.println(data); // Print to Serial Monitor
    sendDataToAPI(data, "ENVIRONMENT"); // Example sensor mode
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

// Function to send data to API
void sendDataToAPI(const String& data, const String& sensor_mode) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(api_url); // Specify API URL
    http.addHeader("Content-Type", "application/json");

    // Updated JSON payload structure
    String jsonData = "{\"sensor_mode\": \"" + sensor_mode + "\", \"data\": \"" + data + "\"}";
    
    int httpResponseCode = http.POST(jsonData); // Send HTTP POST request

    if (httpResponseCode > 0) {
      Serial.print("Data sent to API, response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error sending data to API, error code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi not connected, unable to send data to API.");
  }
}

void software_reset() {
  watchdog_enable(1, 1);
  while (1);
}
