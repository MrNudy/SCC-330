#include <Arduino.h>
#include <WiFi.h>
#include "Wire.h"
#include "hardware/watchdog.h"

void software_reset();
bool connectToServer();

String status = "Connected";

// Enum for objects being observed
enum OBSERVING_OBJECT {
   CHAIR,
   DOOR,
   RUBBISH_BIN,
   TABLE,
   CUPBOARD
};

const char* ap_ssid = "Group6BaseStationTest"; // Access Point SSID
const char* ap_password = "group6best";       // Access Point Password
uint8_t max_connections = 5;                   // Max Connections for AP

int current_stations = 0, new_stations = 0;

// Local IP settings
IPAddress local_IP(192, 168, 1, 2);
IPAddress local_gateway(192, 168, 1, 1);
IPAddress local_subnet(255, 255, 255, 0);
int local_port = 80;

// Hotspot settings
IPAddress ap_IP(192, 168, 10, 2);  
IPAddress ap_gateway(192, 168, 10, 1);  
IPAddress ap_subnet(255, 255, 255, 0);
int ap_port = 5263;

// Server settings
IPAddress server_IP(192, 168, 10, 19); // Server IP // make sure to change this manually when connecting on server
int server_port = 5263;               // Server Port

// WiFi client and server
WiFiClient serverClient;
WiFiServer sensorServer(ap_port);

void setup() {
   Serial.begin(115200);
   while (!Serial)
       ; // Wait until serial is available
   sleep_ms(6);

   WiFi.mode(WIFI_AP);
   WiFi.setHostname("BaseStation");
   WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet); // Static IP for AP

   // Set up Access Point with SSID, Password, and max connections
   if (WiFi.softAP(ap_ssid, ap_password, 1, false, max_connections)) {
       Serial.print("Access Point is created with SSID: ");
       Serial.println(ap_ssid);
       Serial.print("Max Connections Allowed: ");
       Serial.println(max_connections);
       Serial.print("Access Point IP: ");
       Serial.println(WiFi.softAPIP());
   } else {
       Serial.println("Failed to create Access Point");
   }

   sensorServer.begin();  // Start the AP server

   Serial.println("Base Station Started");
   Serial.println();
}

void loop() {
   // Listen for incoming sensor connections
   WiFiClient client = sensorServer.available(); // Wait for client connection
   if (client) {
       Serial.println("Client Connected...");

       // Read data from the sensor client
       String sensorData = client.readStringUntil('\n');
       Serial.println("Sensor Data: " + sensorData); // Display data in serial

       // Forward data to the server
       if (connectToServer()) {
           serverClient.print(sensorData + "\n");  // Send data to server
           Serial.println("Data forwarded to server.");
       } else {
           Serial.println("Failed to forward data to server.");
       }
   }
}

// Function to reset receiver if communication is lost
void software_reset() {
   watchdog_enable(1, 1);
   while (1)
       ; // Infinite loop to trigger reset
}

// Function to connect to the server
bool connectToServer() {
   if (!serverClient.connected()) {
       Serial.println("Connecting to the server...");
       if (serverClient.connect(server_IP, server_port)) {
           Serial.println("Connected to the server.");
       } else {
           Serial.println("Failed to connect to the server.");
           return false;
       }
   }
   return true;
}