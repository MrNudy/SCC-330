#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "KuwshPixel";          // WiFi SSID
const char* password = "kuwshphone";       // WiFi Password
const char* server_ip = "192.168.107.233"; // Server IP
int port = 80;                             // HTTP Port for web server

WiFiClient client;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("PicoW_Client");

  Serial.printf("Connecting to WiFi '%s'...\n", ssid);
  WiFi.begin(ssid, password);

  // Wait for WiFi to connect
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to WiFi");
  Serial.printf("Client IP address: %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  // Attempt to connect to the HTTP server
  if (client.connect(server_ip, port)) {
    Serial.printf("Connected to server at %s:%d\n", server_ip, port);

    // Send an HTTP GET request
    client.println("GET / HTTP/1.1");
    client.printf("Host: %s\r\n", server_ip);
    client.println("Connection: close");
    client.println();

    // Read and print the response from the server
    while (client.connected() || client.available()) {
      if (client.available()) {
        String response = client.readStringUntil('\n');
        Serial.println(response);  // Print each line of the server's response
      }
    }
    client.stop();
    Serial.println("Disconnected from server.");
  } else {
    Serial.println("Connection to server failed.");
  }

  delay(10000); // Wait 10 seconds before making the next request
}
