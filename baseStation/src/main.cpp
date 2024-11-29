#include <Arduino.h>

#include <WiFi.h>
#include "Wire.h"

#include "hardware/watchdog.h"

void software_reset();
void sendLEDSignal();

String status = "Connected";

enum OBSERVING_OBJECT {
    CHAIR,
    DOOR,
    RUBBISH_BIN,//BIN is already a keyword
    TABLE,
    CUPBOARD
};

bool turnOnLED = false;

const char* ap_ssid = "Group6BaseStation";    //Access Point SSID
const char* ap_password= "group6best"; //Access Point Password

uint8_t max_connections=5;//Maximum Connection Limit for AP

int current_stations=0, new_stations=0;

//Local
IPAddress local_IP(192, 168, 1, 2);        // Set your desired static IP address
IPAddress local_gateway(192, 168, 1, 1);   // Usually the same as the IP address
IPAddress local_subnet(255, 255, 255, 0);
int local_port = 80;

//Hotspot
IPAddress ap_IP(192, 168, 10, 2);  // Set your desired static IP address
IPAddress ap_gateway(192, 168, 10, 1);   // Usually the same as the IP address
IPAddress ap_subnet(255, 255, 255, 0);
int ap_port = 5263;

IPAddress IP;

//Specifying the Webserver instance to connect with HTTP Port: 80
WiFiServer server(5263);
 
void setup() {
  //Start the serial communication channel
  Serial.begin(115200);
  while (!Serial); // Wait until serial is available
  sleep_ms(6);//added because this is the minimum time I found that gets all serial message to print(no idea why the line above doesn't fully work)
 
  WiFi.mode(WIFI_AP);
  WiFi.setHostname("BaseStation");
  WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet);  // Configure static IP
   
  //Setting the AP Mode with SSID, Password, and Max Connection Limit
  if(WiFi.softAP(ap_ssid,ap_password,1,false,max_connections))
  {
    Serial.print("Access Point is Created with SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Max Connections Allowed: ");
    Serial.println(max_connections);
    Serial.print("Access Point IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("Unable to Create Access Point");
  }
  
  //Starting the Server
  server.begin();

  Serial.println("Base Station Started");
  Serial.println();
  Serial.println();
}

void loop(){
 WiFiClient client = server.available();               // listen for incoming clients
  if (client) {                                        // if you get a client....                              
    Serial.println("Client Connected...");
    if(turnOnLED){
      sendLEDSignal(client);
    }
    Serial.println(client.readStringUntil('\n'));      // print it out the serial monitor
    client.stop();                                     // stop the client connecting.
    Serial.println("Client Disconnected.");
  }
}

void sendLEDSignal(WiFiClient client){
  //'write' command may be wrong need to test
  client.write("LED: On");
}

//method to change boolean to true to turn on LED
// takes input from external device??

//can be used to reset data receiver if communication is lost
void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}