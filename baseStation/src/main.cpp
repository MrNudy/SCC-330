#include <Arduino.h>

#include <WiFi.h>


#include "hardware/watchdog.h"

void software_reset();

String status = "Connected";

//Specifies the SSID and Password of the AP
const char* local_ssid = "Group6BaseStation";    //Access Point SSID
const char* local_password= "Group6"; //Access Point Password

const char* ap_ssid = "Group6BaseStation";    //Access Point SSID
const char* ap_password= "Group6"; //Access Point Password

uint8_t max_connections=5;//Maximum Connection Limit for AP

int current_stations=0, new_stations=0;

//Local
IPAddress local_IP(192, 168, 6, 1);        // Set your desired static IP address
IPAddress local_gateway(192, 168, 6, 1);   // Usually the same as the IP address
IPAddress local_subnet(255, 255, 255, 0);
int local_port = 80;

//Hotspot
IPAddress ap_IP(192, 168, 10, 1);  // Set your desired static IP address
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
  WiFi.setHostname("Group6BaseStation");
  WiFi.softAPConfig(ap_IP, ap_gateway, ap_subnet);  // Configure static IP
   
  //Setting the AP Mode with SSID, Password, and Max Connection Limit
  if(WiFi.softAP(ap_ssid,ap_password,4,false,max_connections)==true)
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
    while (client.connected()) {                       // loop while the client's connected
      if (client.available()) 
      {                                                // if there's bytes to read from the client,
        Serial.println(client.readStringUntil('\n'));  // print it out the serial monitor
        while(client.read()>0);                        // clear the wifi receive area cache
      }
      //if(Serial.available()){                          // if there's bytes to read from the serial monitor,
      //  client.print(Serial.readStringUntil('\n'));    // print it out the client.
      //  while(Serial.read()>0);                        // clear the wifi receive area cache
      //}
    }
    client.stop();                                     // stop the client connecting.
    Serial.println("Client Disconnected.");
  }
}

//can be used to reset data receiver if communication is lost
void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}