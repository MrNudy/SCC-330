#include <Arduino.h>

#include <WiFi.h>
#include "Wire.h"
#include <string.h>
#include <iostream>  
#include "hardware/watchdog.h"

void software_reset();

String status = "Connected";

enum OBSERVING_OBJECT {
    CHAIR,
    DOOR,
    RUBBISH_BIN,//BIN is already a keyword
    TABLE,
    CUPBOARD
};

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

String actuatorBuffer;
 
void setup() {
  //Start the serial communication channel
  Serial.begin(115200);
  // while (!Serial); // Wait until serial is available
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
    String line = client.readStringUntil('\n');
    Serial.println(line);
    if(line[0]=='E'){
      int lastchar = 3;
      int nums[4];
      int n = 0;
      for(int i = 4;i<line.length();i++){
        if(line[i] == ','){
          nums[n] = line.substring(lastchar+1,i).toFloat();
          lastchar = i;
          n++;
        }
      }
      nums[n] = line.substring(lastchar+1,line.length()).toFloat();
      if((int)nums[0]<20){
        Serial.println("1");
        actuatorBuffer = "A:1";//GP1 on
      }
      else{
        Serial.println("0");
        actuatorBuffer = "A:0";//GP1 off
      }
    }
    if(line[0]=='A'){
      Serial.println("Actuator mode send");
      client.println(actuatorBuffer);
    }
  }
}

//can be used to reset data receiver if communication is lost
void software_reset()
{
    watchdog_enable(1, 1);
    while(1);
}