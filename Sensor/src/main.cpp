#include <Arduino.h>
#include <WiFi.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
                                   //Library added to lib folder on the left

#define     REMOTE_IP          "192.168.6.1"  //remote server IP which that you want to connect to
#define     REMOTE_PORT         5263           //connection port provided on remote server 

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)


const char* ssid = "Group6BaseStation";     //access Point SSID
const char* password= "Group6";  //access Point Password

Bme68x bme;                         //declares climate sensor variable
WiFiClient client;                  //declares WiFi client

//declare functions implemented
void sendClimateData();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); // Wait until serial is available
  sleep_ms(6);//added because this is the minimum time I found that gets all serial message to print(no idea why the line above doesn't fully work)
 

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);              //sets WiFi as station/client
  WiFi.setHostname("Group6Station");
 

  WiFi.begin(ssid, password);       //starts WiFi with access authorisation details
  Serial.print("\nWaiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED){ //awaits connection to remote server
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);

  Serial.print("Connecting to ");
  Serial.println(REMOTE_IP);

  while (!client.connect(REMOTE_IP, REMOTE_PORT)) {
    Serial.println("Connection failed.");
    Serial.println("Waiting a moment before retrying...");
  }

  //initialize climate sensor with I2C address
	  bme.begin(0x76, Wire);

	  if(bme.checkStatus())
	  {
		  if (bme.checkStatus() == BME68X_ERROR)
		  {
			  Serial.println("Sensor error:" + bme.statusString());
			  return;
		  }
		  else if (bme.checkStatus() == BME68X_WARNING)
		  {
			  Serial.println("Sensor Warning:" + bme.statusString());
		  }
	  }

	  //sets the default configuration for temperature, pressure and humidity
	  bme.setTPH();

	  //sets sensor heater temperature in degree Celsius 
	  uint16_t tempProf[10] = { 100, 200, 320 };
	  //sets heating duration in milliseconds 
	  uint16_t durProf[10] = { 150, 150, 150 };

	  bme.setSeqSleep(BME68X_ODR_250_MS);
	  bme.setHeaterProf(tempProf, durProf, 3);
	  bme.setOpMode(BME68X_SEQUENTIAL_MODE);
}

void loop() {
  if (client.available() > 0) 
  {
    delay(20);
    //read back one line from the server
    String line = client.readString();
    Serial.println(REMOTE_IP + String(":") + line);
  }
  if (Serial.available() > 0)  
  {
    delay(20);
    String line = Serial.readString();
    client.print(line);
  }
  if (client.connected () == 0) 
  {
    client.stop();
    WiFi.disconnect();
  }
  else{
    sendClimateData();
  }
}

void sendClimateData()
{
  
  bme68xData data;
  uint8_t nFieldsLeft = 0;
	delay(150);

	if (bme.fetchData())
	{
		do
		{
			nFieldsLeft = bme.getData(data);
			//if (data.status == NEW_GAS_MEAS)
			//{
				client.print(String(data.temperature-4.49) + ", " + String(data.humidity) + ", " + String(data.pressure) + '\n');

				if(data.gas_index == 2) /* Sequential mode sleeps after this measurement */
					delay(250);
			//}
		} while (nFieldsLeft);
	}
}

