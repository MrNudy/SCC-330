#include <Arduino.h>
#include <WiFi.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
                                   //Library added to lib folder on the left
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library
#include <TimeLib.h>

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

//-- defines OLED screen dimensions ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # 
#define SCREEN_ADDRESS 0x3C //OLED I2C address

//creates OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REDButton 12     //connected to pin GP12
#define BLACKButton 13   //connected to pin GP13

#define     REMOTE_IP          "192.168.10.1"  //remote server IP which that you want to connect to
#define     REMOTE_PORT         5263           //connection port provided on remote server 

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

enum SENSOR_MODE{
  ENVIRONMENT,
  OBJECT,
  CUP,
  ACTUATOR
};
SENSOR_MODE mode = ENVIRONMENT;


const char* ssid = "Group6BaseStation";    //Access Point SSID
const char* password= "group6best"; //Access Point Password

Bme68x bme;                         //declares climate sensor variable
WiFiClient client;                  //declares WiFi client

//declare functions implemented
void sendClimateData();             //For sending enrironment data to BaseStation
void sendObjectData();              //For sending object usage data to BaseStation
void sendCupData();                 //For sending water usage data to BaseStation
void redButtonPressed();
void blackButtonPressed();
void changeMode();                  //On button press change sensor mode
time_t requestSync();
void processSyncMessage();


void setup() {
  setSyncProvider( requestSync);  //set function to call when sync required
  Wire.begin();         //Initializes the Wire library and join the I2C bus as a controller
                          //or a peripheral. It is normally be called only once.
    Serial.begin(115200); //Sets the data rate in bits per second (baud) for serial data transmission. 
                          //For communicating with Serial Monitor, make sure to use one of the baud ...

    // initializes OLED display 
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
    {
          display.println(F("SSD1306 allocation failed"));
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

     // OLED display library initializes this with an Adafruit splash screen.
     display.display();    //this function must be called at the end of display statements

     // clears the display buffer
  display.clearDisplay();                 //clears OLED screen
  display.setTextSize(1);                 //Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);    //Draw white text
  display.setCursor(0,0);
  // while (!Serial); // Wait until serial is available
  delay(6);//added because this is the minimum time I found that gets all serial message to print(no idea why the line above doesn't fully work)
 

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);              //sets WiFi as station/client
  WiFi.setHostname("Group6Station");
 

  WiFi.begin(ssid, password);       //starts WiFi with access authorisation details
  Serial.print("\nWaiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED){ //awaits connection to remote server
    display.print("\nWaiting for WiFi");
    display.display();
    delay(200);
    display.print(".");
    //Serial.print(".");
    display.display();
    delay(200);
    display.print(".");
    //Serial.print(".");
    display.display();
    delay(200);
    display.print(".");
    //Serial.print(".");
    display.display();
    delay(200);
    display.clearDisplay();                 //clears OLED screen
    display.setCursor(0,0);
    display.display();
  }

  display.println("");
  Serial.println("");
  display.println("WiFi connected");
  Serial.println("WiFi connected");
  display.println("IP address: ");
  Serial.println("IP address: ");
  display.println(WiFi.localIP());
  Serial.println(WiFi.localIP());
  display.display();
  delay(500);

  display.print("Connecting to ");
  Serial.print("Connecting to ");
  display.println(REMOTE_IP);
  Serial.println(REMOTE_IP);
  display.display();

  while (!client.connect(REMOTE_IP, REMOTE_PORT)) {
      display.println("Connection failed.");
    Serial.println("Connection failed.");
      display.println("Waiting a moment before retrying...");
    Serial.println("Waiting a moment before retrying...");
  }

  //initialize climate sensor with I2C address
	  bme.begin(0x76, Wire);

	  if(bme.checkStatus())
	  {
		  if (bme.checkStatus() == BME68X_ERROR)
		  {
              display.println("Sensor error:" + bme.statusString());
        Serial.println("Sensor error:" + bme.statusString());
			  return;
		  }
		  else if (bme.checkStatus() == BME68X_WARNING)
		  {
              display.println("Sensor Warning:" + bme.statusString());
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

  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);
    
  attachInterrupt(REDButton, redButtonPressed, FALLING);
  attachInterrupt(BLACKButton, blackButtonPressed, FALLING);
}

void loop() {
  
  display.clearDisplay();                 //clears OLED screen
  display.setCursor(0,0);

  if (client.available() > 0) 
  {
    delay(20);
    //read back one line from the server
    String line = client.readString();
      display.println(REMOTE_IP + String(":") + line);
    Serial.println(REMOTE_IP + String(":") + line);
  }
  if (Serial.available() > 0)  
  {
    delay(20);
    String line = Serial.readString();
    client.print(line);
    processSyncMessage();
  }
  if (client.connected () == 0) 
  {
    client.stop();
    WiFi.disconnect();
  }
  else{
    switch (mode){
    case ENVIRONMENT: 
      sendClimateData();
      break;
    case OBJECT:
        sendObjectData();
      break;
    case CUP:
        sendCupData();
      break;
    case ACTUATOR:
      //code for acting as actuator goes here
     break;
    default:
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Sensor mode error");
      display.display();
      delay(300);
    }
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
				client.print(String(hour())+" "+String(minute())+" "+String(second()) + '\n');
        delay(1000);
				// if(data.gas_index == 2) /* Sequential mode sleeps after this measurement */
				// 	delay(250);
			//}
		} while (nFieldsLeft);
	}
}

void sendObjectData(){
  //write code here
}

void sendCupData(){
  //write code here
}

void redButtonPressed(){
  changeMode();
}

void blackButtonPressed(){ //Anyone who wants an input for thier sensor mode use the black button
  switch (mode){
    case ENVIRONMENT: 
      
      break;
    case OBJECT:
        
      break;
    case CUP:
        
      break;
    case ACTUATOR:
      
     break;
    default:
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Sensor mode error");
      display.display();
      delay(300);
    }
}

void changeMode(){
  display.clearDisplay();
  display.setCursor(0,0);
  switch (mode){
    case ENVIRONMENT: 
      mode = OBJECT;
      display.println("object mode");
      break;
    case OBJECT:
      mode = CUP;
      display.println("cup mode");
      break;
    case CUP:
      mode = ACTUATOR;
      display.println("actuator mode");
      break;
    case ACTUATOR:
      mode = ENVIRONMENT;
      display.println("environment mode");
     break;
    default:
      display.println("mode error");
    }
  display.display();
  delay(300);
}
void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

