#include <Arduino.h>
#include <WiFi.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library
#include <SD.h>  // File system library
#include <SPI.h>  // File system for the Pico

#include "SDCard.h" // SD card class
#include "microphone.h" // Include the new microphone header

//-- defines OLED screen dimensions ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # 
#define SCREEN_ADDRESS 0x3C //OLED I2C address

//creates OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SDCard sdCard;

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
void sendClimateData();             //For sending environment data to BaseStation
void sendObjectData();              //For sending object usage data to BaseStation
void sendCupData();                 //For sending water usage data to BaseStation
void redButtonPressed();
void blackButtonPressed();
void changeMode();                  //On button press change sensor mode

void setup() {
  Wire.begin();         //Initializes the Wire library and join the I2C bus as a controller
  Serial.begin(115200); //Sets the data rate in bits per second (baud) for serial data transmission. 

  sdCard.setup();

  // initializes OLED display 
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    display.println(F("SSD1306 allocation failed"));
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Initialize microphone
  initMicrophone();

  display.clearDisplay();                 //clears OLED screen
  display.setTextSize(1);                 //Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);    //Draw white text
  display.setCursor(0,0);

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);              //sets WiFi as station/client
  WiFi.setHostname("Group6Station");
  WiFi.begin(ssid, password);       //starts WiFi with access authorization details
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  display.println("WiFi connected");
  display.println(WiFi.localIP());
  display.display();
  delay(500);

  // Connect to server
  client.connect(REMOTE_IP, REMOTE_PORT);

  // Initialize climate sensor with I2C address
  bme.begin(0x76, Wire);

  if (bme.checkStatus() == BME68X_ERROR) {
    display.println("Sensor error:" + bme.statusString());
    return;
  }

  //sets the default configuration for temperature, pressure and humidity
  bme.setTPH();

  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);
  
  attachInterrupt(REDButton, redButtonPressed, FALLING);
  attachInterrupt(BLACKButton, blackButtonPressed, FALLING);
}

void loop() {
  display.clearDisplay();                 //clears OLED screen
  display.setCursor(0,0);

  if (client.available() > 0) {
    String line = client.readString();
    display.println(line);
    Serial.println(line);
  }

  if (Serial.available() > 0) {
    String line = Serial.readString();
    client.print(line);
  }

  if (client.connected () == 0) {
    client.stop();
    WiFi.disconnect();
  } else {
    switch (mode) {
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
      // actuator code
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

void sendClimateData() {
  bme68xData data;
  uint8_t nFieldsLeft = 0;
  delay(150);

  if (bme.fetchData()) {
    do {
      nFieldsLeft = bme.getData(data);
      float temperature = data.temperature - 4.49;
      float humidity = data.humidity;
      float pressure = data.pressure;

      // Get microphone decibel data
      float decibels = readSoundSamples();

      // Build data line with decibel level appended
      String dataLine = String(temperature) + ", " + String(humidity) + ", " + String(pressure) + ", " + String(decibels);
      client.print(dataLine + '\n');
      sdCard.writeData(dataLine);  // write data to SD card

      // Display and send data
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Data: ");
      display.println(dataLine);
      display.display();
    } while (nFieldsLeft);
  }
}

void sendObjectData() {
  //write code here
}

void sendCupData() {
  //write code here
}

void redButtonPressed() {
  changeMode();
}

void blackButtonPressed() {
  // code for black button
}

void changeMode() {
  display.clearDisplay();
  display.setCursor(0,0);
  switch (mode) {
    case ENVIRONMENT: 
      mode = OBJECT;
      display.println("object mode");
      break;
    case OBJECT:
      mode = CUP;
      display.println("water mode");
      break;
    case CUP:
      mode = ENVIRONMENT;
      display.println("environment mode");
      break;
    default:
      break;
  }
  display.display();
}
