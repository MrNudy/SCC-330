#include <Arduino.h>
#include <WiFi.h>
#include <PDM.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
#include "BH1745NUC.h"          //light measurement sensor library
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library
#include <SD.h>  // File system library
#include <SPI.h>  // File system for the Pico
#include "SDCard.h" // SD card class

//-- defines OLED screen dimensions ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # 
#define SCREEN_ADDRESS 0x3C //OLED I2C address

//creates OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


SDCard sdCard;
bool readFromFile = false;

#define REDButton 12     //connected to pin GP12
#define BLACKButton 13   //connected to pin GP13

#define MOTION_SENSOR 11 //connected to pin GP11

#define     REMOTE_IP          "192.168.10.1"  //remote server IP which that you want to connect to
#define     REMOTE_PORT         5263           //connection port provided on remote server 

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

enum SENSOR_MODE {
    ENVIRONMENT,
    OBJECT,
    CUP,
    ACTUATOR,
    PERSON
};
SENSOR_MODE mode = ENVIRONMENT;

enum OBSERVING_OBJECT {
    CHAIR,
    DOOR,
    RUBBISH_BIN,//BIN is already a keyword
    TABLE,
    CUPBOARD
};
OBSERVING_OBJECT object = DOOR;

int pirState = LOW;             // keeps track of PIR state, LOW (0) when no motion is detected and HIGH (1) when
// motion is detected. Initialised to no motion detected
int motionValue = 0;                  // variable to store the sensor value
int zone = 1;                         //stores what zone sensor is in

const char* ssid = "Group6BaseStation";    //Access Point SSID
const char* password = "group6best"; //Access Point Password

Bme68x bme;                         //declares climate sensor variable
WiFiClient client;                  //declares WiFi client

//----defines light light measurement sensor parameters
#define BH1745NUC_DEVICE_ADDRESS_38 0x38    //light measurement sensor I2C address
BH1745NUC bh1745nuc = BH1745NUC();

//----defines microphone measurement sensor parameters
short sampleBuffer[512];
volatile int samplesRead;

//declare functions implemented
bool connectToBaseStation();
void connectWiFi();
void sendClimateData();             //For sending enrironment data to BaseStation
void sendObjectData();              //For sending object usage data to BaseStation
void sendCupData();                 //For sending water usage data to BaseStation
void sendZoneTriggeredData();       //For sending zone motion data to BaseStation
void sendLocationData();            //For sending live location data to BaseStation
void redButtonPressed();
void blackButtonPressed();
void changeMode();                  //On button press change sensor mode
void onPDMdata();
float readSoundSamples();
void changeZone();                  //for use in Tripwire mode
void changeObject();                //for use in Object mode

const float accessPointPositions[6][2] = {
  {1908, 0},
  {1603, 768},
  {1353, 0},
  {850, 0},
  {550, 768},
  {200, 0}
};
float RSSIPosition[20][2];
int readingsIterator = 0;

float RSSIsForAverage[20];
int tempIterator = 0;

void setup() {
    Wire.begin();         //Initializes the Wire library and join the I2C bus as a controller
    //or a peripheral. It is normally be called only once.
    Serial.begin(115200); //Sets the data rate in bits per second (baud) for serial data transmission. 
    //For communicating with Serial Monitor, make sure to use one of the baud ...

    sdCard.setup();

// initializes OLED display 
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        display.println(F("SSD1306 allocation failed"));
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
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

  //initialize climate sensor with I2C address
	  bme.begin(0x76, Wire);

 
  connectToBaseStation();
    if (bme.checkStatus())
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

    //start light level measurements
    bh1745nuc.begin(BH1745NUC_DEVICE_ADDRESS_38);
    bh1745nuc.startMeasurement();

    //start sound level measurements
    PDM.onReceive(onPDMdata);
    PDM.setCLK(3);
    PDM.setDIN(2);
    if(!PDM.begin(1, 16000)){
      display.clearDisplay();
      display.setCursor(0,0);
      display.write("Failed to start PDM");
      display.display();
      while(1);
    }

    pinMode(REDButton, INPUT);
    pinMode(BLACKButton, INPUT);
    pinMode(MOTION_SENSOR, INPUT);

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
  }
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
  case PERSON:
    sendLocationData();
    break;
  default:
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sensor mode error");
    display.display();
    delay(300);
  }
}

void displayData(String dataLine){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(dataLine);
  display.display();
}

const char *macToString(uint8_t mac[6]) {
  static char s[20];
  sprintf(s, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return s;
}

const char *encToString(uint8_t enc) {
  switch (enc) {
    case ENC_TYPE_NONE: return "NONE";
    case ENC_TYPE_TKIP: return "WPA";
    case ENC_TYPE_CCMP: return "WPA2";
    case ENC_TYPE_AUTO: return "AUTO";
  }
  return "UNKN";
}

double distanceFromRSSI(int RSSI) {
  float strength1m = -40;
  float n = 1.29; // path loss exponent - rough assumption for now
  return pow(10, (strength1m - RSSI) / (10 * n));
}

void calculatePosition(int rssiValues[6], float& xPos, float& zPos) {
  float distances[6];
  for(int i = 0; i < 6; i++)
    distances[i] = distanceFromRSSI(rssiValues[i]);

  // will use weighted average of positions using inverse distance squared
  float sumX = 0;
  float sumY = 0;
  float sumWeight = 0;
  for(int i = 0; i < 6; i++) {
    float weight = 1 / (distances[i] * distances[i]); // inverse square
    sumX += accessPointPositions[i][0] * weight;
    sumY += accessPointPositions[i][1] * weight;
    sumWeight += weight;
  }
  xPos = sumX / sumWeight;
  zPos = sumY / sumWeight;
}

void sendClimateData()
{
  bme68xData data;
  bme.fetchData();
  while(bme.getData(data));
  bh1745nuc.read();
  float sound = readSoundSamples();
  while(!connectToBaseStation());
  client.printf("E:%d,%d,%f,%f\n",zone,bh1745nuc.clear,sound,data.temperature-4.49);
  String dataLine = String(zone) + ", " + bh1745nuc.clear + ", " + sound + ", " + String(data.temperature-4.49); // later date remove spaces for parsing

  sdCard.writeData(dataLine);

  if (readFromFile == true) {
        String* fileData = sdCard.getData();
        for(int i = 0; i < 287; i++){  //DATA_SIZE from SDCard.h
          if (fileData[i] != ""){
            if (readFromFile == false){
              break;
            } else {
              display.clearDisplay();
              display.setCursor(0,0);
              display.println("getting " + String(i));
              delay(1000);
              display.print(fileData[i]);
              display.display();
            }
          }else{
              break;
            };
        }
      } else {
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Live");
          delay(1000);
          display.print(dataLine);
          display.display();
      }
      
}

void sendObjectData(){
  motionValue = digitalRead(MOTION_SENSOR);
  if (motionValue == HIGH) {
    if (pirState == LOW) {
      pirState = HIGH;
      while(!connectToBaseStation());
      display.clearDisplay();
      display.setCursor(0,0);
      client.print("O:"+(String)object+"\n");
      display.println("Motion detected");
      display.display();
    }
  }
  else {
    if (pirState == HIGH) {
      pirState = LOW;
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("No Motion detected");
      display.display();
    }
  }
}

void sendCupData(){
  while(!connectToBaseStation());
  //write code here
}

void sendLocationData() {
  Serial.printf("Beginning scan at %lu\n", millis());
  auto cnt = WiFi.scanNetworks();
  if (!cnt) {
    Serial.printf("No networks found\n");
  } else {
    int rssiValues[6];

    Serial.printf("Found %d networks\n\n", cnt);
    Serial.printf("%32s %5s %17s %2s %4s\n", "SSID", "ENC", "BSSID        ", "CH", "RSSI");
    for(int i = 0; i < cnt; i++) {
      uint8_t bssid[6];
      if(strncmp(WiFi.SSID(i), "SATB074", strlen("SATB074")) == 0) {
        WiFi.BSSID(i, bssid);
        int thisRSSI = WiFi.RSSI(i);
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '1')
          rssiValues[0] = thisRSSI;
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '2')
          rssiValues[1] = thisRSSI;
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '3') 
          rssiValues[2] = thisRSSI;
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '4') 
          rssiValues[3] = thisRSSI;
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '5') 
          rssiValues[4] = thisRSSI;
        if(WiFi.SSID(i)[strlen(WiFi.SSID(i))-1] == '6') 
          rssiValues[5] = thisRSSI;

        const char *thisSSID = WiFi.SSID(i);
        Serial.printf("%32s %5s %17s %2d %4ld\n", WiFi.SSID(i), encToString(WiFi.encryptionType(i)), macToString(bssid), WiFi.channel(i), WiFi.RSSI(i));
        display.print(String(WiFi.SSID(i)));
        display.print(F(" "));
        display.println(WiFi.RSSI(i));
      }     
    }

  
    float xPos, zPos;
    calculatePosition(rssiValues, xPos, zPos);
    RSSIPosition[readingsIterator][0] = xPos;
    RSSIPosition[readingsIterator][1] = zPos;

    for(int i = 0; i < sizeof(RSSIPosition)/sizeof(RSSIPosition[0]); i++) {
      if(RSSIPosition[i] == NULL) {
        readingsIterator = (readingsIterator + 1) % 20;
        continue;
      }
    }

    float totalXPos = 0;
    float totalZPos = 0;
    for(int i = 0; i < 20; i++) {
      totalXPos += RSSIPosition[i][0];
      totalZPos += RSSIPosition[i][1];
    }
    float averageXPos = totalXPos / 20;
    float averageZPos = totalZPos / 20;

    Serial.printf("User Position: X = %.2f, Y = %.2f\n", averageXPos, averageZPos);
    display.printf("X:%.2f Z:%.2f\n", averageXPos, averageZPos);
    display.display();

    readingsIterator = (readingsIterator + 1) % 20;
  }
  Serial.printf("\n--- Sleeping ---\n\n\n");
  delay(500);
}

void blackButtonPressed(){//Anyone who wants an input for thier sensor mode use the black button
    if (readFromFile == true) {
      readFromFile = false;
    } else {
        readFromFile = true;
    }
  switch (mode){
    case OBJECT:
        changeObject();
      break;
    case CUP:
        
      break;
    case ACTUATOR:
      
     break;
    case PERSON:
    case ENVIRONMENT:
        changeZone();
      break;
    default:
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Sensor mode error");
      display.display();
      delay(300);
    }
}

void redButtonPressed(){
  changeMode();
}

void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

float calculateDecibels() {
  float sum = 0;
  for (int i = 0; i < samplesRead; i++) {
    sum += abs(sampleBuffer[i]);
  }
  float average = sum / samplesRead;
  return -20.0f * log10(average / 32767.0f); // Convert average value to decibels
}

// Function to read sound samples and calculate decibel level
float readSoundSamples() {
  if (samplesRead > 0) {
    // Calculate and return the decibel level
    float decibels = calculateDecibels();
    samplesRead = 0; // Reset the sample count after processing
    return decibels;
  }
  return -1; // Return -1 if no samples are available
}

void changeMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  switch (mode) {
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
    mode = PERSON;
    display.println("stalk mode");
    break;
  case PERSON:
    mode = ENVIRONMENT;
    display.println("environment mode");
    break;
  default:
    display.println("mode error");
  }
  display.display();
  delay(300);
}

void changeObject(){
  display.clearDisplay();
  display.setCursor(0, 0);
  switch(object){
    case CHAIR:
      object = DOOR;
      display.println("Observing Door");
      break;
    case DOOR:
      object = RUBBISH_BIN;
      display.println("Observing Bin");
      break;
    case RUBBISH_BIN:
      object = TABLE;
      display.println("Observing Table");
      break;
    case TABLE:
      object = CUPBOARD;
      display.println("Observing Cupboard");
      break;
    case CUPBOARD:
      object = CHAIR;
      display.println("Observing Chair");
      break;
    default:
      display.println("object error");
  }
  display.display();
}

void changeZone(){
  display.clearDisplay();
  display.setCursor(0, 0);
  switch(zone){
    case 1:
      zone = 2;
      display.println("Zone 2");
      break;
    case 2:
      zone = 3;
      display.println("Zone 3");
      break;
    case 3:
      zone = 1;
      display.println("Zone 1");
      break;
    default:
      display.println("zone error");
  }
  display.display();
}

bool connectToBaseStation(){
  while (!client.connect(REMOTE_IP, REMOTE_PORT)) {
    display.clearDisplay();                 //clears OLED screen
    display.setCursor(0,0);
    display.print("\nWaiting for BaseStation");
    display.display();
    if(WiFi.status() != WL_CONNECTED){
      connectWiFi();
    }
  }
  return true;
}

void connectWiFi(){
  WiFi.begin(ssid, password);       //starts WiFi with access authorisation details
  while (WiFi.status() != WL_CONNECTED){ //awaits connection to remote server
    display.clearDisplay();                 //clears OLED screen
    display.setCursor(0,0);
    display.display();
    display.print("\nWaiting for WiFi");
    display.display();
    delay(200);
  }
}
