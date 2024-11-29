#include <cmath>

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
#include "SparkFun_LSM6DSV16X.h" // inertial sensor library not available in PlatformIO library

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

#define BUZZER 7        //connected to pin GP7

#define     REMOTE_IP          "192.168.10.1"  //remote server IP which that you want to connect to
#define     REMOTE_PORT         5263           //connection port provided on remote server 

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

enum SENSOR_MODE {
    ENVIRONMENT,
    OBJECT,
    CUP,
    ACTUATOR,
    TRIPWIRE
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
void computeTiltAndGyro();
float getAccAngleX();
void readIMU();
void calibrateGyro();
double calculateLiquidLeftCup(double angle);
float cupRadius = 5; // can be changed for different cup sizes
float cupHeight = 12.7; // can be changed for different cup sizes
double cupCapacity = M_PI * pow(cupRadius, 2) * cupHeight;
double cupContents = cupCapacity; // assume cup starts full;
int soundIterator = 0;

float gyroRateX = 0.0;
float gyroRateY = 0.0;
float gyroRateZ = 0.0;
float gyroRate = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float gOffX = 0.0;
float gOffY = 0.0;
float gOffZ = 0.0;

SparkFun_LSM6DSV16X myLSM;

// Structs for X,Y,Z data
sfe_lsm_data_t accelData;
sfe_lsm_data_t gyroData;

void sendZoneTriggeredData();       //For sending zone motion data to BaseStation
void redButtonPressed();
void blackButtonPressed();
void changeMode();                  //On button press change sensor mode
void onPDMdata();
float readSoundSamples();
void changeZone();                  //for use in Tripwire mode
void changeObject();                //for use in Object mode

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
    pinMode(BUZZER, OUTPUT);

    attachInterrupt(REDButton, redButtonPressed, FALLING);
    attachInterrupt(BLACKButton, blackButtonPressed, FALLING);

    if (!myLSM.begin()) //starts inertial measurement sensor
    {
        Serial.println("Did not begin, check your wiring and/or I2C address!");
        while (1);
    }

    // Reset the device to default settings. This if helpful is you're doing multiple
    // uploads testing different settings.
    myLSM.deviceReset();

    // Wait for it to finish resetting
    while (!myLSM.getDeviceReset())
    {
        delay(1);
    }

    // Accelerometer and Gyroscope registers will not be updated
    // until read.
    myLSM.enableBlockDataUpdate();

    // Sets the output data rate and precision of the accelerometer
    myLSM.setAccelDataRate(LSM6DSV16X_ODR_AT_7Hz5);
    myLSM.setAccelFullScale(LSM6DSV16X_16g);

    // Sets the output data rate and precision of the gyroscope
    myLSM.setGyroDataRate(LSM6DSV16X_ODR_AT_15Hz);
    //myLSM.setGyroFullScale(LSM6DSV16X_2000dps);
    myLSM.setGyroFullScale(LSM6DSV16X_250dps);

    // Enables filter settling.
    myLSM.enableFilterSettling();

    // Turns on the accelerometer's filter and apply settings.
    myLSM.enableAccelLP2Filter();
    myLSM.setAccelLP2Bandwidth(LSM6DSV16X_XL_STRONG);

    // Turns on the gyroscope's filter and apply settings.
    myLSM.enableGyroLP1Filter();
    myLSM.setGyroLP1Bandwidth(LSM6DSV16X_GY_ULTRA_LIGHT);
    void calibrateGyro();   //calibrates gyroscope 
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
  case TRIPWIRE:
    sendZoneTriggeredData();
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



// Cup stuff...
void sendCupData(){
  while(!connectToBaseStation());
  computeTiltAndGyro();
}

double calcluateLiquidLeftCup(double angle) {
  double maxFrequency = 8800; // starting frequency
  double currentMaxContents = cupCapacity * sin(angle * M_PI / 180);
  if(currentMaxContents > cupCapacity) {
    cupContents = 0;
  }
  if(abs(currentMaxContents) < cupContents) {
    cupContents = abs(currentMaxContents);
    double frequencyToPlay = maxFrequency - 6600 * (cupCapacity-cupContents)/cupCapacity;
    tone(BUZZER, frequencyToPlay);
    delay(50);
    tone(BUZZER, frequencyToPlay * 4/3);
    delay(50);
    noTone(BUZZER);
    return cupCapacity - cupContents;
  }
  
  if(soundIterator % 4 == 0) {
    double frequencyToPlay = maxFrequency - 6600 * (cupCapacity-cupContents)/cupCapacity;
    tone(BUZZER, frequencyToPlay);
    delay(50);
    tone(BUZZER, frequencyToPlay * 4/3);
    delay(50);
    noTone(BUZZER);
    soundIterator = 0;
  }
  soundIterator++;
  return cupCapacity - cupContents;
}

void computeTiltAndGyro()
{
  readIMU();
  gyroX = gyroData.xData - gOffX;
  gyroY = gyroData.yData - gOffY;
  gyroZ = gyroData.zData - gOffZ;

  gyroRateX = (gyroX/131.0)*0.01745329252;
  gyroRateY = (gyroY/131.0)*0.01745329252;
  gyroRateZ = (gyroZ/131.0)*0.01745329252;

  gyroRate = sqrt(sq(gyroRateX)+sq(gyroRateX)+sq(gyroRateX));

  double accelAngle = getAccAngleX()*RAD_TO_DEG + 180.0;
  String angleStr = String(round(accelAngle), 1);

  //display tilt and gyro values values on OLED display
  display.setCursor(0,0);                //sets cursor to Col = 0, Row = 0
  display.print(F("Tilt angle: "));
  display.print(angleStr);
  display.println(F(" deg"));

  display.setCursor(0,20);                //sets cursor to Col = 0, Row = 20

  display.println(F(""));


  double liquidUsed = calcluateLiquidLeftCup(accelAngle);
  display.print(F("ml used: "));
  
  display.print(String(static_cast<int>(round(liquidUsed))));

  display.print("\n\n\nReset: BLACK");

  client.print("C:"+(String)cupContents+"\n");

  display.display();      //this function must be called for the OLED to display values

  delay(150);
}

float getAccAngleX()
{
    float ang = atan2(accelData.yData,accelData.zData);
    return ang;
}

//returns accelerometer and gyroscope data
void readIMU()
{
  if (myLSM.checkStatus())
  {
    myLSM.getAccel(&accelData);
    myLSM.getGyro(&gyroData);
  }
}

void resetVolume() {
  cupContents = cupCapacity;
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Re-calibrating...");
  display.display();
}

//calibrates gyro
void calibrateGyro()
{
  float gTempX = 0.0;
  float gTempY = 0.0;
  float gTempZ = 0.0;
  int i =0;
  while(i<200)
  {
    delay(10);
    if (myLSM.checkStatus())
    {
      myLSM.getGyro(&gyroData);
      gTempX += gyroData.xData;
      gTempY += gyroData.yData;
      gTempZ += gyroData.zData;
      i += 1;
    }
  }
  gOffX = gTempX/200.0;
  gOffY = gTempY/200.0;
  gOffZ = gTempZ/200.0;
}



void sendZoneTriggeredData() {
  motionValue = digitalRead(MOTION_SENSOR);
  if (motionValue == HIGH) {
    if (pirState == LOW) {
      pirState = HIGH;
      while(!connectToBaseStation());
      client.print("T:" + String(zone) + "\n");
    }
  }
  else {
    if (pirState == HIGH) {
      pirState = LOW;
    }
  }
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
        resetVolume();
      break;
    case ACTUATOR:
      
     break;
    case TRIPWIRE:
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
    mode = TRIPWIRE;
    display.println("tripwire mode");
    break;
  case TRIPWIRE:
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
