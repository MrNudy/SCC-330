#include <Arduino.h>
#include <WiFi.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library
#include <SD.h>  // File system library
#include <SPI.h>  // File system for the Pico
#include "SDCard.h" // SD card class

#include <PDM.h>
#include "microphone.h"

//-- defines OLED screen dimensions ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin #
#define SCREEN_ADDRESS 0x3C //OLED I2C address

// Create OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SDCard sdCard;

#define REDButton 12     // Connected to pin GP12
#define BLACKButton 13   // Connected to pin GP13

#define REMOTE_IP "192.168.10.1"  // Remote server IP which you want to connect to
#define REMOTE_PORT 5263           // Connection port provided on remote server

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

enum SENSOR_MODE {
  ENVIRONMENT,
  OBJECT,
  CUP,
  ACTUATOR
};
SENSOR_MODE mode = ENVIRONMENT;

const char* ssid = "Group6BaseStation";    // Access Point SSID
const char* password = "group6best"; // Access Point Password

Bme68x bme;                         // Declares climate sensor variable
WiFiClient client;                  // Declares WiFi client

// Declare microphone-related variables
#define SAMPLE_BUFFER_SIZE 256
#define BIT_PER_SAMPLE 16
#define FREQUENCY 16000  // Microphone frequency
extern short sampleBuffer[SAMPLE_BUFFER_SIZE];  // Buffer for microphone data
extern int samplesRead;                         // Number of samples read

// Function prototypes
void initializeMicrophone();
void onPDMdata();
float getDecibels();
void sendClimateData();
void sendObjectData();
void sendCupData();
void changeMode();
void redButtonPressed();
void blackButtonPressed();





void setup() {
  Wire.begin();         // Initializes the Wire library and join the I2C bus as a controller
  Serial.begin(115200); // Sets the data rate for serial data transmission

  sdCard.setup();

  // Initializes OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    display.println(F("SSD1306 allocation failed"));
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.display();    // This function must be called at the end of display statements
  display.clearDisplay();                 // Clears OLED screen
  display.setTextSize(1);                 // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);    // Draw white text
  display.setCursor(0, 0);

  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);              // Sets WiFi as station/client
  WiFi.setHostname("Group6Station");

  WiFi.begin(ssid, password);       // Starts WiFi with access authorization details
  Serial.print("\nWaiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) { // Awaits connection to remote server
    display.print("\nWaiting for WiFi");
    display.display();
    delay(200);
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

  // Initialize climate sensor with I2C address
  bme.begin(0x76, Wire);

  if (bme.checkStatus()) {
    if (bme.checkStatus() == BME68X_ERROR) {
      display.println("Sensor error:" + bme.statusString());
      Serial.println("Sensor error:" + bme.statusString());
      return;
    } else if (bme.checkStatus() == BME68X_WARNING) {
      display.println("Sensor Warning:" + bme.statusString());
      Serial.println("Sensor Warning:" + bme.statusString());
    }
  }

  // Sets the default configuration for temperature, pressure, and humidity
  bme.setTPH();
  bme.setOpMode(BME68X_SEQUENTIAL_MODE);

  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);
  attachInterrupt(REDButton, redButtonPressed, FALLING);
  attachInterrupt(BLACKButton, blackButtonPressed, FALLING);

  // Initialize microphone
  initializeMicrophone();
}

void loop() {
  display.clearDisplay();                 // Clears OLED screen
  display.setCursor(0, 0);

  if (client.available() > 0) {
    delay(20);
    // Read back one line from the server
    String line = client.readString();
    display.println(REMOTE_IP + String(":") + line);
    Serial.println(REMOTE_IP + String(":") + line);
  }

  if (Serial.available() > 0) {
    delay(20);
    String line = Serial.readString();
    client.print(line);
  }

  if (client.connected() == 0) {
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
        // Code for acting as actuator goes here
        break;
      default:
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Sensor mode error");
        display.display();
        delay(300);
    }
  }
}

void displayData(String dataLine) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(dataLine);
  display.display();
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

      // Get microphone decibel value
      float decibels = getDecibels();

      // Create data line with sensor data and microphone decibels
      String dataLine = String(temperature) + ", " + String(humidity) + ", " + String(pressure) + ", " + String(decibels);

      client.print(dataLine + '\n');

      sdCard.writeData(dataLine);  // Write data to SD card

      String* fileData = sdCard.getData();
      for (int i = 0; i < 287; i++) {  // DATA_SIZE from SDCard.h
        if (fileData[i] != "") {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("getting " + String(i));
          delay(1000);
          display.print(fileData[i]);
          display.display();
        } else {
          break;
        }
      }
    } while (nFieldsLeft);
  }
}

// Microphone-related functions
void initializeMicrophone() {
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, FREQUENCY)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;  // Each sample is 2 bytes (16 bits)
}

float getDecibels() {
  long sumSquares = 0;
  for (int i = 0; i < samplesRead; i++) {
    sumSquares += sampleBuffer[i] * sampleBuffer[i];  // Sum of squares of sample values
  }

  if (samplesRead > 0) {
    float rms = sqrt(sumSquares / float(samplesRead));  // Root Mean Square of the samples
    // Convert RMS to Decibels (dB)
    float decibels = 20 * log10(rms / 32768.0);  // Assuming 16-bit signed samples, max value is 32768
    return decibels;
  } else {
    return 0.0;  // Return 0 if no samples
  }
}

void sendObjectData() {
  // Write code for object data here
}

void sendCupData() {
  // Write code for cup data here
}

void redButtonPressed() {
  changeMode();
}

void blackButtonPressed() {
  switch (mode) {
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
      display.setCursor(0, 0);
      display.println("Sensor mode error");
      display.display();
      delay(300);
  }
}

void changeMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  switch (mode) {
    case ENVIRONMENT:
      mode = OBJECT;
      display.println("Object mode");
      break;
    case OBJECT:
      mode = CUP;
      display.println("Cup mode");
      break;
    case CUP:
      mode = ACTUATOR;
      display.println("Actuator mode");
      break;
    case ACTUATOR:
      mode = ENVIRONMENT;
      display.println("Environment mode");
      break;
    default:
      display.println("Mode error");
  }
  display.display();
  delay(300);
}