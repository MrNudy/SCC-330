#include <Arduino.h>

#include "BH1745NUC.h"          //light measurement sensor library
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library

// screen setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// button setup
#define REDButton 12
#define BLACKButton 13
// led setup
#define detector LED_BUILTIN
// motion sensor setup
int buzzer = 7;
int sensorPIN = 11;
int pirState = LOW;

int value = 0;

// light sensor setup
#define BH1745NUC_DEVICE_ADDRESS_38 0x38
BH1745NUC sensor = BH1745NUC();

// button press flags
//volatile bool redButtonPressed = false;
//volatile bool blackButtonPressed = false;

// mode tracking
enum Mode {MOTION, LIGHT};
volatile Mode currentMode = MOTION;

//function declaration
void redPressed();
void blackPressed();
void turnLED_ON();
void turnLED_OF();

//different applications?
void motionSensor();
void readLightMeasurements();

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  pinMode(detector, OUTPUT);
  pinMode(sensorPIN, INPUT);
  pinMode(REDButton, INPUT);
  pinMode(BLACKButton, INPUT);

  attachInterrupt(digitalPinToInterrupt(REDButton), redPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BLACKButton), blackPressed, FALLING);


// initialise oled
if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
{
  Serial.println(F("SSD1306 allocation failed"));
  for(;;);
}

display.display();
delay(1000);

display.clearDisplay();

//light level measurements
sensor.begin(BH1745NUC_DEVICE_ADDRESS_38);
sensor.startMeasurement();

}

void loop()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  if (currentMode == MOTION)
  {
    motionSensor();
  } 
  else if (currentMode == LIGHT)
  {
    readLightMeasurements();
  }

}

// button presses
void redPressed()
{
  currentMode = MOTION;
  Serial.println("Switched to Motion Sensor Mode.");
  display.clearDisplay();
  display.println("Motion Sensor Mode:");
  display.display();
}

void blackPressed()
{
  currentMode = LIGHT;  // Switch to light measurement mode
  Serial.println("Switched to Light Sensor Mode");
  display.clearDisplay();
  display.println("Light Sensor Mode");
  display.display();
}

//LED FUNCTIONS
void turnLED_ON()
{
  digitalWrite(detector,HIGH);
}

void turnLED_OFF()
{
  digitalWrite(detector, LOW);
}

// MOTION SENSOR
void motionSensor()
{
  value = digitalRead(sensorPIN);
  if (value == HIGH)
  {
    turnLED_ON();
    if(pirState == LOW)
    {
      Serial.println("Motion Detected!");
      display.println("Motion Detected!!");
      pirState = HIGH;
      display.display();
    }
  }
  else
  {
    turnLED_OFF();
    if (pirState == HIGH)
    {
      Serial.println("Motion stopped!");
      display.println("Motion stopped!!");
      pirState = LOW;
      display.display();
    }
  }

}

// LIGHT SENSOR
void readLightMeasurements()
{
  if (!sensor.read())  //attempts to read light measurement values
  { 
    Serial.println("Light sensor failed to read measurement values!");
    delay(500);
    return;
  }
  unsigned short rgbc[4];    //creates a 4 value array
  rgbc[0] = sensor.red;
  rgbc[1] = sensor.green;
  rgbc[2] = sensor.blue;
  rgbc[3] = sensor.clear;

  //print light values on the serial monitor
  Serial.write("Red = ");
  Serial.print(rgbc[0]);
  Serial.write(" Green = ");
  Serial.print(rgbc[1]);
  Serial.write(" Blue = ");
  Serial.print(rgbc[2]);
  Serial.write(" Clear = ");
  Serial.println(rgbc[3]);  

  //display light values in OLED display
  display.println(F("Light measurements.. "));
  display.print(F("Red:   "));
  display.println(String(rgbc[0]));
  display.print(F("Green: "));
  display.println(String(rgbc[1]));
  display.print(F("Blue:  "));
  display.println(String(rgbc[2]));
  display.print(F("Clear: "));
  display.println(String(rgbc[3]));
  display.display();
}
