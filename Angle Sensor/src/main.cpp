#include <Arduino.h>
#include "SparkFun_LSM6DSV16X.h" //inertial sensor library not available in PlatformIO libraries.
                                 //library added to lib folder
#include "Wire.h"
#include <Adafruit_GFX.h>       //OLED display support library
#include <Adafruit_SSD1306.h>   //OLED display library

//-- defines OLED screen dimensions ---
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # 
#define SCREEN_ADDRESS 0x3C //OLED I2C address

//define black button
#define BLACKButton 13

//creates OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RAD_TO_DEGREES 57.2957795131

//initialize gyroscope values
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

SparkFun_LSM6DSV16X myLSM;  //creates inertial sensor object

// Structs for X,Y,Z data
sfe_lsm_data_t accelData;
sfe_lsm_data_t gyroData;

//--- declare functions
float getAccAngleX();
void readIMU();
// void calibrateGyro();
void computeTiltAndGyro();
void blackButtonPressed();


void setup()
{
    Wire.begin();
    Serial.begin(115200);
    pinMode(BLACKButton, INPUT);
    attachInterrupt(digitalPinToInterrupt(BLACKButton), blackButtonPressed, FALLING);
    
    // initializes OLED display 
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
    {
      Serial.println(F("SSD1306 allocation failed"));
      while (1); // Don't proceed, loop forever
    }

     // OLED display library initializes this with an Adafruit splash screen.
     display.display();    //this function must be called at the end of display statements
     delay(2000);          // pauses for 2 seconds

     // clears the display buffer
     display.clearDisplay();

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

void loop()
{
  display.clearDisplay();                 //clears OLED screen
  display.setTextSize(1);                 //Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);    //Draw white text
  
  computeTiltAndGyro();        //computes tilt with device standing vertically on a flat surface. The Pico should be at 
                         //the bottom and the OLED end at top. Gently tilt the device and observe the angle change
  delay(100);
}

void blackButtonPressed(){
  Serial.println("Cup refilled");
  display.clearDisplay();
  delay(500);

  //erase values stored for calcualting volume
}

void redButtonPressed(){ //potentially used to change the variable that holds the cup details
  Serial.println("Attached to new Cup");
  display.clearDisplay();
  delay(500);

  
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

  float accelAngle = getAccAngleX()*RAD_TO_DEG + 180.0;
  String angleStr = String(accelAngle, 1);

  Serial.print("Tilt Angle: ");
  Serial.print(angleStr);
  Serial.println(" deg");
 
  // Serial.print("GryoX: ");
  // Serial.print(gyroRateX);
  // Serial.print(" GryoY: ");
  // Serial.print(gyroRateY);
  // Serial.print(" GryoZ: ");
  // Serial.print(gyroRateZ);
  // Serial.print(" GryoVector: ");
  // Serial.println(gyroRate);

  //display tilt and gyro values values on OLED display
  display.setCursor(0,0);                //sets cursor to Col = 0, Row = 0
  display.print(F("Tilt angle: "));
  display.print(angleStr);
  display.println(F(" deg"));


  //variables used to show the tilt (should be made to change with the angle)
  ///NEED TO FIGURE OUT THE MATHS OF THE TILTING (USE GYRO VALUES??)
  /*
    (x1,y1)         (x3,y3)
        \             /
         \           /
          \         /
           \_______/
        (x2,y2) (x4,y4)
  when flat, y1 = y3 && y2 = y4, but when tilting will be different
  */
  int x1;
  int y1;
  int x2;
  int y2;
  int x3;
  int y3;
  int x4;
  int y4;

  display.drawLine(34 + x1, 27 + y1, 38 + x2, 52 + y2, SSD1306_WHITE); //left vertical line 
  display.drawLine(64 + x3, 27 + y3, 60 + x4, 52 + y4, SSD1306_WHITE); //right vertical line
  display.drawLine(38 + x2, 52 + y2, 60 + x4, 52 + y4, SSD1306_WHITE); //horizontal line

  ///DESIRABLE: for loop to draw a wavy line for fluid in cup, use many lines starting at fixed point on the left line,
   //moving up and down y to create wiggle and finishing on fixed point on right line

  // display.setCursor(0,20);                //sets cursor to Col = 0, Row = 20
  // display.print(F("GyroX: "));
  // display.println(String(gyroRateX));
  // display.print(F("GyroY: "));
  // display.println(String(gyroRateY));
  // display.print(F("GyroZ: "));
  // display.println(String(gyroRateZ));
  // display.println(F(""));
  // display.print(F("GyroVector: "));        //ove
  // display.println(String(gyroRate));

  display.display();      //this function must be called for the OLED to display values
}

//computes sensor tilt angle on the x-axis using accelerometer
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

