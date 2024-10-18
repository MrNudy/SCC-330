#include <Arduino.h>

#include "Wire.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "bme68xLibrary.h"         //This library is not available in PlatformIO
                                   //Library added to lib folder on the left
//--------------------
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)


Bme68x bme;  //declares climate sensor variable

//------
//define OLED screen dimensions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # 
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//create OLED display object "display"
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//declare functions
 void readClimateValues();

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
    {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

     // the library initializes this with an Adafruit splash screen.
     display.display();
     delay(2000); // Pause for 2 seconds

     // Clear the OLED display buffer
     display.clearDisplay();

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

	  //Serial.println("Temperature(deg C), Pressure(Pa), Humidity(%)");
    //--------------
}

void loop()
{
  readClimateValues();                 //reads climate measurements
  delay(1000);
}

void readClimateValues()
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
			{
				Serial.print(String(data.temperature-4.49) + ", ");
				Serial.print(String(data.pressure) + ", ");
				Serial.println(String(data.humidity) + ", ");
				
        display.clearDisplay();                 //clear OLED screen
        display.setTextSize(1);                 //Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);    //Draw white text
        display.setCursor(0,0);                 //Start at top-left corner (Col=0, Row=0)
       
        display.print(F("Temp:     "));
        display.print(String(data.temperature-4.49));
        display.println(F("'C"));
        display.print(F("Humidity: "));
        display.print(String(data.humidity,2));
        display.println(F("%"));
        display.print(F("Pressure: "));
        display.print(String(data.pressure,2));
        display.println(F(" Pa"));
        display.display();

				if(data.gas_index == 2) /* Sequential mode sleeps after this measurement */
					delay(250);
			}
		} while (nFieldsLeft);
	}
}
