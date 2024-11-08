#include <SD.h>
#include <SPI.h>

const int chipSelect = 10;  // SD card pin

void setup() {
    Serial.begin(9600);
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");
}

void writeEnvironmentalData(float light, float sound, float temp) {
    File dataFile = SD.open("environmental_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());         // Timestamp
        dataFile.print(",E:");
        dataFile.print(light);            
        dataFile.print(",");
        dataFile.print(sound);          
        dataFile.print(",");
        dataFile.println(temp);          
        dataFile.close();
        Serial.println("Environmental data written.");
    } else {
        Serial.println("Error opening environmental_data.csv");
    }
}

void writeObjectData(String object) {
    File dataFile = SD.open("object_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());      
        dataFile.print(",O:'");
        dataFile.print(object);       
        dataFile.println("'");
        dataFile.close();
        Serial.println("Object data written.");
    } else {
        Serial.println("Error opening object_data.csv");
    }
}

void writeTripwireData(String zone) {
    File dataFile = SD.open("tripwire_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());       
        dataFile.print(",T:");
        dataFile.println(zone);           
        dataFile.close();
        Serial.println("Tripwire data written.");
    } else {
        Serial.println("Error opening tripwire_data.csv");
    }
}

void writeCupData(float mlUsed) {
    File dataFile = SD.open("cup_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());      
        dataFile.print(",C:");
        dataFile.println(mlUsed);   
        dataFile.close();
        Serial.println("Cup data written.");
    } else {
        Serial.println("Error opening cup_data.csv");
    }
}

void writeActuatorData(int level) {
    File dataFile = SD.open("actuator_data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());    
        dataFile.print(",A:");
        dataFile.println(level);     
        dataFile.close();
        Serial.println("Actuator data written.");
    } else {
        Serial.println("Error opening actuator_data.csv");
    }
}
