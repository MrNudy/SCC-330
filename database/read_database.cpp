#include <SD.h>
#include <SPI.h>

const int chipSelect = 10;

void setup() {
    Serial.begin(9600);
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");
}

void readEnvironmentalData() {
    File dataFile = SD.open("environmental_data.csv", FILE_READ);
    if (dataFile) {
        Serial.println("Reading Environmental Data:");
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');  // Read a line until newline
            int commaIndex1 = line.indexOf(',');           // Find the first comma (timestamp end)
            int commaIndex2 = line.indexOf(',', commaIndex1 + 1); // Light-Sound separator
            int commaIndex3 = line.indexOf(',', commaIndex2 + 1); // Sound-Temp separator

            String timestamp = line.substring(0, commaIndex1);
            String light = line.substring(commaIndex1 + 3, commaIndex2); // Skip "E:"
            String sound = line.substring(commaIndex2 + 1, commaIndex3);
            String temp = line.substring(commaIndex3 + 1);

            // Sample method to produce the data from CSV files, can later be changed to return struct for use in other code and functions
            Serial.print("Timestamp: ");
            Serial.print(timestamp);
            Serial.print(" | Light: ");
            Serial.print(light);
            Serial.print(" | Sound: ");
            Serial.print(sound);
            Serial.print(" | Temperature: ");
            Serial.println(temp);
        }
        dataFile.close();
    } else {
        Serial.println("Error opening environmental_data.csv");
    }
}

void readObjectData() {
    File dataFile = SD.open("object_data.csv", FILE_READ);
    if (dataFile) {
        Serial.println("Reading Object Data:");
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');  
            int commaIndex = line.indexOf(',');          

            String timestamp = line.substring(0, commaIndex);
            String object = line.substring(commaIndex + 3);

            Serial.print("Timestamp: ");
            Serial.print(timestamp);
            Serial.print(" | Object: ");
            Serial.println(object);
        }
        dataFile.close();
    } else {
        Serial.println("Error opening object_data.csv");
    }
}

void readTripwireData() {
    File dataFile = SD.open("tripwire_data.csv", FILE_READ);
    if (dataFile) {
        Serial.println("Reading Tripwire Data:");
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');  
            int commaIndex = line.indexOf(',');           

            String timestamp = line.substring(0, commaIndex);
            String zone = line.substring(commaIndex + 3);

            Serial.print("Timestamp: ");
            Serial.print(timestamp);
            Serial.print(" | Zone: ");
            Serial.println(zone);
        }
        dataFile.close();
    } else {
        Serial.println("Error opening tripwire_data.csv");
    }
}

void readCupData() {
    File dataFile = SD.open("cup_data.csv", FILE_READ);
    if (dataFile) {
        Serial.println("Reading Cup Data:");
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');
            int commaIndex = line.indexOf(',');           

            String timestamp = line.substring(0, commaIndex);
            String mlUsed = line.substring(commaIndex + 3); 

            Serial.print("Timestamp: ");
            Serial.print(timestamp);
            Serial.print(" | ML Used: ");
            Serial.println(mlUsed);
        }
        dataFile.close();
    } else {
        Serial.println("Error opening cup_data.csv");
    }
}

void readActuatorData() {
    File dataFile = SD.open("actuator_data.csv", FILE_READ);
    if (dataFile) {
        Serial.println("Reading Actuator Data:");
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n'); 
            int commaIndex = line.indexOf(',');            

            String timestamp = line.substring(0, commaIndex);
            String level = line.substring(commaIndex + 3); 

            Serial.print("Timestamp: ");
            Serial.print(timestamp);
            Serial.print(" | Level: ");
            Serial.println(level);
        }
        dataFile.close();
    } else {
        Serial.println("Error opening actuator_data.csv");
    }
}

