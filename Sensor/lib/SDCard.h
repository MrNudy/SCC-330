#ifndef SDCARD_H
#define SDCARD_H

#include <array>

#include <SPI.h>
#include <SD.h>
#include "SDCard.h"

#define DATA_SIZE 289

class SDCard {
    private:

        #define chipSelect 4

        String data[DATA_SIZE];
        String buffer;

    public:

        void setup() {

            Serial.print("Initialiszing SD Card");

            if (!SD.begin()) {
                Serial.println("SD failed");
                return;
            }
            Serial.print("SD Initialized");
        }

        void writeData(String data) {

            File dataFile = SD.open("climate_text.txt", FILE_WRITE);

            if (dataFile) {
                dataFile.print(data + "\n");
                dataFile.flush();
                dataFile.close();
            }
        }

        void removeData () {
            SD.remove("climate_text.txt");
        }

        String* getData() {
            File dataFile = SD.open(filename);

            for (int i = 0; i < DATA_SIZE; i++) {
                data[i] = "";
            }

            if (dataFile) {
                for (int i = 0; dataFile.available() && i < DATA_SIZE; i++){
                    buffer = dataFile.readStringUntil('\n');
                    data[i] = buffer;
                }
                dataFile.close();
            } else {
                Serial.println("Error reading file");
            }

            return data;
        }
}

#endif

