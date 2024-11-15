////PATH = ~platformio/packages/framework-arduinopico/libraries/PDM/src/rp2040

#ifndef MICROPHONE_H
#define MICROPHONE_H

#include <Arduino.h>
#include <PDM.h>

#define SAMPLE_BUFFER_SIZE 256
#define BIT_PER_SAMPLE 16
#define FREQUENCY 16000

extern short sampleBuffer[SAMPLE_BUFFER_SIZE];
extern int samplesRead;

void initializeMicrophone();
void onPDMdata();
float getDecibels();


#endif