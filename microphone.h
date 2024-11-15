#ifndef MICROPHONE_H
#define MICROPHONE_H

#include <Arduino.h>
#include <PDM.h>
#include <Wire.h>

// Buffer to read samples into, each sample is 16-bits
extern short sampleBuffer[512];

// Number of audio samples read
extern volatile int samplesRead;

// Function to initialize microphone
void initMicrophone() {
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

// Callback function to process the data from the PDM microphone
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

// Function to calculate the decibel level from the microphone samples
float calculateDecibels() {
  float sum = 0;
  for (int i = 0; i < samplesRead; i++) {
    sum += abs(sampleBuffer[i]);
  }
  float average = sum / samplesRead;
  return 20.0f * log10(average / 32767.0f); // Convert average value to decibels
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

#endif
