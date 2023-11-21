#include <M5Stack.h>
#include <arduinoFFT.h>

#define MIC_PIN 36 // Assuming the microphone is connected to pin 0
#define FFT_SAMPLES 2048
#define SAMPLING_FREQUENCY 10000
#define MIN_FREQUENCY 400
#define MAX_FREQUENCY 1000

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);

double threshold = 1;  // Adjust the threshold as needed

void setup() {
  M5.begin();
  Serial.begin(115200);
  pinMode(MIC_PIN, INPUT);
}

void loop() {
  sample(FFT_SAMPLES);
  DCRemoval2(vReal, FFT_SAMPLES);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Find the peak and average magnitude in the specified frequency range
  double peakMagnitude = 0;
  double avgMagnitude = 0;
  int peakIndex = 0;

  int minIndex = map(MIN_FREQUENCY, 0, SAMPLING_FREQUENCY / 2, 0, FFT_SAMPLES / 2);
  int maxIndex = map(MAX_FREQUENCY, 0, SAMPLING_FREQUENCY / 2, 0, FFT_SAMPLES / 2);

  for (int i = minIndex; i <= maxIndex; i++) {
    double magnitude = vReal[i] / FFT_SAMPLES;
    avgMagnitude += magnitude;

    if (magnitude > peakMagnitude && magnitude > threshold) {
      peakMagnitude = magnitude;
      peakIndex = i;
    }
  }

  avgMagnitude /= (maxIndex - minIndex + 1);

  // Adjust the calculation for the frequency
  int peakFrequency = (map(peakIndex, 0, FFT_SAMPLES / 2, 0, SAMPLING_FREQUENCY))/2;

  // Send the results over serial
  Serial.print("Peak Frequency: ");
  Serial.print(peakFrequency);
  Serial.print(" Hz, Peak Magnitude: ");
  Serial.print(peakMagnitude);
  Serial.print(", Average Magnitude: ");
  Serial.println(avgMagnitude);

  delay(100);  // Adjust the delay as needed
}

void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    unsigned int sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    unsigned long t = micros();
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us);
  }
}

void DCRemoval2(double *vData, uint16_t samples) {
  double mean = 0;
  for (uint16_t i = 1; i < samples; i++) {
    mean += vData[i];
  }
  mean /= samples;
  for (uint16_t i = 1; i < samples; i++) {
    vData[i] -= mean;
  }
}
