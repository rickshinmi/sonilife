/*
 * 2022/05/08(Sun)
 * Developer: Kabosuxu
 * Opensource
 */

#include <arduinoFFT.h>
#define MIC 35

#define fftsamples 2048
#define SAMPLING_FREQUENCY 10000
double vReal[fftsamples];
double vImag[fftsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, fftsamples, SAMPLING_FREQUENCY);

double threshold = 0.01;

void setup() {
  Serial.begin(115200);
  pinMode(MIC, INPUT);
}

void loop() {
  sample(fftsamples);
  DCRemoval2(vReal, fftsamples);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  int strongestFrequency = findStrongestFrequency(fftsamples / 2, threshold);
  Serial.println(strongestFrequency);
  delay(100); // Adjust the delay as needed
}

void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    unsigned int sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    unsigned long t = micros();
    vReal[i] = analogRead(MIC);
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

int findStrongestFrequency(int nsamples, double threshold) {
  double maxMagnitude = 0;
  int maxIndex = 0;

  for (int i = 1; i < nsamples / 2; i++) {
    double magnitude = vReal[i] / nsamples;
    if (magnitude > maxMagnitude && magnitude > threshold) {
      maxMagnitude = magnitude;
      maxIndex = i;
    }
  }

  // Adjust the calculation for the frequency
  return (maxIndex * SAMPLING_FREQUENCY) / fftsamples;
}



