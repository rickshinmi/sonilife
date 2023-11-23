#include <M5Stack.h>
#include <arduinoFFT.h>
#include <ESP32Servo.h>

Servo servo1;
#define FFT_SAMPLES 2048
#define SAMPLING_FREQUENCY 10000
#define MIN_FREQUENCY 400
#define MAX_FREQUENCY 1000
#define servoPin 2
#define DC_MOTERPIN 5
#define MIC_PIN 36


double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);

double threshold = 1;  // Adjust the threshold as needed
bool rotateEnabled = false;

unsigned long previousMillis = 0;

void nonBlockingDelay(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  }
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

void DCmoter(int speed){
  analogWrite(DC_MOTERPIN, speed);
}



void setup() {
  M5.begin();
  Serial.begin(115200);
  pinMode(MIC_PIN, INPUT);
  servo1.attach(servoPin, 500, 2400); // サーボのピンと制限パルス幅を指定
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
  int peakFrequency = (map(peakIndex, 0, FFT_SAMPLES / 2, 0, SAMPLING_FREQUENCY))/2;

  // Send the results over serial
  //Serial.print("Peak Frequency: ");
  //Serial.println(peakFrequency);
  //Serial.print(" Hz, Peak Magnitude: ");
  Serial.print(peakMagnitude);
  Serial.print(" ");
  //Serial.print(", Average Magnitude: ");
  Serial.println(avgMagnitude);
  nonBlockingDelay(100);  // Adjust the delay as needed
  int previousValue = 0; // 前回の値を保持する変数

  if (Serial.available() > 0) 
  {
    while (Serial.available() > 0) 
    {
      int receivedChar = Serial.read();
      M5.Lcd.println(receivedChar);
      servo1.write(receivedChar);
      // 一個前の値と最新の値の差を計算して変数に格納する。
      int diff = receivedChar - previousValue;
      // 今回の値を前回の値として保存
      previousValue = receivedChar;
      // 0から10の範囲の値を100から255の範囲にマッピング
      int mappedDiff = map(diff, 0, 10, 100, 255);
      // DCモーターにマッピングされた値を適用
      DCmoter(mappedDiff);
    }
  }

  if (peakFrequency >= MIN_FREQUENCY && peakFrequency <= MAX_FREQUENCY && !rotateEnabled) 
  {
    rotateEnabled = true;
    DCmoter(75);
    nonBlockingDelay(1000);
    rotateEnabled = false;
  }
}
