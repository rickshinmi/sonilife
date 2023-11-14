#include <arduinoFFT.h>
#include "Ultrasonic.h"
#include <M5Stack.h>

Ultrasonic ultrasonic(22);

// マイクのピン、これはちょっと聞いてみてね♪
#define MIC 35

// FFTのパラメータ設定、キラキラしてる感じ♪
#define fftsamples 2048
#define SAMPLING_FREQUENCY 10000

// 非ブロッキングディレイ、待ち時間があるときはちょっとお休み♪
unsigned long previousMillis = 0;

// お手軽非ブロッキングディレイ、これでミリ秒待つよ♪
void nonBlockingDelay(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  }
}

// サーボ制御ちゃん、かわいいね♪
class ServoControl {
public:
  int servoPin;
  int pwmChannel;
  double pwmHz;
  uint8_t pwmLevel;
  int rotationSpeed;
  bool rotationDirection;

public:
  // コンストラクタ、サーボのかんなで〜す♪
  ServoControl(int pin, int channel, double hz, uint8_t level, int speed)
    : servoPin(pin), pwmChannel(channel), pwmHz(hz), pwmLevel(level), rotationSpeed(speed), rotationDirection(true) {}

  // セットアップするよ♪
  void setup() {
    pinMode(servoPin, OUTPUT);
    ledcSetup(pwmChannel, pwmHz, pwmLevel);
    ledcAttachPin(servoPin, pwmChannel);
  }

  // サーボをゆらゆら5秒間回すの♪
  void rotate() {
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 3000;

    while (millis() < endTime) {
      for (int i = 1500; i <= 1600; i += rotationSpeed) {
        ledcWrite(pwmChannel, i);
        nonBlockingDelay(50);
      }
    }

    stopRotation();
  }

  // サーボを停止して、向きをトグルするよ♪
  void stopRotation() {
    ledcWrite(pwmChannel, 0);
    nonBlockingDelay(1000);
    rotationDirection = !rotationDirection;
  }
};

// FFTとサーボ制御のためのアレコレ♪
double vReal[fftsamples];
double vImag[fftsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, fftsamples, SAMPLING_FREQUENCY);
ServoControl servo(16, 1, 50, 16, 1); // ピン16、PWMチャンネル1、PWM周波数50Hz、PWMレベル16、回転速度100
bool rotateEnabled = false;

// 周波数検出のための閾値、これが大事なの♪
double threshold = 0.01;

// セットアップするよ♪
void setup() {
  Serial.begin(115200);
  pinMode(MIC, INPUT);
  servo.setup();
  M5.begin();
  bts.begin("MSR IoT Device");
}

// マイクのサンプルをゲットするよ♪
void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    unsigned int sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    unsigned long t = micros();
    vReal[i] = analogRead(MIC);
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us);
  }
}

// DCオフセットを取り除くんだよ♪
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

// 一番強い周波数を見つけるよ♪
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

  // 周波数の計算を調整、わかるよね♪
  return (maxIndex * SAMPLING_FREQUENCY) / fftsamples;
}



void rotateServoSlowly(int start, int end, int step, int delayTime) {
  unsigned long startTime = millis();
  unsigned long endTime = startTime + 3000;
  for (int i = start; i <= end; i += step) {
    ledcWrite(servo.pwmChannel, i);
    nonBlockingDelay(delayTime);
  servo.stopRotation();
  }
}

// ループループ♪
void loop() {
  // マイクサンプルをゲットするよ♪
  sample(fftsamples);

  // DCオフセットを取り除くんだよ♪
  DCRemoval2(vReal, fftsamples);

  // ハミング窓を適用してFFTするよ♪
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // 一番強い周波数成分を見つけるよ♪
  int strongestFrequency = findStrongestFrequency(fftsamples / 2, threshold);
  //Serial.println(strongestFrequency);

  int RangeInCentimeters = ultrasonic.MeasureInCentimeters();
   //Serial.print(RangeInCentimeters);//0~400cm
   bts.println(RangeInCentimeters, strongestFrequency);

  // 検出された周波数が指定範囲にあって、回転がオフなら、回転をスタートするよ♪
  if (strongestFrequency >= 400 && strongestFrequency <= 600 && !rotateEnabled) {
    rotateEnabled = true;
    unsigned long rotationStartTime = millis();

    // サーボをゆっくりと回すの♪
    rotateServoSlowly(1000, 2000, servo.rotationSpeed, 100);
    // サーボの回転をストップして、回転フラグをリセットするよ♪

    rotateEnabled = false;
  }

  if (RangeInCentimeters <= 100){
    rotateServoSlowly(1000, 2000, servo.rotationSpeed, 10);

  }
}

