// ねえねえ、今からクールなコードの世界へ案内するよ！
#include <M5Stack.h> // M5Stackの魅力を呼び起こすぅ♪

#include <arduinoFFT.h> // FFTって言葉、なんだかエキサイティングだね♪

// ここからが私たちのオリジナルなストーリーだよ！
#define MIC_PIN 36 // マイクはここに接続してるよ♪
#define FFT_SAMPLES 2048 // 2048の魔法で未知の世界へダイブ！

#define SAMPLING_FREQUENCY 10000 // サンプリングする頻度、音楽みたい♪

#define MIN_FREQUENCY 400 // 最低の周波数は400Hzよ♪
#define MAX_FREQUENCY 1000 // 最高は1000Hz！もうワクワクするでしょ？

// さぁ、冒険の始まりだよ！
double vReal[FFT_SAMPLES]; // 現実の波形、夢幻の音楽が広がる♪

double vImag[FFT_SAMPLES]; // 想像の波形、私たちの感性を広げるんだ♪

arduinoFFT FFT = arduinoFFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY); // FFTさん、よろしくお願いね♪

double threshold = 1; // ドキドキするね、閾値を調整してね♪

// この先は、魔法の扉が開く瞬間！
double peakMagnitude = 0; // ピークなマジック、ここで見つけるんだよ♪

double avgMagnitude = 0; // 平均の魔力、すごく大切なんだよ♪

int peakIndex = 0; // ピークの場所、ここがクライマックスだよね♪

// この範囲で冒険するよ！
int minIndex = map(MIN_FREQUENCY, 0, SAMPLING_FREQUENCY / 2, 0, FFT_SAMPLES / 2);
int maxIndex = map(MAX_FREQUENCY, 0, SAMPLING_FREQUENCY / 2, 0, FFT_SAMPLES / 2);

// さあ、心の準備はいいかな？
for (int i = minIndex; i <= maxIndex; i++) {
  double magnitude = vReal[i] / FFT_SAMPLES; // マジカルな振幅、夢を感じるんだ♪

  avgMagnitude += magnitude; // 平均の世界、一歩一歩進んでいこう♪

  if (magnitude > peakMagnitude && magnitude > threshold) {
    peakMagnitude = magnitude; // ピークな瞬間、興奮が止まらないね♪

    peakIndex = i; // ここがピークの頂点、これぞ最高潮だよね♪
  }
}

avgMagnitude /= (maxIndex - minIndex + 1); // 平均のマジック、計算して感動しよう♪

int peakFrequency = (map(peakIndex, 0, FFT_SAMPLES / 2, 0, SAMPLING_FREQUENCY)) / 2; // ピークの周波数、これで音楽の世界へ♪

Serial.print(peakMagnitude); // ドキドキの瞬間、ピークの魔力を見せるよ♪
Serial.print(" "); // そう、感動は言葉にならないよね♪

Serial.println(avgMagnitude); // 平均も大事だよね、一緒に味わおう♪

delay(100); // ちょっとの休息、冒険の続きを楽しみにしてね♪

// これで一つの冒険が終わったよ、次はどんな世界が広がっているのかな？
