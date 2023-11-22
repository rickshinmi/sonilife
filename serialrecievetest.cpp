#include <M5Stack.h>

void setup() {
  M5.begin();
  Serial.begin(115200);
}

void loop() {
  // シリアルデータを受信する
  if (Serial.available() > 0) {
    // 受信したデータを読み込む
    char receivedChar = Serial.read();

    // アスキーコードから文字に変換
    char decodedChar = char(receivedChar);

    // 画面に受信したデータを表示する
    M5.Lcd.printf("Received: %c (ASCII: %d)\n", decodedChar, receivedChar);

  }

  // 他の処理を追加する場合はここに記述する
}
