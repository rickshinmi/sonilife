#include <M5Stack.h>

void setup() {
  M5.begin();
  Serial.begin(115200);
}

void loop() {
  // シリアルデータを受信する
  if (Serial.available() > 0) {
    // 最新のデータを受信する
    while (Serial.available() > 0) {
      char receivedChar = Serial.read();
      
      // アスキーコードから文字に変換
      char decodedChar = char(receivedChar);

      // 画面に受信したデータを表示する
      M5.Lcd.clear(); // 画面をクリアして最新の値のみ表示
      M5.Lcd.printf("Received: %c (ASCII: %d)\n", decodedChar, receivedChar);
      
      delay(100); // 状態が急激に変わる場合に備えて適宜調整
    }
  }

  // 他の処理を追加する場合はここに記述する
}
