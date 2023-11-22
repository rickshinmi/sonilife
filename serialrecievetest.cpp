#include <M5Stack.h>

void setup() {
  M5.begin();
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    // 改行文字 (0x0a) までのテキストを受信
    String text = Serial.readStringUntil(0x0a);

    if (text.length() > 0 && text.startsWith("*")) {
      text.trim();
      text.replace("*", "");

      // 画面をクリア
      M5.Lcd.clear();

      // 大きなフォントサイズでテキストを表示
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(10, 50);
      M5.Lcd.print(text);
    }
  }

  // 他の処理を追加する場合はここに記述する
}
