#include <M5Stack.h>
#include <Servo.h>

int servoAng = 0; // サーボの角度を格納する変数
Servo myServo;     // Servo オブジェクトを作成

void setup() {
  M5.begin();
  Serial.begin(115200);
  Serial2.begin(115200); // M5StackのSerial2ポートを使用

  myServo.attach(21); // サーボの信号線をM5Stackのピン21に接続

  // その他の初期化処理を追加する場合はここに記述
}

void loop() {
  // シリアル通信からデータを受信する
  if (Serial2.available() > 0) {
    // シリアル通信から1バイト読み込む
    char receivedChar = Serial2.read();

    if (isdigit(receivedChar)) {
      servoAng = Serial2.parseInt();
      myServo.write(servoAng);
      M5.Lcd.fillScreen(TFT_BLACK); 
      M5.Lcd.setCursor(10, 10);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(TFT_WHITE);
      M5.Lcd.print("Received servo angle: ");
      M5.Lcd.print(servoAng);
    }
  }

}
