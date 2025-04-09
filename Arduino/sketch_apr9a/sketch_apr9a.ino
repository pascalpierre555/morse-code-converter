#include <Wire.h>
#include <LiquidCrystal.h> 
LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);

void receiveEvent(int howMany) {
  static int cursor;
  while (Wire.available()) {
    char c = Wire.read();  // 讀取字元
    Serial.print("Received: ");
    Serial.println(c);
    lcd.setCursor(cursor % 16, cursor / 16); // 設定游標位置在第二行行首
    cursor++;
    lcd.print(c); // 列印 "Hello Mirotek !!" 訊息到 LCD 上
  }
}

void setup() {
  
  lcd.begin(16, 2); // 初始化 LCD，一行 16 的字元，共 2 行，預設開啟背光
  Serial.begin(9600);
  lcd.setCursor(0, 0); // 設定游標位置在第二行行首
  Wire.begin(0x08);  // 設定為 slave，地址 0x08
  Wire.onReceive(receiveEvent);
}

void loop() {
  // 這裡可空
}


// 建立 LiquidCrystal 的變數 lcd
// LCD 接腳: RS, R/W, Enable, D4, D5, D6, D7 
// Arduino 接腳: 12, 11, 10, 5, 4, 3, 2

