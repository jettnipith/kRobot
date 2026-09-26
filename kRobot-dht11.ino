#include "lotus.h"

void setup() {
  Serial.begin(115200);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
  //pinMode(BUTTON_PIN, INPUT_PULLUP);       // Calibration button
  pinMode(27, INPUT);  // กำหนดขา 27 เป็น input

  gServoArm.attach(32, 500, 2400);
  gServoCraw.attach(33, 500, 2400);
  dht.setup(_DHTPIN, DHTesp::DHT11);
  pinMode(_DL1, OUTPUT);
  pinMode(_DL2, OUTPUT);
  pinMode(_PWML, OUTPUT);
  pinMode(_DR1, OUTPUT);
  pinMode(_DR2, OUTPUT);
  pinMode(_PWMR, OUTPUT);

  tone(18, 660, 100);
  show4lines("Calibrating front..", "", "place front on black", "", "", "", "", "");
  wait_SW1();
}

void loop() {
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    show4lines("Failed to", "", "read from DHT sensor!", "", "", "", "", "");
  } else {
    show4lines("Temperature: ", String(temperature)+ " C", "Humidity: ", String(temperature) + " %", "", "", "", "");
  }

  delay(2000);  // อ่านค่าทุก 2 วินาที
}
