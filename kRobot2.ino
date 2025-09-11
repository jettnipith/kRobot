#include "lotus-line-sonic.h"


void setup() {
  Serial.begin(115200);

  pinMode(_MUX_S0, OUTPUT);
  pinMode(_MUX_S1, OUTPUT);
  pinMode(_MUX_S2, OUTPUT);
  pinMode(_MUX_SIG, INPUT);

  analogSetWidth(12);
  analogSetPinAttenuation(_MUX_SIG, ADC_11db);

  Serial.println("ESP32 + CD4051BM ready");

  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
  //pinMode(BUTTON_PIN, INPUT_PULLUP);       // Calibration button
  pinMode(27, INPUT);  // กำหนดขา 27 เป็น input

  gServoArm.attach(33, 500, 2400);
  gServoCraw.attach(5, 500, 2400);
  pinMode(_DL1, OUTPUT);
  pinMode(_DL2, OUTPUT);
  pinMode(_PWML, OUTPUT);
  pinMode(_DR1, OUTPUT);
  pinMode(_DR2, OUTPUT);
  pinMode(_PWMR, OUTPUT);
  tone(18, 660, 100);
  show4lines("Calibrating front..", "", "place front on black", "", "", "", "", "");
  wait_SW1();
  calibrateFrontSensors();
  show4lines("Calibrating front..", "", "place front on white", "", "", "", "", "");
  wait_SW1();
  calibrateFrontSensors();
  show4lines("Front Treshold", "", String(_frontThreshold[0]) + " " + String(_frontThreshold[1]), String(_frontThreshold[2]) + " " + String(_frontThreshold[3]), String(_frontThreshold[4]) + " " + String(_frontThreshold[5]), String(_frontThreshold[6]) + " " + String(_frontThreshold[7]), "", "");
  wait_SW1_done();
  delay(1000);
  fixFrontThreshold(1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900);
  _kp = 0.0125;
  const int speed_fast = 128;  //ตั้งค่าความเร็วที่ใช้วิ่งเร็ว
  const int speed_slow = 62;   //ตั้งค่าความเร็วที่ใช้วิ่งช้า

  //---------------------------------------เริ่ม Code ตรงนี้---------------------------------------

  obj_prepare();
  fw(20000, speed_slow, "f", "l");
  fw(20000, speed_slow, "f", "s");
  obj_catch();
  spin_l_to_line(109);
  fw(20000, speed_slow, "f", "r");
  fw(20000, speed_slow, "f", "l");



  //---------------------------------------จบการทำงาน---------------------------------------
  run(0, 0);
}

void loop() {

  getSensorValues();
  show4lines(String(_frontValues[0]), String(_frontValues[1]), String(_frontValues[2]), String(_frontValues[3]), String(_frontValues[4]), String(_frontValues[5]), String(_frontValues[6]), String(_frontValues[7]));
}
