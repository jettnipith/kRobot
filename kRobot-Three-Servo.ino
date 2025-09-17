#include"lotus-three-servo.h"

void setup() {
  Serial.begin(115200);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
  //pinMode(BUTTON_PIN, INPUT_PULLUP);       // Calibration button
  pinMode(27, INPUT);  // กำหนดขา 27 เป็น input

  gServoArm.attach(32, 500, 2400);
  gServoL.attach(33, 500, 2400);
  gServoR.attach(5, 500, 2400);
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
  show4lines("Front Treshold", "", String(_frontThreshold[0]), String(_frontThreshold[1]), String(_frontThreshold[2]), String(_frontThreshold[3]),  String(_frontThreshold[4]),"");
  wait_SW1_done();
  delay(1000);
  //อย่าลืมวัดแสงแล้วตั้งค่าแบ่งสีตรงนี้ด้วย
  fixFrontThreshold(1210,2097,1701,1225,2067);
  _kp = 0.0125;
  _speed_fast = 255;
  _speed_slow = 125;
  //---------------------------------------เริ่ม Code ตรงนี้---------------------------------------
  obj_prepare();
  //fw(20000,_speed_fast,"f", "s");
  
  //obj_catch();
  //spin_l_to_line(128);
  //fw(20000,128,"f", "s");
  //fw(20000,128,"f", "s");
  //fw(20000,128,"f", "s");
  //fw(20000,128,"f", "s");
  //obj_release();

  //---------------------------------------จบการทำงาน---------------------------------------

}

void loop() {
  getSensorValues();
  show4lines(String(_frontValues[0]),String(_frontValues[1]),String(_frontValues[2]),String(_frontValues[3]),String(_frontValues[4]),"","","");

}
