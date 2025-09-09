#include"lotus.h"

void setup() {
  Serial.begin(115200);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
  //pinMode(BUTTON_PIN, INPUT_PULLUP);       // Calibration button
  pinMode(27, INPUT);  // กำหนดขา 27 เป็น input

  gServoArm.attach(32, 500, 2400);
  gServoCraw.attach(33, 500, 2400);
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
  fixFrontThreshold(2000,2000,2000,403,2000);
  _kp = 0.0125;

  //---------------------------------------เริ่ม Code ตรงนี้---------------------------------------
  obj_prepare();//กางมือเพื่อเตรียมจับ
  fw(20000,148,"f", "l");//เดินหน้าตามเส้น 20 วินาที ด้วยความเร็ว 178 ใช้เซนเซอน์หน้าเพื่อนับแยก เมื่อเจอแยกให้หยุดแม้เวลาไม่หมดก็ตาม
  fw(20000,72,"f", "s");//เดินช้าเข้าไปจับ
  obj_catch(); //จับวัตถุ
  //fw(20000,148,"f", "l");
  //fw(20000,148,"f", "l");

  //spin_l_to_line(128);//เลี้ยวซ้ายจนกว่าจะเข้าเส้นอีกครั้ง
  //fw(20000,128,"f", "s");//เดินหน้าตามเส้น 20 วินาที ด้วยความเร็ว 128 ใช้เซนเซอน์หน้าเพื่อนับแยก เมื่อเจอแยกให้หยุดแม้เวลาไม่หมดก็ตาม
  //obj_release(); //ปล่อยวัตถุ */

  //---------------------------------------จบการทำงาน---------------------------------------
  run(0,0);

}

void loop() {
  getSensorValues();
  show4lines(String(_frontValues[0]),String(_frontValues[1]),String(_frontValues[2]),String(_frontValues[3]),String(_frontValues[4]),"","","");

}
