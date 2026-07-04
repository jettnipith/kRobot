#include "lotus.h"

int sensor_l;
int sensor_r;
int knob;
int splitter_l = (1100 + 50) / 2;
int splitter_r = (1100 + 50) / 2;

void setup() {
  pinMode(_DL1, OUTPUT);
  pinMode(_DL2, OUTPUT);
  pinMode(_PWML, OUTPUT);
  pinMode(_DR1, OUTPUT);
  pinMode(_DR2, OUTPUT);
  pinMode(_PWMR, OUTPUT);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
  show4lines("Hello", "", "Jettnipith", "", "", "", "", "");
  delay(2000);
  wait_SW1();
}

void loop() {
  sensor_l = analogRead(26);
  sensor_r = analogRead(25);
  knob = analogRead(35);
  int speed = map(knob,0,1023,0,255);
  bool is_l_black = sensor_l > splitter_l ? 1 : 0;
  bool is_r_black = sensor_r > splitter_r ? 1 : 0;
  String action = "";
  if (is_l_black && is_r_black) {
    action = "STOP";
    run(0,0);
  } else if (is_l_black && !is_r_black) {
    action = "SPIN L";
    run(-125,125);
    delay(100);
  } else if (!is_l_black && is_r_black) {
    action = "SPIN R";
    run(125,-125);
    delay(100);
  } else if (!is_l_black && !is_r_black) {
    action = "GO";
    run(speed,speed);
  }
  show4lines("L", String(sensor_l), "R", String(sensor_r), "", "", action, String(speed));
}
