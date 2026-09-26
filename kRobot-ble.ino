#include "lotus.h"

void setup() {
  
  init_robot();
  tone(18, 660, 100);
  show4lines("Calibrating front..", "", "place front on black", "", "", "", "", "");
  wait_SW1();
  init_ble("ESP32-K");
}

void loop() {

  ble_notify("Hello Jettnipith");
}
