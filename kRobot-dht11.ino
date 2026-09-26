#include "lotus.h"

void setup() {
  
  init_robot();
  init_dht11(23);
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
