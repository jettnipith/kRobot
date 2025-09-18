#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Arduino.h>

Adafruit_SSD1306 OLED(-1);
Servo gServoArm;
Servo gServoL;
Servo gServoR;

// กำหนดชื่อGPIO มอเตอร์ฝั่งซ้าย
#define _DR1 2
#define _DR2 15
#define _PWMR 13
// กำหนดชื่อGPIO มอเตอร์ฝั่งขวา
#define _DL1 16
#define _DL2 17
#define _PWML 4

#define BUTTON_PIN 27


int _NumofSensor = 5;
int _Sensitive = 20;
int _lastPosition = 0;

int _frontMin[5] = { 100, 100, 100, 100, 100 };
int _frontMax[5] = { 100, 100, 100, 100, 100 };
int _frontValues[5];
int _frontThreshold[5];
int _frontPins[5] = { 26, 25, 14, 39, 36 };

int _baseSpeed = 110;  // Set your base speed here (range 0-255, adjust as needed)
int _speed_fast = 166;
int _speed_slow = 89;
int _biasL = 0;
int _biasR = 0;

float _kp = 0.1, _ki = 0.0, _kd = 0;  // kp(0.1-15.0) kd(0.05)
float _previousError = 0;
float _integral = 0;




void run(int spl, int spr)  // ประกาศฟังก์ชัน run(กำลังมอเตอร์ซ้าาย,กำลังมอเตอร์ขวา);
{
  if (spl > 0)  // เมื่อค่า PWM มอเตอร์ซ้ายมากกว่า 0 จะทำให้มอเตอร์ซ้ายเดินหน้าตามค่าสัมบูรณ์PWM(spl)
  {
    digitalWrite(_DL1, LOW);
    digitalWrite(_DL2, HIGH);
    analogWrite(_PWML, spl);
  } else if (spl < 0)  // เมื่อค่า PWM มอเตอร์ซ้ายน้อยกว่า 0 จะทำให้มอเตอร์ซ้ายเดินถอยหลังตามค่าสัมบูรณ์PWM(spl)

  {
    digitalWrite(_DL1, HIGH);
    digitalWrite(_DL2, LOW);
    analogWrite(_PWML, -spl);
  } else  // นอกเหนือจากนั้นจะให้มอเตอร์หยุดแบบเบรก

  {
    digitalWrite(_DL1, HIGH);
    digitalWrite(_DL2, HIGH);
  }
  //////////////////////////////////////
  if (spr > 0)  // เมื่อค่า PWM มอเตอร์ขวามากกว่า 0 จะทำให้มอเตอร์ขวาเดินหน้าตามค่าสัมบูรณ์PWM(spl)

  {
    digitalWrite(_DR1, LOW);
    digitalWrite(_DR2, HIGH);
    analogWrite(_PWMR, spr);
  } else if (spr < 0) {
    digitalWrite(_DR1, HIGH);
    digitalWrite(_DR2, LOW);
    analogWrite(_PWMR, -spr);
  } else  // นอกเหนือจากนั้นจะให้มอเตอร์หยุดแบบเบรก
  {
    digitalWrite(_DR1, HIGH);
    digitalWrite(_DR2, HIGH);
  }
}

void show4lines(String title1, String val1, String title2, String val2, String title3, String val3, String title4, String val4) {
  String firstLine = title1 + "\t" + val1;
  String secondLine = title2 + "\t" + val2;
  String thirdLine = title3 + "\t" + val3;
  String fouthLine = title4 + "\t" + val4;
  OLED.clearDisplay();              // คำสั่งเคลียร์หน้าจอ
  OLED.setTextColor(WHITE, BLACK);  // ตั้งค่าตัวอักษรสีขาว พืนหลังดำ
  OLED.setTextSize(1);              // กำหนดขนาดอักษร
  OLED.setCursor(0, 0);             // กำหนดตำแหน่งการวางอักษรตัวแรกแกนx,y

  OLED.println(firstLine);   // พิมพ์ตัวอักษรคำว่า Lotus หากมีตัวอักษรชุดใหม่ จะพิมพ์ที่บรรทัดถัดไปต่อ
  OLED.println(secondLine);  // พิมพ์ตัวอักษรคำว่า Lotus หากมีตัวอักษรชุดใหม่ จะพิมพ์ที่บรรทัดถัดไปต่อ
  OLED.println(thirdLine);   // พิมพ์ตัวอักษรคำว่า Lotus หากมีตัวอักษรชุดใหม่ จะพิมพ์ที่บรรทัดถัดไปต่อ
  OLED.println(fouthLine);   // พิมพ์ตัวอักษรคำว่า Lotus หากมีตัวอักษรชุดใหม่ จะพิมพ์ที่บรรทัดถัดไปต่อ
  OLED.display();            // แสดงข้อความบนOLED
}

void calibrateFrontSensors() {
  //STOP MOTOR
  run(0, 0);
  // Read sensors and calibrate min/max
  for (int i = 0; i < _NumofSensor; i++) {
    _frontValues[i] = analogRead(_frontPins[i]);
    if (_frontValues[i] > _frontMax[i]) _frontMax[i] = _frontValues[i];
    if (_frontValues[i] < _frontMin[i]) _frontMin[i] = _frontValues[i];
  }
  delay(100);  // Calibration speed
  // Calculate thresholds after calibration
  for (int i = 0; i < _NumofSensor; i++) {
    _frontThreshold[i] = (_frontMin[i] + _frontMax[i]) / 2;
  }
  show4lines("Calibrated", "", "", "", "", "", "", "");
}

void getSensorValues() {
  for (int i = 0; i < _NumofSensor; i++) {
    _frontValues[i] = analogRead(_frontPins[i]);
  }
}

void fixFrontThreshold(int ft1, int ft2, int ft3, int ft4, int ft5) {
  _frontThreshold[0] = ft1;
  _frontThreshold[1] = ft2;
  _frontThreshold[2] = ft3;
  _frontThreshold[3] = ft4;
  _frontThreshold[4] = ft5;
}




int getLinePosition() {
  int weightedSum = 0;
  int totalWeight = 0;

  for (int i = 0; i < _NumofSensor; i++) {
    _frontValues[i] = analogRead(_frontPins[i]);
    // Update binary detection for the black line
    int value = (_frontValues[i] > _frontThreshold[i]) ? 0 : 1;  // Line detected as 1 when below threshold

    weightedSum += value * (i * 1000);  // Weighting position
    totalWeight += value;
  }

  if (totalWeight == 0) return 2000;  // Default to center if no line detected
  return weightedSum / totalWeight;   // Return line position
}

void spin_l_to_line(int speed) {
  run(0, 0);
  delay(10);
  run(constrain(_biasL - speed, -255, 255), constrain(_biasR + speed, -255, 255));
  delay(100);
  getSensorValues();
  while (_frontValues[1] > _frontThreshold[1]) {
    getSensorValues();
    run(constrain(_biasL - speed, -255, 255), constrain(_biasR + speed, -255, 255));
    delay(1);
  }
  run(0, 0);
  //run(constrain(_biasL + speed, -255, 255), constrain(_biasR - speed, -255, 255));
  //delay(10);
}

void spin_r_to_line(int speed) {
  run(0, 0);
  delay(10);
  run(constrain(_biasL + speed, -255, 255), constrain(_biasR - speed, -255, 255));
  delay(100);
  getSensorValues();
  while (_frontValues[3] > _frontThreshold[3]) {
    getSensorValues();
    run(constrain(_biasL + speed + 49, -255, 255), constrain(_biasR - speed, -255, 255));
    delay(1);
  }
  run(0, 0);
  //run(constrain(_biasL - speed + 0, -255, 255), constrain(_biasR + speed, -255, 255));
  //delay(10);
}

void spin_l(int timeLimiter) {
  unsigned long startTime = millis();
  int speed = 128;
  while (millis() - startTime < timeLimiter) {
    run(constrain(_biasL - speed, -255, 255), constrain(_biasR + speed, -255, 255));
  }
  run(0, 0);
  delay(10);
  run(constrain(_biasL + speed, -255, 255), constrain(_biasR - speed, -255, 255));
  delay(60);
  run(0, 0);
}

void spin_r(int timeLimiter) {
  unsigned long startTime = millis();
  int speed = 128;
  while (millis() - startTime < timeLimiter) {
    run(constrain(_biasL + speed, -255, 255), constrain(_biasR - speed, -255, 255));
  }
  run(0, 0);
  delay(10);
  run(constrain(_biasL - speed, -255, 255), constrain(_biasR + speed, -255, 255));
  delay(60);
  run(0, 0);
}

void fw_no_line(int timeLimiter) {
  unsigned long startTime = millis();
  int speed = 128;
  while (millis() - startTime < timeLimiter) {
    run(constrain(_biasL + speed, -255, 255), constrain(_biasR + speed, -255, 255));
  }
  run(0, 0);
  delay(10);
  run(constrain(_biasL - speed, -255, 255), constrain(_biasR - speed, -255, 255));
  delay(60);
  run(0, 0);
}

void bw_no_line(int timeLimiter) {
  unsigned long startTime = millis();
  int speed = 128;
  while (millis() - startTime < timeLimiter) {
    run(constrain(_biasL - speed, -255, 255), constrain(_biasR - speed, -255, 255));
  }
  run(0, 0);
  delay(10);
  run(constrain(_biasL + speed, -255, 255), constrain(_biasR + speed, -255, 255));
  delay(60);
  run(0, 0);
}


void PIDforward() {
  int linePosition = getLinePosition();  // Position from the sensor array
  float error = 2000 - linePosition;     // Target position is center (4000) for 8 sensors

  _integral += error;
  float derivative = error - _previousError;

  float correction = _kp * error + _ki * _integral + _kd * derivative;
  _previousError = error;

  int leftMotorSpeed = constrain(_baseSpeed + _biasL - correction, -255, 255);
  int rightMotorSpeed = constrain(_baseSpeed + _biasR + correction, -255, 255);

  // Drive motors using TB6612FNG
  run(leftMotorSpeed, rightMotorSpeed);
}

void fw(int timeLimiter, int speed, String detector, String action) {
  unsigned long startTime = millis();
  _baseSpeed = speed;
  unsigned long detectionDelay = 100;  // Wait 100 ms before checking

  while (millis() - startTime < timeLimiter) {
    PIDforward();

    delay(1);
    if (detector == "p") continue;
    if (detector == "f" && millis() - startTime > detectionDelay) {
      if ((_frontValues[0] < _frontThreshold[0] && _frontValues[1] < _frontThreshold[1] && _frontValues[2] < _frontThreshold[2]) || (_frontValues[2] < _frontThreshold[2] && _frontValues[3] < _frontThreshold[3] && _frontValues[4] < _frontThreshold[4])) {

        if (action == "s") {
          //BREAK
          run(0, 0);
          delay(10);
          run(constrain(_biasL - _baseSpeed, -255, 255), constrain(_biasR - _baseSpeed, -255, 255));
          delay(60);
          run(0, 0);

          getSensorValues();
          while (_frontValues[2] < _frontThreshold[2]) {
            getSensorValues();
            run(_speed_slow, _speed_slow);
            delay(500);
          }
          run(0, 0);
        } else if (action == "l") {
          //BREAK
          run(0, 0);
          delay(10);
          getSensorValues();
          while (_frontValues[2] < _frontThreshold[2]) {
            getSensorValues();
            run(_speed_slow, _speed_slow);
            delay(500);
          }
          spin_l_to_line(157);
        } else if (action == "r") {
          //BREAK
          run(0, 0);
          delay(10);
          getSensorValues();
          while (_frontValues[2] < _frontThreshold[2]) {
            getSensorValues();
            run(_speed_slow, _speed_slow);
            delay(100);
          }
          spin_r_to_line(157);
        }
        break;
      }
    }
  }
}

void obj_release() {
  gServoL.write(180);  //น้อย >> หุบ
  gServoR.write(180);  //น้อย >> หุบ
  delay(200);
  gServoArm.write(90);  //น้อย >> ลง
  delay(200);
}

void obj_prepare() {
  gServoL.write(0);
  delay(200);
  //gServoL.write(0);
  //gServoR.write(180);  //น้อย >> หุบ
  //delay(200);
  //gServoArm.write(15);
  //delay(200);
}

void obj_catch() {
  gServoL.write(90);
  gServoR.write(180);  //น้อย >> หุบ
  delay(200);
}

void wait_SW1() {
  while (1) {
    Serial.println("waiting...");
    delay(10);
    if (digitalRead(BUTTON_PIN) == LOW) {
      tone(18, 660, 100);
      delay(150);
      tone(18, 660, 100);
      break;
    }
  }
}

void wait_SW1_done() {
  while (1) {
    Serial.println("waiting...");
    delay(10);
    if (digitalRead(BUTTON_PIN) == LOW) {
      tone(18, 660, 300);
      break;
    }
  }
}
