#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include "DHTesp.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>



Adafruit_SSD1306 OLED(-1);
Servo gServoArm;
Servo gServoCraw;
DHTesp dht;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

// กำหนดชื่อGPIO มอเตอร์ฝั่งซ้าย
#define _DR1 2
#define _DR2 15
#define _PWMR 13
// กำหนดชื่อGPIO มอเตอร์ฝั่งขวา
#define _DL1 16
#define _DL2 17
#define _PWML 4

#define BUTTON_PIN 27

#define _DHTPIN 23  // ขา Data ต่อกับ D23 ของ ESP32
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


int _NumofSensor = 5;
int _Sensitive = 20;
int _lastPosition = 0;

int _frontMin[5] = { 100, 100, 100, 100, 100 };
int _frontMax[5] = { 100, 100, 100, 100, 100 };
int _frontValues[5];
int _frontThreshold[5];
int _frontPins[5] = { 26, 25, 14, 39, 36 };

int _baseSpeed = 110;  // Set your base speed here (range 0-255, adjust as needed)
int _biasL = 0;
int _biasR = 0;

float _kp = 0.1, _ki = 0.0, _kd = 0;  // kp(0.1-15.0) kd(0.05)
float _previousError = 0;
float _integral = 0;

bool _deviceConnected = false;
bool _oldDeviceConnected = false;
uint32_t _value = 0;



class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    _deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    _deviceConnected = false;
  }
};

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

void ble_notify(String msg) {
  // notify changed value
  if (_deviceConnected) {

    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(500);
  }
  // disconnecting
  if (!_deviceConnected && _oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    _oldDeviceConnected = _deviceConnected;
  }
  // connecting
  if (_deviceConnected && !_oldDeviceConnected) {
    // do stuff here on connecting
    _oldDeviceConnected = _deviceConnected;
  }
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

void init_dht11(int dht_pin){
  dht.setup(dht_pin, DHTesp::DHT11);
}
void init_robot() {
  Serial.begin(115200);
  pinMode(27, INPUT);  // กำหนดขา 27 เป็น input
  pinMode(_DL1, OUTPUT);
  pinMode(_DL2, OUTPUT);
  pinMode(_PWML, OUTPUT);
  pinMode(_DR1, OUTPUT);
  pinMode(_DR2, OUTPUT);
  pinMode(_PWMR, OUTPUT);
  gServoArm.attach(32, 500, 2400);
  gServoCraw.attach(33, 500, 2400);
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // กำหนดแอดเดรสของพอร์ตจอเป็น 0x3C (for the 128x64)
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

void init_ble(String ble_name) {
  // Create the BLE Device
  BLEDevice::init(ble_name);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
  // Descriptor 2902 is not required when using NimBLE as it is automatically added based on the characteristic properties
  pCharacteristic->addDescriptor(new BLE2902());
  // Adds also the Characteristic User Description - 0x2901 descriptor
  descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription("My own description for this characteristic.");
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  pCharacteristic->addDescriptor(descriptor_2901);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  show4lines("Waiting", "Connection...", "", "", "", "", "", "");
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
  if (error == 0) _integral = 0;
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
            run(_baseSpeed, _baseSpeed);
          }
          run(0, 0);
        } else if (action == "l") {
          //BREAK
          run(0, 0);
          delay(10);
          getSensorValues();
          while (_frontValues[2] < _frontThreshold[2]) {
            getSensorValues();
            run(_baseSpeed, _baseSpeed);
          }
          spin_l_to_line(157);
        } else if (action == "r") {
          //BREAK
          run(0, 0);
          delay(10);
          getSensorValues();
          while (_frontValues[2] < _frontThreshold[2]) {
            getSensorValues();
            run(_baseSpeed, _baseSpeed);
          }
          spin_r_to_line(157);
        }
        break;
      }
    }
  }
}

void obj_release() {
  gServoCraw.write(180);  //น้อย >> หุบ
  delay(200);
  gServoArm.write(90);  //น้อย >> ลง
  delay(200);
}

void obj_prepare() {
  gServoCraw.write(180);
  delay(200);
  gServoArm.write(15);
  delay(200);
}

void obj_catch() {
  gServoCraw.write(90);
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
