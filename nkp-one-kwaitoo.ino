//ใช้กับ esp32 espressif 2.xx เท่านั้น
#include <NKP_ONE.h>


uint16_t state_on_Line = 0;

uint8_t numSensor = 7;

void Run_fast(int delay_timer) {
  float Kp = 4;
  float Ki = 0;
  float Kd = 50;
  uint16_t setpoint;
  float present_position;
  float errors = 0;
  float output = 0;
  float integral;
  float derivative;
  float previous_error;
  long timer = 0;
  timer = millis();
  do {
    int speed_max = 80;
    present_position = readline() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    integral = integral + errors;
    derivative = (errors - previous_error);
    output = Kp * errors + Ki * integral + Kd * derivative;
    int max_output = 100;
    previous_error = errors;
    if (output > max_output) output = max_output;
    else if (output < -max_output) output = -max_output;
    motor(1, speed_max - output);
    motor(2, speed_max + output);
    delay(1);

  } while ((millis() - timer) < delay_timer);
}
void Run_slow(int delay_timer) {
  float Kp = 4;
  float Ki = 0;
  float Kd = 50;
  uint16_t setpoint;
  float present_position;
  float errors = 0;
  float output = 0;
  float integral;
  float derivative;
  float previous_error;
  long timer = millis();
  do {
    int speed_max = 40;
    present_position = readline() / ((numSensor - 1) * 10);
    setpoint = 50.0;
    errors = setpoint - present_position;
    integral = integral + errors;
    derivative = (errors - previous_error);
    output = Kp * errors + Ki * integral + Kd * derivative;
    int max_output = 100;
    previous_error = errors;
    if (output > max_output) output = max_output;
    else if (output < -max_output) output = -max_output;
    motor(1, speed_max - output);
    motor(2, speed_max + output);
    delay(1);

  } while (millis() - timer < delay_timer);
}


void Run_until(String detectors) {

  float Kp = 0.5;
  float Ki = 0;
  float Kd = 0.25;

  float setpoint = 50.0;
  float present_position;
  float error = 0;
  float output = 0;

  float integral = 0;
  float derivative = 0;
  float previous_error = 0;

  int speed_max = 15;
  int max_output = 100;

  do {

    /* ---------- PID FOLLOW LINE ---------- */
    present_position = readline() / ((numSensor - 1) * 10);
    error = setpoint - present_position;

    integral += error;
    derivative = error - previous_error;
    previous_error = error;

    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, -max_output, max_output);

    motor(1, speed_max - output);
    motor(2, speed_max + output);

    delay(1);

    /* ---------- ตรวจเซนเซอร์ขอบ ---------- */
    bool left_white =
      analogRead(_sensorPins[0]) > _min_sensor_values[0] + 300 && analogRead(_sensorPins[1]) > _min_sensor_values[1] + 300;

    bool right_white =
      analogRead(_sensorPins[5]) > _min_sensor_values[5] + 300 && analogRead(_sensorPins[6]) > _min_sensor_values[6] + 300;
    bool head_white = analogRead(_sensorPins[0]) > _min_sensor_values[0] + 300;
    bool mid_left_white = analogRead(A7) > 600 + 300;
    bool mid_right_white = analogRead(A8) > 450 + 300;
    
    /* วิ่งต่อเมื่อขอบยังขาวทั้งหมด */

    /* if (!(left_white && right_white)) {
      break;   // เจอเส้นดำ → ออกจาก loop
    } */
    if (detectors == "mid_2") {
      if (!mid_left_white && !mid_right_white) {
        break;  // เจอเส้นดำคู่ → ออกจาก loop
      }
    }
    else if(detectors == "mid_l"){
      if (head_white && !mid_left_white) {
        break;  // เจอเส้นดำซ้าย → ออกจาก loop
      }
    }
    else if(detectors == "mid_r"){
      if (head_white && !mid_right_white) {
        break;  // เจอเส้นดำขวา → ออกจาก loop
      }
    }




  } while (1);
}



void Run_motor(int speed) {
  motor(1, speed);
  motor(2, speed);
}

void Spin_l(int speed, int time_lim) {
  motor(1, speed * -1);
  motor(2, speed);
  delay(100);
}

void Move_backward(int speed, int time_lim) {
  // Both motors run in reverse
  motor(1, -speed);
  motor(2, -speed);

  // Keep moving for the given time
  delay(time_lim);

  // Stop motors after moving backward
  motor(1, 0);
  motor(2, 0);
}

void setup() {
  NKP_ONE();
  Serial.begin(115200);
  IO15();
  setSensorPins((const int[]){ A0, A1, A2, A3, A4, A5, A6 }, numSensor);
  setSensorMax((const uint16_t[]){ 3800, 2000, 2700, 2000, 1800, 2500, 4000 });
  setSensorMin((const uint16_t[]){ 560, 400, 260, 900, 490, 530, 640 });
  /*   for (int i = 0; i < 3; i++) {
    for(int l = 0;l<300;l++){
      setCalibrate();
      delay(1);
      motor(1,30);
      motor(2,30);  
    }
    aot(100);
    for(int l = 0;l<300;l++){
      setCalibrate();
      delay(1);
      motor(1,-30);
      motor(2,-30);  
    }aot(100);
    
    for(int l = 0;l<200;l++){
      setCalibrate();
      delay(1);
      motor(1,-30);
      motor(2,-30);
    }aot(100);
    for(int l = 0;l<300;l++){
      setCalibrate();
      delay(1);
      motor(1,30);
      motor(2,30);  
    }
    aot(100);
    
  } */
  aot(100);  //หยุดค้าไว้

  for (uint8_t i = 0; i < numSensor; i++) {
    Serial.print(ReadSensorMinValue(i));
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < numSensor; i++) {
    Serial.print(ReadSensorMaxValue(i));
    Serial.print(' ');
  }
  IO15();
}

void Stop_At_Line() {
  Run_until("mid_2"); // วิ่งจนเจอเส้นดำ (A7, A8)
  ao();               // หยุดมอเตอร์
  delay(1000);        // รอ 1 วินาที
}

// --- ฟังก์ชันที่ 2: เดินไปเจอเส้นดำ หยุด 1 วิ แล้วเลี้ยวซ้ายตั้งฉาก ---
void Line_And_TurnLeft(int turn_speed, int turn_time) {
  Run_until("mid_2");    // วิ่งจนเจอเส้นดำ
  ao();                  // หยุดมอเตอร์
  delay(1000);           // รอ 1 วินาที
  
  // เลี้ยวซ้ายตั้งฉาก (ใช้ฟังก์ชัน Spin_l ที่คุณมี)
  Spin_l(turn_speed, turn_time); 
  ao();
  delay(300);
}

void Run_until_Detect(String mode) {
  // ค่า threshold: เจอดำค่าจะน้อย (ต่ำกว่า 600) ถ้าหุ่นไม่หยุดให้ลองเพิ่มเป็น 700 หรือ 800
  int threshold = 800; 
  
  while (true) {
    /* ---------- PID FOLLOW LINE ---------- */
    // อ่านค่าตำแหน่งเซนเซอร์ 7 ดวง (A0-A6)
    float present_position = readline() / ((numSensor - 1) * 10);
    float error = 50.0 - present_position;
    
    // ใช้ความเร็ว 20 เพื่อให้เซนเซอร์ A7, A8 ตรวจจับเส้นได้แม่นยำ ไม่ไถล
    motor(1, 20 - (0.5 * error)); 
    motor(2, 20 + (0.5 * error));
    delay(1);

    /* ---------- ตรวจเซนเซอร์หยุด (A7, A8) ---------- */
    int back_L = analogRead(A7);
    int back_R = analogRead(A8);

    if (mode == "LEFT") {
      if (back_L < threshold) { 
        ao();          // หยุดมอเตอร์
        delay(1000);   // หยุดรอ 1 วินาทีตามสั่ง
        break;         // ออกจาก Loop เพื่อไปทำงานคำสั่งถัดไป (เช่น เลี้ยวซ้าย)
      }
    } 
    else if (mode == "RIGHT") {
      if (back_R < threshold) { 
        ao();          // หยุดมอเตอร์
        delay(1000);   // หยุดรอ 1 วินาทีตามสั่ง
        break;         // ออกจาก Loop เพื่อไปทำงานคำสั่งถัดไป (เช่น เลี้ยวขวา)
      }
    }
  }
}

void loop() {
  delay(1000);

  // 1. เดินหน้าตรงไป จนเซนเซอร์ A7 (ซ้ายหลัง) เจอเส้นดำ -> หยุด 1 วิ -> แล้วเดินต่อทันที
  Run_until_Detect("LEFT");
   // 2. เดินหน้าตรงไป จนเซนเซอร์ A8 (ขวาหลัง) เจอเส้นดำ -> หยุด 1 วิ 
  Run_until_Detect("RIGHT");

  // 3. เมื่อหยุดครบ 1 วิจากฟังก์ชันบนแล้ว ให้สั่งเลี้ยวซ้ายตั้งฉาก
  // ปรับค่า 40, 850 ตามความแรงมอเตอร์เพื่อให้ได้ 90 องศา
  Spin_l(100, 850);
   ao();
  while (1); 
}
