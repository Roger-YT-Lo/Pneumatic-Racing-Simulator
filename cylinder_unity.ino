#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ────────── MPU6050 與濾波 ──────────
Adafruit_MPU6050 mpu;
float filteredAngleX = 0.0, filteredAngleY = 0.0;
const float filterAlpha = 0.5;

// ────────── 容許誤差 & 物理極限 ──────────
const float ANGLE_TOLERANCE_X = 1;     // ±1.0°
const float ANGLE_TOLERANCE_Y = 2.0;   // ±2.0°
const float MAX_ANGLE_X       = 4.5;   // ±4.5°
const float MAX_ANGLE_Y       = 9.0;   // ±9.0°

float baselineX = 0.0;
float baselineY = 0.0;

// 氣缸腳位 (左前、左後、右前、右後)
const int pushPins[4] = {2, 4, 6, 8};
const int pullPins[4] = {3, 5, 7, 9};

// 按鈕腳位
const int interruptButtonPin = 10;
const int exitButtonPin      = 11;

// 狀態旗標
bool isInterruptMode = false;  
bool isExitMode      = false;  
bool isGameMode      = false;  

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for (int i = 0; i < 4; ++i) {
    pinMode(pushPins[i], OUTPUT);
    pinMode(pullPins[i], OUTPUT);
    digitalWrite(pushPins[i], LOW);
    digitalWrite(pullPins[i], LOW);
  }
  pinMode(interruptButtonPin, INPUT_PULLUP);
  pinMode(exitButtonPin,      INPUT_PULLUP);
}

void loop() {
  updateMPU();
  checkInterruptButton();
  handleExitButton();

  if (isGameMode && !isInterruptMode && !isExitMode && Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length()) parseUnityData(data);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    float seatX = filteredAngleX - baselineX; 
    float seatY = filteredAngleY - baselineY;
    /*
    Serial.print("SeatAngle => X: ");
    Serial.print(seatX,1);
    Serial.print(", Y: ");
    Serial.println(seatY,1);
*/
  }
  delay(10);
}

// 按鈕處理
void checkInterruptButton() {
  bool pressed = (digitalRead(interruptButtonPin) == LOW);
  if (pressed && !isInterruptMode) {
    isInterruptMode = true;
    seatNeutralPosition();
  } 
  else if (!pressed && isInterruptMode) {
    isInterruptMode = false;
    baselineX = filteredAngleX;
    baselineY = filteredAngleY;
    isGameMode = true;
  }
}

void handleExitButton() {
  bool pressed = digitalRead(exitButtonPin) == LOW;
  if (pressed) {
    isExitMode = true;
    isGameMode = false;
    for (int i = 0; i < 4; ++i) {
      digitalWrite(pushPins[i], LOW);
      digitalWrite(pullPins[i], HIGH);
    }
  } 
  else if (isExitMode && !pressed) {
    isExitMode = false;
    stopAllCylinders();
  }
}

// 置平：全部拉→全部推
void seatNeutralPosition() {
  for (int i = 0; i < 4; ++i) digitalWrite(pullPins[i], HIGH);
  delay(1000);
  stopAllCylinders();
  for (int i = 0; i < 4; ++i) digitalWrite(pushPins[i], HIGH);
  delay(100);
  stopAllCylinders();
}

// Unity 指令解析
void parseUnityData(const String& str) {
  int idx = str.indexOf(',');
  if (idx < 0) return;

  float tx = str.substring(0, idx).toFloat();
  float ty = str.substring(idx + 1).toFloat();
  if (isnan(tx) || isnan(ty)) return;
  if (fabs(tx) < 0.001 && fabs(ty) < 0.001) return;

  tx = constrain(tx, -MAX_ANGLE_X, MAX_ANGLE_X);
  ty = constrain(ty, -MAX_ANGLE_Y, MAX_ANGLE_Y);

  singleMoveWithXY(tx, ty);
}

// 單次動作
void singleMoveWithXY(float tx, float ty) {
  float seatX = filteredAngleX - baselineX;
  float seatY = filteredAngleY - baselineY;

  float errX  = tx - seatX;
  float errY  = ty - seatY;

  // 若 X、Y 誤差都在容許範圍內，則不動作
  if (fabs(errX) < ANGLE_TOLERANCE_X && fabs(errY) < ANGLE_TOLERANCE_Y) return;

  int tX = map(int(min(fabs(errX), MAX_ANGLE_X) * 100), 0, 700, 50, 150);
  int tY = map(int(min(fabs(errY), MAX_ANGLE_Y) * 100), 0, 900, 50, 400);
  int t  = max(tX, tY);

  bool push[4] = {false}, pull[4] = {false};

  // X 軸邏輯
  if (fabs(errX) > ANGLE_TOLERANCE_X) {
    if (errX > 0) {
      // X正：左前(0)、右前(2) 拉；左後(1)、右後(3) 推
      pull[0] = true; pull[2] = true;
      push[1] = true; push[3] = true;
    } 
    else {
      // X負：左前(0)、右前(2) 推；左後(1)、右後(3) 拉
      push[0] = true; push[2] = true;
      pull[1] = true; pull[3] = true;
    }
  }

  // Y 軸邏輯：
  // ★ 如果 Unity 傳來的 ty == 0 → 跳過 Y 軸控制
  if (ty != 0 && fabs(errY) > ANGLE_TOLERANCE_Y) {
    if (errY > 0) {
      // Y正：左前(0)、左後(1) 推；右前(2)、右後(3) 拉
      push[0] = true; push[1] = true;
      pull[2] = true; pull[3] = true;
    } 
    else {
      // Y負：左前(0)、左後(1) 拉；右前(2)、右後(3) 推
      pull[0] = true; pull[1] = true;
      push[2] = true; push[3] = true;
    }
  }

  // 同步啟動
  for (int i = 0; i < 4; ++i) {
    if (push[i]) digitalWrite(pushPins[i], HIGH);
    if (pull[i]) digitalWrite(pullPins[i], HIGH);
  }

  delay(t);
  stopAllCylinders();
}

// 工具函式
void stopAllCylinders() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pushPins[i], LOW);
    digitalWrite(pullPins[i], LOW);
  }
}

void updateMPU() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float rawX = atan2(a.acceleration.x,
                     sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0 / PI;
  float rawY = atan2(-a.acceleration.y,
                     sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z)) * 180.0 / PI;

  filteredAngleX = filterAlpha * rawX + (1 - filterAlpha) * filteredAngleX;
  filteredAngleY = filterAlpha * rawY + (1 - filterAlpha) * filteredAngleY;
}
