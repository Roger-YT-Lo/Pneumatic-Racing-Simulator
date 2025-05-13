//AS5048A 使用SPI通訊
//真實物理模型力回饋方向盤控制
//2025.3.18
#include <AS5048A.h>

// AS5048A 感測器設定
AS5048A angleSensor(7, true);

// L298N 馬達驅動器設定
// 第一個馬達
const int IN1 = 8;  // 方向控制腳1
const int IN2 = 9;  // 方向控制腳2
const int ENA = 10;  // PWM速度控制腳A

// 第二個馬達
const int IN3 = 11;  // 方向控制腳3
const int IN4 = 12;  // 方向控制腳4
const int ENB = 13;  // PWM速度控制腳B

// 力回饋參數定義
#define CENTERING_GAIN 3.0       // 基本回正力增益
#define SPEED_GAIN 1.5           // 速度相關力增益
#define MAX_SPEED 100.0          // 最大車速參考值
#define ROAD_FEEDBACK_GAIN 0.8   // 路面反饋增益
#define DYNAMIC_FORCE_GAIN 0.6   // 動態效應增益
#define PWM_DEADZONE 45          // PWM死區值

// 中心點和測量值
uint16_t centerRawValue = 0;     // 儲存中心點原始值
float currentSpeed = 0.0;        // 當前車速 (模擬值)
float roadCondition = 0.0;       // 路面狀況 (0.0-平滑, 1.0-粗糙)
float vehicleDynamics = 0.0;     // 車輛動態 (轉向慣性等)

// 輔助函數: 獲取數字符號
float sign(float value) {
  if (value > 0) return 1.0;
  if (value < 0) return -1.0;
  return 0.0;
}

void setup() {
  Serial.begin(115200);
  
  // 初始化感測器
  angleSensor.begin();
  
  // 初始化馬達控制針腳
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // 先停止馬達
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  Serial.println("方向盤力回饋系統初始化 - 真實物理模型");
  Serial.println("等待3秒，請將方向盤置於中心位置...");
  delay(3000);
  
  // 讀取多次中心點位置取平均值
  centerRawValue = 0;
  for(int i = 0; i < 10; i++) {
    centerRawValue += angleSensor.getRawRotation();
    delay(50);
  }
  centerRawValue /= 10;
  
  Serial.print("中心點原始值已設定為: 0x");
  Serial.println(centerRawValue, HEX);
  Serial.println("------------------------");
  
  // 設定模擬參數初始值
  currentSpeed = 50.0;  // 模擬中等車速
  roadCondition = 0.2;  // 較平坦的路面
}

void loop() {
  // 讀取當前角度原始值
  uint16_t rawAngle = angleSensor.getRawRotation();
  
  // 計算與中心點的差異 (取最短路徑)
  int16_t rawDiff = rawAngle - centerRawValue;
  
  // 處理環繞情況 (AS5048A是14位元，最大值為16383)
  if (rawDiff > 8192) rawDiff -= 16384;
  if (rawDiff < -8192) rawDiff += 16384;
  
  // 轉換為角度 (-180 到 +180 度)
  float steeringAngle = (rawDiff / 16384.0) * 360.0;
  steeringAngle -= 144;
  //if steeringAngle<240
  
  
  // 計算力回饋
  float force = calculateRealisticForce(steeringAngle, currentSpeed, roadCondition);
  
  // 應用力回饋到馬達
  applyMotorForce(force);
  
  // 顯示信息
  Serial.print("原始值: 0x");
  Serial.print(rawAngle, HEX);
  Serial.print(" | 角度: ");
  Serial.print(steeringAngle, 2);
  Serial.print("° | 力回饋: ");
  Serial.print(force);
  
  // 每10次循環更新模擬參數（實際應從遊戲接收）
  static int counter = 0;
  if (++counter % 10 == 0) {
    // 模擬車速變化 (在現實應用中，這些數據應從Unity接收)
    currentSpeed = 30.0 + 20.0 * sin(millis() / 10000.0);
    roadCondition = 0.2 + 0.1 * sin(millis() / 5000.0);
    
    Serial.print(" | 速度: ");
    Serial.print(currentSpeed, 1);
    Serial.print(" | 路面: ");
    Serial.println(roadCondition, 2);
  } else {
    Serial.println();
  }
  
  delay(20);  // 50Hz更新頻率
}

// 更真實的力回饋計算函數
float calculateRealisticForce(float steeringAngle, float vehicleSpeed, float roadCondition) {
  // 1. 基本回正力 - 非線性增加
  float centeringForce = CENTERING_GAIN * pow(abs(steeringAngle), 1.8) * -sign(steeringAngle);
  
  // 2. 速度相關力 - 隨速度平方增加
  float speedFactor = 1.0 + SPEED_GAIN * pow(vehicleSpeed / MAX_SPEED, 2);
  
  // 3. 路面反饋 (簡化模型)
  float roadFeedback = calculateRoadFeedback(roadCondition);
  
  // 4. 動態行為力 (簡化模型)
  float dynamicForce = calculateDynamicEffects(steeringAngle, vehicleSpeed);
  
  // 組合所有力回饋
  float totalForce = centeringForce * speedFactor + roadFeedback + dynamicForce;
  
  // 將力轉換為PWM值，應用死區補償 (-255 到 +255)
  float pwmForce = constrain(totalForce, -255, 255);
  
  // 應用PWM死區補償
  if (abs(pwmForce) < 10) {
    return 0;  // 避免微小抖動
  } else if (pwmForce > 0) {
    return map(pwmForce, 10, 255, PWM_DEADZONE, 255);
  } else {
    return map(pwmForce, -10, -255, -PWM_DEADZONE, -255);
  }
}

// 路面反饋計算 (在真實應用中應從遊戲接收)
float calculateRoadFeedback(float roadCondition) {
  // 簡化模型：路面粗糙度產生震動
  float vibration = ROAD_FEEDBACK_GAIN * roadCondition * sin(millis() / 10.0);
  return vibration;
}

// 車輛動態效應計算 (在真實應用中應從遊戲接收)
float calculateDynamicEffects(float steeringAngle, float vehicleSpeed) {
  // 簡化模型：轉向時的慣性阻力
  float inertiaEffect = DYNAMIC_FORCE_GAIN * vehicleSpeed/100.0 * sin(steeringAngle * PI / 180.0);
  return inertiaEffect;
}

// 應用力回饋到馬達
void applyMotorForce(int force) {
  if (abs(force) < 5) {
    // 停止馬達
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } 
  else if (force > 0) {
    // 正方向轉動
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(force));
  } 
  else {
    // 負方向轉動
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(force));
  }
}
