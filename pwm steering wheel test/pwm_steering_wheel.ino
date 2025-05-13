//AS5048A 使用SPI通訊
//平滑力回饋方向盤控制
//無自檢系統，直接在代碼中調整角度
//2025.3.18
//雙馬達同步運作版本
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

// 力回饋參數
float centerAngle = 0.0;     // 中心位置設為0度
float deadZone = 20;         // 中心死區(度)，避免中心點抖動
int minPWM = 60;             // 最小有效PWM值
float initialPosition = 0.0; // 儲存初始位置
float forceGain = 0.9;       // 力回饋增益係數
float maxAngle = 500;        // 最大角度範圍

// 多轉追蹤變數
int fullRotations = 0;        // 完整旋轉圈數
float lastAngle = 0.0;        // 上次讀取的角度

// 平滑控制變數
float currentForce = 0.0;     // 當前馬達力值
float targetForce = 0.0;      // 目標馬達力值
float smoothFactor = 0.1;     // 力值平滑係數（值越小，啟動越緩慢）

void setup() {
  Serial.begin(115200);
  
  // 初始化感測器
  angleSensor.begin();
  
  // 初始化第一個馬達控制針腳
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // 初始化第二個馬達控制針腳
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // 先停止所有馬達
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  Serial.println("平滑方向盤力回饋系統初始化 (雙馬達版)");
  Serial.println("- 180度位置將自動調整為0度");
  Serial.println("- 順時針為正，逆時針為負");
  Serial.println("- 範圍支援+-500度");
  
  // 等待感測器穩定
  delay(1000);
  
  // 讀取當前角度
  float rawAngle = angleSensor.getRotationInDegrees();
  Serial.print("原始角度讀數: ");
  Serial.print(rawAngle);
  Serial.println("°");
  
  // 修正角度 - 將180度調整為0度
  float adjustedAngle;
  if (rawAngle >= 180) {
    adjustedAngle = rawAngle - 180;
  } else {
    adjustedAngle = rawAngle + 180;
  }
  
  Serial.print("調整後角度: ");
  Serial.print(adjustedAngle);
  Serial.println("°");
  
  // 初始化角度追蹤
  lastAngle = adjustedAngle;
  
  Serial.println("系統準備就緒");
  Serial.println("------------------------");
}

void loop() {
  // 讀取當前角度
  float rawAngle = angleSensor.getRotationInDegrees();
  
  // 修正角度 - 將180度調整為0度
  float currentRawAngle;
  if (rawAngle >= 180) {
    currentRawAngle = rawAngle - 180;
  } else {
    currentRawAngle = rawAngle + 180;
  }
  
  // 處理多圈旋轉（超過360度範圍）
  // 檢測圈數變化 (使用修正後的角度)
  if (lastAngle > 270 && currentRawAngle < 90) {
    // 順時針穿過零點
    fullRotations++;
  } else if (lastAngle < 90 && currentRawAngle > 270) {
    // 逆時針穿過零點
    fullRotations--;
  }
  
  // 儲存當前修正角度用於下次比較
  lastAngle = currentRawAngle;
  
  // 計算絕對角度（包含多圈）
  float currentAngle = currentRawAngle + (fullRotations * 360);
  
  // 限制在+-maxAngle度範圍
  if (currentAngle > maxAngle) {
    currentAngle = maxAngle;
  } else if (currentAngle < -maxAngle) {
    currentAngle = -maxAngle;
  }
  
  // 計算與中心點的差異
  float angleDiff = -currentAngle;  // 注意負號：角度為正，需要反向力拉回
  
  // 計算理想力回饋
  targetForce = calculateForce(angleDiff);
  
  // 平滑漸進到目標力值（軟啟動和軟停止）
  currentForce = currentForce * (1 - smoothFactor) + targetForce * smoothFactor;
  
  // 應用平滑後的力回饋到兩個馬達
  applyDualMotorForce((int)currentForce);
  
  // 顯示信息
  Serial.print("角度: ");
  Serial.print(currentAngle, 2);
  Serial.print(" | 偏差: ");
  Serial.print(angleDiff, 2);
  Serial.print(" | 目標力: ");
  Serial.print(targetForce);
  Serial.print(" | 實際力: ");
  Serial.println(currentForce);
  
  // 減少診斷信息的顯示頻率
  static int counter = 0;
  if (counter++ % 100 == 0) {
    int agcValue = angleSensor.getGain();
    Serial.print("AGC值: ");
    Serial.println(agcValue);
    
    String errors = angleSensor.getErrors();
    if (errors != "") {
      Serial.print("錯誤: ");
      Serial.println(errors);
    }
    
    String diagnostic = angleSensor.getDiagnostic();
    if (diagnostic != "") {
      Serial.print("診斷: ");
      Serial.println(diagnostic);
    }
    
    Serial.println("------------------------");
  }
  
  delay(20);  // 更新頻率
}

// 改進的指數曲線力回饋計算
int calculateForce(float angleDiff) {
  // 如果偏差在中心死區範圍內，不產生力回饋
  if (abs(angleDiff) < deadZone) {
    return 0;
  }
  
  // 計算從死區到最大範圍的標準化位置 (0.0 - 1.0)
  float normalizedPos = (abs(angleDiff) - deadZone) / (maxAngle - deadZone);
  normalizedPos = constrain(normalizedPos, 0.0, 1.0);
  
  // 應用更高指數曲線 (從3.0增加到4.0，讓初始段更平緩)
  float forceCurve = pow(normalizedPos, 4.0);
  
  // 應用增益係數並標準化到PWM範圍
  float forceMagnitude = forceGain * forceCurve * 255.0;
  
  // 保持方向
  if (angleDiff < 0) {
    forceMagnitude = -forceMagnitude;
  }
  
  // 將力轉換為PWM值，同時應用最小PWM閾值
  int pwmValue;
  if (abs(forceMagnitude) > 0) {
    pwmValue = map(constrain(abs(forceMagnitude), 0, 255), 0, 255, minPWM, 255);
  } else {
    pwmValue = 0;
  }
  
  // 保持方向
  return (forceMagnitude >= 0) ? pwmValue : -pwmValue;
}

// 應用力回饋到兩個馬達，使它們一起運作
void applyDualMotorForce(int force) {
  if (abs(force) < 5) {
    // 停止所有馬達
    // 第一個馬達
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    
    // 第二個馬達
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } 
  else if (force > 0) {
    // 正方向轉動 - 將方向盤拉向中心點
    // 第一個馬達
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(force));
    
    // 第二個馬達
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(force));
  } 
  else {
    // 負方向轉動 - 將方向盤拉向中心點
    // 第一個馬達
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(force));
    
    // 第二個馬達
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(force));
  }
}
