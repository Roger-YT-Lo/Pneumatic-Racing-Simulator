//AS5048A 使用SPI通訊
//開機位置設為中心點(0度)，限制角度範圍在±180度內
//具有兩段式馬達力回饋
//2025.3.30


#include <AS5048A.h>

// AS5048A 感測器設定
AS5048A angleSensor(7, true);

// L298N 馬達驅動器設定 - 不使用ENA/ENB (假設已接跳帽)
const int IN1 = 8;   // 方向控制腳1
const int IN2 = 9;   // 方向控制腳2
const int IN3 = 11;  // 方向控制腳3 (第二馬達)
const int IN4 = 12;  // 方向控制腳4 (第二馬達)

// 力回饋參數
float centerAngle = 0.0;     // 中心位置設為0度
float deadZone = 3;          // 中心死區(度)，在此範圍內不產生力回饋
float forceZone1 = 25.0;     // 第一級力回饋區域 (使用單馬達)
float forceZone2 = 60.0;     // 第二級力回饋區域 (使用雙馬達)
float maxAngle = 180.0;      // 最大顯示角度範圍（±180度）

// 雙角度追蹤變數
float lastRawAngle = 0.0;        // 上次讀取的原始角度
float displayAngle = 0.0;        // 顯示角度（限制在±180度內）
float actualAngle = 0.0;         // 實際角度（可以超過±180度）
bool overLimitPositive = false;  // 是否超過+180度限制
bool overLimitNegative = false;  // 是否超過-180度限制

// 力回饋控制變數
bool forceFeedbackEnabled = true; // 控制是否啟用力回饋
bool centeringMode = true;       // 啟用自動回中模式

// 通信控制變數
unsigned long lastSendTime = 0;  // 上次發送數據的時間
int sendInterval = 20;           // 發送間隔(毫秒)
const char* anglePrefix = "SW:"; // 方向盤角度識別前綴

void setup() {
  Serial.begin(115200);
  
  // 初始化感測器
  angleSensor.begin();
  
  // 初始化馬達控制針腳
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // 先停止馬達
  stopMotors();
  
  // 等待感測器穩定
  delay(1000);
  
  // 將當前位置設為零點（中心點）
  float currentRawAngle = getFilteredAngle();
  uint16_t currentZero = angleSensor.getZeroPosition();
  int16_t zeroShift = (currentRawAngle * 16384.0 / 360.0); // 轉換角度到感測器原始值
  
  // 設定新的零點
  angleSensor.setZeroPosition(currentZero - zeroShift);
  delay(100); // 等待零點設定生效
  
  // 初始化上次角度
  lastRawAngle = angleSensor.getRotationInDegrees();
  
  // 確保馬達初始化為停止狀態
  stopMotors();
}

// 明確停止所有馬達
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // 讀取當前角度
  float currentRawAngle = angleSensor.getRotationInDegrees();
  
  // 計算方向盤角度差異（追踪過零點的連續角度）
  float angleDiff = currentRawAngle - lastRawAngle;
  
  // 處理過零點的角度跳變
  if (angleDiff > 180) {
    // 從359度到0度的逆時針轉動
    angleDiff -= 360;
  } else if (angleDiff < -180) {
    // 從0度到359度的順時針轉動
    angleDiff += 360;
  }
  
  // 更新實際角度（無限制）
  actualAngle += angleDiff;
  
  // 更新顯示角度，但監測是否跨過180度邊界
  float newDisplayAngle = displayAngle + angleDiff;
  
  // 檢查是否超過顯示限制
  if (newDisplayAngle > maxAngle && !overLimitPositive) {
    // 首次超過+180度
    overLimitPositive = true;
    overLimitNegative = false;
    newDisplayAngle = maxAngle; // 限制在+180度
  } 
  else if (newDisplayAngle < -maxAngle && !overLimitNegative) {
    // 首次超過-180度
    overLimitNegative = true;
    overLimitPositive = false;
    newDisplayAngle = -maxAngle; // 限制在-180度
  }
  
  // 檢查是否從超限狀態返回
  if (overLimitPositive && newDisplayAngle < maxAngle * 0.9) {
    // 從+180度超限狀態返回
    overLimitPositive = false;
  }
  else if (overLimitNegative && newDisplayAngle > -maxAngle * 0.9) {
    // 從-180度超限狀態返回
    overLimitNegative = false;
  }
  
  // 應用顯示角度限制
  if (overLimitPositive) {
    displayAngle = maxAngle;
  } else if (overLimitNegative) {
    displayAngle = -maxAngle;
  } else {
    // 正常範圍內，直接更新顯示角度
    displayAngle = newDisplayAngle;
  }
  
  // 更新上次讀取的角度
  lastRawAngle = currentRawAngle;
  
  // 如果啟用了力回饋，計算並應用
  if (forceFeedbackEnabled) {
    int force = 0;
    
    if (centeringMode) {
      // 自動回中模式 - 使用實際角度計算回中力
      force = calculateCenteringForce(actualAngle);
      
      // 當超過限制時，增強回中力
      if (overLimitPositive || overLimitNegative) {
        // 加強回中力度，確保從極限位置可以回中
        if (force > 0) force = 2;
        if (force < 0) force = -2;
      }
    } else {
      // 一般力回饋模式
      force = calculateForce(displayAngle);
    }
    
    // 直接應用力回饋到馬達
    applyMotorForce(force);
  } else {
    // 如果力回饋被禁用，確保馬達停止
    stopMotors();
  }
  
  // 定期發送角度數據
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    // 發送帶有前綴的角度值
    //Serial.print(anglePrefix);
    Serial.println(actualAngle, 2);  // 保留2位小數
    lastSendTime = currentTime;
  }
  
  delay(5);  // 減少延遲以提高響應性
}

// 獲取濾波後的角度
float getFilteredAngle() {
  // 讀取多次角度取平均值以減少噪聲
  float sum = 0;
  
  // 獲取樣本
  for (int i = 0; i < 5; i++) {
    sum += angleSensor.getRotationInDegrees();
    delay(2);
  }
  
  // 計算平均值
  return sum / 5;
}

// 計算自動回中力回饋 - 基於實際角度計算
int calculateCenteringForce(float angle) {
  // 如果偏差在中心死區範圍內，不產生力回饋
  if (abs(angle) < deadZone) {
    return 0;
  }
  
  // 計算回中力 - 根據角度直接判斷力度和方向
  if (angle > 0) {
    // 向右轉動，需要向左回中力
    if (abs(angle) > forceZone2) {
      return -2;  // 很遠離中心，強力回中(雙馬達)
    } else if (abs(angle) > forceZone1) {
      return -1;  // 中等距離，中等回中力(單馬達)
    } else {
      return -1;  // 較近，小回中力(單馬達)
    }
  } else {
    // 向左轉動，需要向右回中力
    if (abs(angle) > forceZone2) {
      return 2;   // 很遠離中心，強力回中(雙馬達)
    } else if (abs(angle) > forceZone1) {
      return 1;   // 中等距離，中等回中力(單馬達)
    } else {
      return 1;   // 較近，小回中力(單馬達)
    }
  }
}

// 計算普通力回饋強度 - 無PWM版
int calculateForce(float angle) {
  // 如果偏差在中心死區範圍內，不產生力回饋
  if (abs(angle) < deadZone) {
    return 0;
  }
  
  // 計算從死區到最大角度的標準化位置 (0.0 - 1.0)
  float normalizedPos = (abs(angle) - deadZone) / (maxAngle - deadZone);
  normalizedPos = constrain(normalizedPos, 0.0, 1.0);
  
  // 簡化力回饋計算，只分兩個區域
  if (normalizedPos > 0.5) {
    // 高強度 - 使用兩個馬達
    return (angle > 0) ? -2 : 2;  // 角度為正(右轉)時，需要反向力拉回
  } else {
    // 低強度 - 使用一個馬達
    return (angle > 0) ? -1 : 1;
  }
}

// 應用力回饋到馬達 - 自動回中無PWM版
void applyMotorForce(int force) {
  // 先停止所有馬達
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // 如果沒有力回饋，直接返回
  if (force == 0) {
    return;
  } 
  
  // 根據力量等級和方向控制馬達
  if (force > 0) {
    // 正向力回饋 (向右拉)
    if (force >= 2) {
      // 高強度 - 兩個馬達同時工作
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      // 低強度 - 只用A馬達
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
  } else {
    // 反向力回饋 (向左拉)
    if (force <= -2) {
      // 高強度 - 兩個馬達同時工作
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      // 低強度 - 只用A馬達
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
  }
}
