//AS5048A 使用SPI通訊
//具有開機自檢系統﹐自動回歸至0
//2025.3.19
//使用參數校正自檢偏差問題
//加入按鈕手動校正

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
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
float deadZone = 30;         // 中心死區(度)，避免中心點抖動35
int minPWM = 15;             // 最小有效PWM值15
float initialPosition = 0.0; // 儲存初始位置
float forceGain = 0.7;       // 力回饋增益係數
float maxAngle = 440.0;      // 最大角度範圍

// 多轉追蹤變數
int fullRotations = 0;        // 完整旋轉圈數
float lastAngle = 0.0;        // 上次讀取的角度

// 平滑控制變數
float currentForce = 0.0;     // 當前馬達力值
float targetForce = 0.0;      // 目標馬達力值
float smoothFactor = 0.1;     // 力值平滑係數

// 自檢系統參數
float leftLimit = 0.0;        // 左側極限位置
float rightLimit = 0.0;       // 右側極限位置
float calculatedCenter = 0.0; // 計算出的中心位置
const int CALIB_MOTOR_POWER = 70; // 校準時的馬達功率
const int STALL_THRESHOLD = 2;    // 移動停止閾值（度）
const int STALL_COUNT = 10;       // 確認停止的次數
const float CENTER_CORRECTION = -15.0; // 中心點校正值（減去(向左)15度）

//手動校正參數
const int BUTTON_PIN = 2;     // 按鈕接在D2引腳（可根據需要調整）
bool buttonWasPressed = false; // 追蹤按鈕之前的狀態
unsigned long buttonPressTime = 0; // 記錄按鈕按下的時間
const int DEBOUNCE_DELAY = 50;    // 防抖延遲（毫秒）


// LCD顯示器設定 - 加到全局變數區
LiquidCrystal_I2C lcd(0x27, 16, 2); // 設置LCD地址為0x27，16列2行顯示器
unsigned long lcdUpdateTime = 0;     // 上次更新LCD的時間
const int LCD_UPDATE_INTERVAL = 50; // LCD更新間隔(毫秒)，避免過於頻繁刷新

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

  //初始化手動校正按鈕
  pinMode(BUTTON_PIN, INPUT_PULLUP); // 使用內部上拉電阻，記得加到setup中

  // 初始化I2C LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // 顯示初始化中訊息
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
  
  Serial.println("方向盤力回饋系統初始化");
  Serial.println("- 開始自檢和校準程序");
  
  // 等待感測器穩定
  delay(1000);
  
  // 執行自檢程序
  //performSelfTest();
  
  // 初始化上次角度
  //lastAngle = angleSensor.getRotationInDegrees();
  
  Serial.println("系統準備就緒");
  Serial.println("------------------------");
}

void loop() {

  
  manualCalibration();
 
  // 如果按鈕按下，跳過正常的力回饋計算
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(20);  // 短暫延遲
    return;     // 跳過剩餘的loop
  }
  // 讀取當前角度
  float currentRawAngle = angleSensor.getRotationInDegrees();  
  // 處理多圈旋轉（超過360度範圍）

  // 檢測圈數變化
  if (lastAngle > 270 && currentRawAngle < 90) {
    // 順時針穿過零點
    fullRotations++;
  } else if (lastAngle < 90 && currentRawAngle > 270) {
    // 逆時針穿過零點
    fullRotations--;
  }
  
  // 計算絕對角度（包含多圈）
  float currentAngle = currentRawAngle + (fullRotations * 360);
  lastAngle = currentRawAngle;
  updateLCD(currentAngle);
  // 限制在+-440度範圍
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
  
  // 應用平滑後的力回饋
  applyMotorForce((int)currentForce);
  
  // 顯示信息
  Serial.print("角度: ");
  Serial.print(currentAngle, 2);
  Serial.print(" | 偏差: ");
  Serial.print(angleDiff, 2);
  Serial.print(" | 力回饋: ");
  Serial.println(currentForce);
  
  // 減少診斷信息的顯示頻率
  static int counter = 0;
  if (counter++ % 20 == 0) {
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


// 手動校正功能函式 - 在方向盤可自由轉動時設定中心點
void manualCalibration() {
  // 讀取按鈕狀態（使用INPUT_PULLUP，按下為LOW）
  bool buttonIsPressed = (digitalRead(BUTTON_PIN) == LOW);
  
  // 按鈕狀態變化時防抖
  if (buttonIsPressed != buttonWasPressed) {
    if (millis() - buttonPressTime > DEBOUNCE_DELAY) {
      buttonPressTime = millis(); // 更新時間戳
      
      // 處理按鈕狀態變化
      if (buttonIsPressed) {
        // 按鈕剛被按下
        Serial.println("按鈕按下 - 進入自由轉動模式");
        displayManualCalibrationMode(true);
        // 停止馬達，不施加力回饋
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
      } else {
        // 按鈕剛被放開，進行校正
        Serial.println("按鈕放開 - 設定當前位置為中心點");
        displayManualCalibrationMode(false);
        // 讀取當前位置
        float currentPosition = getFilteredAngle();
        Serial.print("當前位置: ");
        Serial.println(currentPosition);
        
        // 設定零點位置，使當前位置讀數變為0度
        float offset = 0.0 - currentPosition;
        uint16_t currentZero = angleSensor.getZeroPosition();
        int16_t zeroShift = (offset * 16384.0 / 360.0); // 轉換角度到感測器原始值
        
        // 設定新的零點
        angleSensor.setZeroPosition(currentZero - zeroShift);
        
        // 重置圈數計數器和角度記錄
        fullRotations = 0;
        lastAngle = angleSensor.getRotationInDegrees();
        
        // 驗證設定
        delay(100);
        float newAngle = angleSensor.getRotationInDegrees();
        Serial.print("校準完成，當前角度: ");
        Serial.println(newAngle);
      }
      
      // 更新按鈕狀態
      buttonWasPressed = buttonIsPressed;
    }
  }
}
// 執行自檢和自動校準
void performSelfTest() {
  Serial.println("開始自檢系統...");
  
  // 先讀取當前位置
  float currentPosition = getFilteredAngle();
  Serial.print("當前位置: ");
  Serial.println(currentPosition);
  
  // 步驟1: 向左轉動直到碰到極限
  Serial.println("1. 向左轉動至極限...");
  leftLimit = findLeftLimit();
  Serial.print("左極限位置: ");
  Serial.println(leftLimit);
  
  // 短暫休息
  delay(500);
  
  // 步驟2: 向右轉動直到碰到極限
  Serial.println("2. 向右轉動至極限...");
  rightLimit = findRightLimit();
  Serial.print("右極限位置: ");
  Serial.println(rightLimit);
  
  // 步驟3: 計算中心位置
  calculatedCenter = ((leftLimit + rightLimit) / 2);
  Serial.print("計算得到的中心位置: ");
  Serial.println(calculatedCenter);
  
  // 步驟4: 移動到中心位置
  Serial.println("3. 移動到中心位置...");
  moveToCenter();
 // calculatedCenter +=CENTER_CORRECTION;
  // 步驟5: 設定零點位置
  Serial.println("4. 設定中心點為零點...");
  float currentRawAngle = angleSensor.getRotationInDegrees();
  
  // 設定零點位置，使當前位置讀數變為0度
  float offset = 0.0 - currentRawAngle+CENTER_CORRECTION;
  uint16_t currentZero = angleSensor.getZeroPosition();
  int16_t zeroShift = (offset * 16384.0 / 360.0); // 轉換角度到感測器原始值
  
  // 設定新的零點
  angleSensor.setZeroPosition(currentZero - zeroShift);
  
  // 驗證設定
  delay(200);
  float newAngle = angleSensor.getRotationInDegrees();
  Serial.print("校準完成，當前角度: ");
  Serial.println(newAngle);
  
  // 初始化跟踪變數
  fullRotations = 0;
  lastAngle = newAngle;
  
  // 顯示檢測到的方向盤範圍
  float totalRange = abs(rightLimit - leftLimit);
  Serial.print("方向盤總轉動範圍: ");
  Serial.print(totalRange);
  Serial.println(" 度");
  
  Serial.println("自檢和校準完成！");

}

// 尋找左側極限
float findLeftLimit() {
  float lastPosition = getFilteredAngle();
  int stallCounter = 0;
  
  // 啟動向左轉動
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, CALIB_MOTOR_POWER);
  
  // 持續轉動直到檢測到停止
  while (stallCounter < STALL_COUNT) {
    delay(100);
    float currentPosition = getFilteredAngle();
    float movement = abs(currentPosition - lastPosition);
    
    Serial.print("左移: 位置=");
    Serial.print(currentPosition);
    Serial.print(", 移動量=");
    Serial.println(movement);
    
    if (movement < STALL_THRESHOLD) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }
    
    lastPosition = currentPosition;
  }
  
  // 停止馬達
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  // 返回檢測到的左極限位置
  return getFilteredAngle();
}

// 尋找右側極限
float findRightLimit() {
  float lastPosition = getFilteredAngle();
  int stallCounter = 0;
  
  // 啟動向右轉動
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, CALIB_MOTOR_POWER);
  
  // 持續轉動直到檢測到停止
  while (stallCounter < STALL_COUNT) {
    delay(100);
    float currentPosition = getFilteredAngle();
    float movement = abs(currentPosition - lastPosition);
    
    Serial.print("右移: 位置=");
    Serial.print(currentPosition);
    Serial.print(", 移動量=");
    Serial.println(movement);
    
    if (movement < STALL_THRESHOLD) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }
    
    lastPosition = currentPosition;
  }
  
  // 停止馬達
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  // 返回檢測到的右極限位置
  return getFilteredAngle();
}

// 移動到中心位置
void moveToCenter() {
  float currentPosition = getFilteredAngle();
  float targetPosition = calculatedCenter;
  
  Serial.print("從 ");
  Serial.print(currentPosition);
  Serial.print(" 移動到中心點 ");
  Serial.println(targetPosition);
  
  // 判斷移動方向
  bool moveRight = (targetPosition > currentPosition);
  
  // 設定方向
  if (moveRight) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  // 啟動馬達
  analogWrite(ENA, CALIB_MOTOR_POWER);
  
  // 監控位置直到接近目標
  while (abs(getFilteredAngle() - targetPosition) > 5.0) {
    float currentPos = getFilteredAngle();
    Serial.print("當前: ");
    Serial.print(currentPos);
    Serial.print(", 目標: ");
    Serial.println(targetPosition);
    
    // 接近目標時降低速度
    if (abs(currentPos - targetPosition) < 30) {
      analogWrite(ENA, CALIB_MOTOR_POWER / 2);
    }
    
    // 檢查是否已經過頭
    if ((moveRight && currentPos > targetPosition) || 
        (!moveRight && currentPos < targetPosition)) {
      break;
    }
    
    delay(100);
  }
  
  // 停止馬達
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  Serial.println("已到達中心位置");
}

// 獲取濾波後的角度
float getFilteredAngle() {
  // 讀取多次角度取平均值以減少噪聲
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += angleSensor.getRotationInDegrees();
    delay(10);
  }
  return sum / 5;
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

// 應用力回饋到馬達
void applyMotorForce(int force) {
  if (abs(force) < 5) {
    // 停止馬達
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } 
  else if (force > 0) {
    // 正方向轉動 - 將方向盤拉向中心點
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(force));
  } 
  else {
    // 負方向轉動 - 將方向盤拉向中心點
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(force));
  }
}




// 更新LCD顯示 - 可以在loop()中調用
void updateLCD(float currentAngle) {
  // 限制更新頻率以避免LCD閃爍
  unsigned long currentTime = millis();
  if (currentTime - lcdUpdateTime < LCD_UPDATE_INTERVAL) {
    return;
  }
  lcdUpdateTime = currentTime;
  
  // 更新LCD顯示
  lcd.clear();
  
  // 第一行顯示角度
  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  
  // 格式化角度顯示（保留1位小數）
  char angleBuffer[10];
  dtostrf(currentAngle, 6, 1, angleBuffer);
  lcd.print(angleBuffer);
  lcd.print((char)223); // 顯示度數符號 °
  
  // 第二行顯示方向盤位置的圖形表示
  lcd.setCursor(0, 1);
  lcd.print("Position: ");
  
  // 顯示圖形指示器
  int position = map(constrain(currentAngle, -90, 90), -90, 90, 0, 10);
  lcd.setCursor(10, 1);
  
  for (int i = 0; i < 6; i++) {
    if (i == position/2) {
      lcd.print("|"); // 方向盤位置指示
    } else {
      lcd.print("-");
    }
  }
}

// 顯示手動校正模式
void displayManualCalibrationMode(bool isActive) {
  if (isActive) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Manual Calibrate");
    lcd.setCursor(0, 1);
    lcd.print("Release to set");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("New Center Set!");
    lcd.setCursor(0, 1);
    lcd.print("Angle: 0.0");
    delay(1500); // 顯示確認訊息1.5秒
  }
}
