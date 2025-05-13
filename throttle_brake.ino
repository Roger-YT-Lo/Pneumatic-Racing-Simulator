// 轉換為0~1供Unity使用，數值代表力度

// 校準類比值範圍
const int THROTTLE_MAX = 990;  // 油門放開時的值
const int THROTTLE_MIN = 36;   // 油門踩到底時的值
const int BRAKE_MAX = 990;     // 剎車放開時的值
const int BRAKE_MIN = 46;      // 剎車踩到底時的值


const int throttlePin = A0;     // 油門可變電阻接 A0
const int brakePin = A1;        // 剎車可變電阻接 A1

// 原始讀數參數
int throttleValue = 0;          // 讀取的油門 ADC 數值 (0-1023)
int brakeValue = 0;             // 讀取的剎車 ADC 數值 (0-1023)

// 轉換後的值
float throttleNormalized = 0.0; // 轉換後的油門值 (0到1)
float brakeNormalized = 0.0;    // 轉換後的剎車值 (0到1)

// 平滑處理參數
const int SAMPLES = 5;          // 採樣數量
int throttleSamples[SAMPLES];   // 油門採樣陣列
int brakeSamples[SAMPLES];      // 煞車採樣陣列
int sampleIndex = 0;            // 當前採樣索引
float smoothFactor = 0.2;       // 平滑因子 (0-1)，較小的值 = 更平滑但反應更慢

// 上一次的值 (用於平滑處理)
float lastThrottleNormalized = 0.0;
float lastBrakeNormalized = 0.0;

// 死區設置 (防止小抖動)
const float DEADZONE = 0.01;    // 死區閾值

void setup() {
  Serial.begin(115200);         // 設定 Serial 通訊，連接 Unity
  pinMode(throttlePin, INPUT);
  pinMode(brakePin, INPUT);
  
  // 初始化採樣陣列
  for(int i = 0; i < SAMPLES; i++) {
    throttleSamples[i] = analogRead(throttlePin);
    brakeSamples[i] = analogRead(brakePin);
    delay(5);
  }
}

// 計算平均值函數
int getAverage(int samples[], int numSamples) {
  long sum = 0;
  for(int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  return sum / numSamples;
}

// 應用死區的函數
float applyDeadzone(float value, float threshold) {
  if(abs(value) < threshold) {
    return 0.0;
  } else {
    // 平滑地從死區過渡
    return value > 0 ? 
           (value - threshold) / (1.0 - threshold) : 
           (value + threshold) / (1.0 - threshold);
  }
}

void loop() {
  // 讀取油門和剎車值並存入採樣陣列
  throttleSamples[sampleIndex] = analogRead(throttlePin);
  brakeSamples[sampleIndex] = analogRead(brakePin);
  
  // 更新採樣索引
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  // 計算平均值
  int avgThrottle = getAverage(throttleSamples, SAMPLES);
  int avgBrake = getAverage(brakeSamples, SAMPLES);
  
  // 確保值在範圍內
  avgThrottle = constrain(avgThrottle, THROTTLE_MIN, THROTTLE_MAX);
  avgBrake = constrain(avgBrake, BRAKE_MIN, BRAKE_MAX);
  
  // 轉換為0到1的範圍（踩到底為1，放開為0）
  float rawThrottle = map(avgThrottle, THROTTLE_MAX, THROTTLE_MIN, 0, 100) / 100.0;
  float rawBrake = map(avgBrake, BRAKE_MAX, BRAKE_MIN, 0, 100) / 100.0;
  
  // 應用死區
  rawThrottle = applyDeadzone(rawThrottle, DEADZONE);
  rawBrake = applyDeadzone(rawBrake, DEADZONE);
  
  // 平滑處理 (低通濾波器)
  throttleNormalized = lastThrottleNormalized + smoothFactor * (rawThrottle - lastThrottleNormalized);
  brakeNormalized = lastBrakeNormalized + smoothFactor * (rawBrake - lastBrakeNormalized);
  
  // 更新上一次的值
  lastThrottleNormalized = throttleNormalized;
  lastBrakeNormalized = brakeNormalized;
  
  // 傳送格式化數據給Unity (保留兩位小數)
  Serial.print("T:");
  Serial.print(throttleNormalized, 2);
  Serial.print(",B:");
  Serial.println(brakeNormalized, 2);
  

  delay(10); // 更短的延遲提高響應性
}
