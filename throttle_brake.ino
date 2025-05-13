//讀取油門與煞車1023~0的上下限，供v2使用
const int throttlePin = A0; // 油門可變電阻接 A0
const int brakePin = A1;    // 剎車可變電阻接 A1

int throttleValue = 0;      // 讀取的油門 ADC 數值 (0-1023)
int brakeValue = 0;         // 讀取的剎車 ADC 數值 (0-1023)

float throttlePercent = 0.0; // 轉換後的油門百分比 (0-100%)
float brakePercent = 0.0;    // 轉換後的剎車百分比 (0-100%)

void setup() {
  Serial.begin(115200);     // 設定 Serial 通訊，連接 Unity
  pinMode(throttlePin, INPUT);
  pinMode(brakePin, INPUT);
}

void loop() {
  // 讀取油門值 (0-1023)
  throttleValue = analogRead(throttlePin);
  // 讀取剎車值 (0-1023)
  brakeValue = analogRead(brakePin);
  
  // 轉換成百分比 (0 - 100%)
  //throttlePercent = map(throttleValue, 0, 1023, 0, 100);
  //brakePercent = map(brakeValue, 0, 1023, 0, 100);
  
  // 傳送油門數據給 Unity
  Serial.print("T:");
  Serial.print(throttleValue);
  
  // 傳送剎車數據給 Unity
  Serial.print(",B:");
  Serial.println(brakeValue);
  
  delay(50); // 減少延遲，確保數據流暢
}
