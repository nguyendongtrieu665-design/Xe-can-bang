#include <Wire.h>          // Thư viện giao tiếp I2C, cần cho MPU6050
#include <I2Cdev.h>        // Lớp cơ sở cho thiết bị I2C
#include <MPU6050.h>       // Thư viện cho cảm biến Gia tốc và Con quay hồi chuyển
#include <PID_v1.h>        // Thư viện cho bộ điều khiển PID
#include <Fuzzy.h>         // Thư viện cho Logic Mờ (Fuzzy Logic)
#include <SoftwareSerial.h> // THÊM: Thư viện điều khiển Bluetooth

// --- 1. ĐỊNH NGHĨA PHẦN CỨNG ---
const int ENA = 5;         
const int IN1 = 8;         
const int IN2 = 9;         
const int ENB = 6;         
const int IN3 = 10;        
const int IN4 = 11;        

// THÊM: Khởi tạo Bluetooth (RX: 7, TX: 13)
SoftwareSerial myBT(7, 13); 

// --- 2. KHỞI TẠO ĐỐI TƯỢNG VÀ BIẾN TOÀN CỤC ---
MPU6050 mpu;               
Fuzzy* fuzzy = new Fuzzy();

float angle = 0;           
float gyroRate = 0;        
unsigned long prevTime = 0; 
float dt;                  

// --- 3. CÀI ĐẶT BỘ ĐIỀU KHIỂN PID ---
double setpointAngle, pidInput, pidOutput; 
double Kp = 25;            
double Ki = 150;           
double Kd = 0.8;           
PID balancePID(&pidInput, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);

// THÊM: Biến hỗ trợ điều khiển Bluetooth
double originalSetpoint = -0.9; 
int steering = 0;

// --- 4. KHAI BÁO TRƯỚC CÁC HÀM ---
void fuzzy_init();         
void moveMotor(int speed, int turn); // Cập nhật tham số turn

// -------------------------------------------------------------------
// --- HÀM SETUP ---
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);    
  myBT.begin(9600);        // THÊM: Khởi tạo Bluetooth tốc độ 9600
  Wire.begin();            

  mpu.initialize();        
  if (!mpu.testConnection()) { while (1); } 
  mpu.setXGyroOffset(220); 
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  pinMode(ENA, OUTPUT);    
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setpointAngle = originalSetpoint;    
  balancePID.SetSampleTime(10); 
  balancePID.SetOutputLimits(-255, 255); 
  balancePID.SetMode(AUTOMATIC); 

  fuzzy_init();            
  
  prevTime = millis();     
}

// -------------------------------------------------------------------
// --- HÀM LOOP ---
// -------------------------------------------------------------------
void loop() {
  // --- THÊM: Đọc lệnh Bluetooth ---
  if (myBT.available()) {
    char cmd = myBT.read();
    if (cmd == 'F') { setpointAngle = originalSetpoint + 2.5; steering = 0; }   // Tiến
    else if (cmd == 'B') { setpointAngle = originalSetpoint - 2.5; steering = 0; } // Lùi
    else if (cmd == 'L') { steering = -50; }                                      // Trái
    else if (cmd == 'R') { steering = 50; }                                       // Phải
    else if (cmd == 'S') { setpointAngle = originalSetpoint; steering = 0; }      // Dừng
  }

  // --- Đọc cảm biến và tính toán góc (Giữ nguyên) ---
  unsigned long currentTime = millis(); 
  dt = (currentTime - prevTime) / 1000.0; 
  prevTime = currentTime;               

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
  float accelAngle = atan2(ay, az) * 180 / PI; 
  gyroRate = gx / 131.0; 
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle; 

  // --- Chạy bộ điều khiển (Giữ nguyên) ---
  pidInput = angle;        
  balancePID.Compute();    

  fuzzy->setInput(1, angle);  
  fuzzy->setInput(2, gyroRate);
  fuzzy->fuzzify();        
  float fuzzyCorrection = fuzzy->defuzzify(1); 

  // --- Tổng hợp và ra lệnh cho động cơ ---
  float finalOutput = pidOutput + fuzzyCorrection; 
  
  // Ngắt an toàn nếu ngã
  if (abs(angle) > 45) {
    moveMotor(0, 0);
  } else {
    moveMotor(finalOutput, steering); // Thêm biến steering vào hàm điều khiển
  }

  // In giá trị (Giữ nguyên)
  Serial.print("Angle: "); Serial.print(angle);
  Serial.print(" | Final: "); Serial.println(finalOutput);
}

// -------------------------------------------------------------------
// --- HÀM ĐIỀU KHIỂN ĐỘNG CƠ (Cập nhật logic rẽ) ---
// -------------------------------------------------------------------
void moveMotor(int speed, int turn) {
  int speedLeft = speed + turn;
  int speedRight = speed - turn;

  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  // Động cơ trái
  if (speedLeft > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  analogWrite(ENA, abs(speedLeft));

  // Động cơ phải
  if (speedRight > 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); } 
  else { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  analogWrite(ENB, abs(speedRight));
}

// -------------------------------------------------------------------
// --- HÀM KHỞI TẠO LOGIC MỜ (Giữ nguyên 25 luật của bạn) ---
// -------------------------------------------------------------------
void fuzzy_init() {
  // Các tập mờ Angle, Rate, Output (Giữ nguyên như code gốc bạn gửi)
  FuzzySet* angleZE = new FuzzySet(-6, 0, 0, 6);
  FuzzySet* angleNS = new FuzzySet(-12, -7, -7, -2);
  FuzzySet* anglePS = new FuzzySet(2, 7, 7, 12);
  FuzzySet* angleNB = new FuzzySet(-30, -20, -12, -6);
  FuzzySet* anglePB = new FuzzySet(6, 12, 20, 30);

  FuzzySet* rateNB = new FuzzySet(-150, -100, -50, -10);
  FuzzySet* rateNS = new FuzzySet(-50, -20, -20, 0);
  FuzzySet* rateZE = new FuzzySet(-10, 0, 0, 10);
  FuzzySet* ratePS = new FuzzySet(0, 20, 20, 50);
  FuzzySet* ratePB = new FuzzySet(10, 50, 100, 150);

  FuzzySet* outputNB = new FuzzySet(-120, -80, -80, -40);
  FuzzySet* outputNS = new FuzzySet(-80, -40, -40, 0);
  FuzzySet* outputZE = new FuzzySet(-20, 0, 0, 20);
  FuzzySet* outputPS = new FuzzySet(0, 40, 40, 80);
  FuzzySet* outputPB = new FuzzySet(40, 80, 80, 120);

  FuzzyInput* fuzzyAngle = new FuzzyInput(1);
  fuzzyAngle->addFuzzySet(angleNB); fuzzyAngle->addFuzzySet(angleNS); fuzzyAngle->addFuzzySet(angleZE); fuzzyAngle->addFuzzySet(anglePS); fuzzyAngle->addFuzzySet(anglePB);
  fuzzy->addFuzzyInput(fuzzyAngle);
  
  FuzzyInput* fuzzyRate = new FuzzyInput(2);
  fuzzyRate->addFuzzySet(rateNB); fuzzyRate->addFuzzySet(rateNS); fuzzyRate->addFuzzySet(rateZE); fuzzyRate->addFuzzySet(ratePS); fuzzyRate->addFuzzySet(ratePB);
  fuzzy->addFuzzyInput(fuzzyRate);
  
  FuzzyOutput* fuzzyOutput = new FuzzyOutput(1);
  fuzzyOutput->addFuzzySet(outputNB); fuzzyOutput->addFuzzySet(outputNS); fuzzyOutput->addFuzzySet(outputZE); fuzzyOutput->addFuzzySet(outputPS); fuzzyOutput->addFuzzySet(outputPB);
  fuzzy->addFuzzyOutput(fuzzyOutput);

  // --- 25 LUẬT MỜ CỦA BẠN (GIỮ NGUYÊN HOÀN TOÀN) ---
  // Dòng 1: Khi Angle là PB
  FuzzyRuleAntecedent* antecedent1 = new FuzzyRuleAntecedent(); antecedent1->joinWithAND(anglePB, rateNB); FuzzyRuleConsequent* consequent1 = new FuzzyRuleConsequent(); consequent1->addOutput(outputZE); FuzzyRule* rule1 = new FuzzyRule(1, antecedent1, consequent1); fuzzy->addFuzzyRule(rule1);
  FuzzyRuleAntecedent* antecedent2 = new FuzzyRuleAntecedent(); antecedent2->joinWithAND(anglePB, rateNS); FuzzyRuleConsequent* consequent2 = new FuzzyRuleConsequent(); consequent2->addOutput(outputPS); FuzzyRule* rule2 = new FuzzyRule(2, antecedent2, consequent2); fuzzy->addFuzzyRule(rule2);
  FuzzyRuleAntecedent* antecedent3 = new FuzzyRuleAntecedent(); antecedent3->joinWithAND(anglePB, rateZE); FuzzyRuleConsequent* consequent3 = new FuzzyRuleConsequent(); consequent3->addOutput(outputPB); FuzzyRule* rule3 = new FuzzyRule(3, antecedent3, consequent3); fuzzy->addFuzzyRule(rule3);
  FuzzyRuleAntecedent* antecedent4 = new FuzzyRuleAntecedent(); antecedent4->joinWithAND(anglePB, ratePS); FuzzyRuleConsequent* consequent4 = new FuzzyRuleConsequent(); consequent4->addOutput(outputPB); FuzzyRule* rule4 = new FuzzyRule(4, antecedent4, consequent4); fuzzy->addFuzzyRule(rule4);
  FuzzyRuleAntecedent* antecedent5 = new FuzzyRuleAntecedent(); antecedent5->joinWithAND(anglePB, ratePB); FuzzyRuleConsequent* consequent5 = new FuzzyRuleConsequent(); consequent5->addOutput(outputPB); FuzzyRule* rule5 = new FuzzyRule(5, antecedent5, consequent5); fuzzy->addFuzzyRule(rule5);

  // Dòng 2: Khi Angle là PS
  FuzzyRuleAntecedent* antecedent6 = new FuzzyRuleAntecedent(); antecedent6->joinWithAND(anglePS, rateNB); FuzzyRuleConsequent* consequent6 = new FuzzyRuleConsequent(); consequent6->addOutput(outputNS); FuzzyRule* rule6 = new FuzzyRule(6, antecedent6, consequent6); fuzzy->addFuzzyRule(rule6);
  FuzzyRuleAntecedent* antecedent7 = new FuzzyRuleAntecedent(); antecedent7->joinWithAND(anglePS, rateNS); FuzzyRuleConsequent* consequent7 = new FuzzyRuleConsequent(); consequent7->addOutput(outputZE); FuzzyRule* rule7 = new FuzzyRule(7, antecedent7, consequent7); fuzzy->addFuzzyRule(rule7);
  FuzzyRuleAntecedent* antecedent8 = new FuzzyRuleAntecedent(); antecedent8->joinWithAND(anglePS, rateZE); FuzzyRuleConsequent* consequent8 = new FuzzyRuleConsequent(); consequent8->addOutput(outputPS); FuzzyRule* rule8 = new FuzzyRule(8, antecedent8, consequent8); fuzzy->addFuzzyRule(rule8);
  FuzzyRuleAntecedent* antecedent9 = new FuzzyRuleAntecedent(); antecedent9->joinWithAND(anglePS, ratePS); FuzzyRuleConsequent* consequent9 = new FuzzyRuleConsequent(); consequent9->addOutput(outputPB); FuzzyRule* rule9 = new FuzzyRule(9, antecedent9, consequent9); fuzzy->addFuzzyRule(rule9);
  FuzzyRuleAntecedent* antecedent10 = new FuzzyRuleAntecedent(); antecedent10->joinWithAND(anglePS, ratePB); FuzzyRuleConsequent* consequent10 = new FuzzyRuleConsequent(); consequent10->addOutput(outputPB); FuzzyRule* rule10 = new FuzzyRule(10, antecedent10, consequent10); fuzzy->addFuzzyRule(rule10);

  // Dòng 3: Khi Angle là ZE
  FuzzyRuleAntecedent* antecedent11 = new FuzzyRuleAntecedent(); antecedent11->joinWithAND(angleZE, rateNB); FuzzyRuleConsequent* consequent11 = new FuzzyRuleConsequent(); consequent11->addOutput(outputNB); FuzzyRule* rule11 = new FuzzyRule(11, antecedent11, consequent11); fuzzy->addFuzzyRule(rule11);
  FuzzyRuleAntecedent* antecedent12 = new FuzzyRuleAntecedent(); antecedent12->joinWithAND(angleZE, rateNS); FuzzyRuleConsequent* consequent12 = new FuzzyRuleConsequent(); consequent12->addOutput(outputNS); FuzzyRule* rule12 = new FuzzyRule(12, antecedent12, consequent12); fuzzy->addFuzzyRule(rule12);
  FuzzyRuleAntecedent* antecedent13 = new FuzzyRuleAntecedent(); antecedent13->joinWithAND(angleZE, rateZE); FuzzyRuleConsequent* consequent13 = new FuzzyRuleConsequent(); consequent13->addOutput(outputZE); FuzzyRule* rule13 = new FuzzyRule(13, antecedent13, consequent13); fuzzy->addFuzzyRule(rule13);
  FuzzyRuleAntecedent* antecedent14 = new FuzzyRuleAntecedent(); antecedent14->joinWithAND(angleZE, ratePS); FuzzyRuleConsequent* consequent14 = new FuzzyRuleConsequent(); consequent14->addOutput(outputPS); FuzzyRule* rule14 = new FuzzyRule(14, antecedent14, consequent14); fuzzy->addFuzzyRule(rule14);
  FuzzyRuleAntecedent* antecedent15 = new FuzzyRuleAntecedent(); antecedent15->joinWithAND(angleZE, ratePB); FuzzyRuleConsequent* consequent15 = new FuzzyRuleConsequent(); consequent15->addOutput(outputPB); FuzzyRule* rule15 = new FuzzyRule(15, antecedent15, consequent15); fuzzy->addFuzzyRule(rule15);

  // Dòng 4: Khi Angle là NS
  FuzzyRuleAntecedent* antecedent16 = new FuzzyRuleAntecedent(); antecedent16->joinWithAND(angleNS, rateNB); FuzzyRuleConsequent* consequent16 = new FuzzyRuleConsequent(); consequent16->addOutput(outputNB); FuzzyRule* rule16 = new FuzzyRule(16, antecedent16, consequent16); fuzzy->addFuzzyRule(rule16);
  FuzzyRuleAntecedent* antecedent17 = new FuzzyRuleAntecedent(); antecedent17->joinWithAND(angleNS, rateNS); FuzzyRuleConsequent* consequent17 = new FuzzyRuleConsequent(); consequent17->addOutput(outputNB); FuzzyRule* rule17 = new FuzzyRule(17, antecedent17, consequent17); fuzzy->addFuzzyRule(rule17);
  FuzzyRuleAntecedent* antecedent18 = new FuzzyRuleAntecedent(); antecedent18->joinWithAND(angleNS, rateZE); FuzzyRuleConsequent* consequent18 = new FuzzyRuleConsequent(); consequent18->addOutput(outputNS); FuzzyRule* rule18 = new FuzzyRule(18, antecedent18, consequent18); fuzzy->addFuzzyRule(rule18);
  FuzzyRuleAntecedent* antecedent19 = new FuzzyRuleAntecedent(); antecedent19->joinWithAND(angleNS, ratePS); FuzzyRuleConsequent* consequent19 = new FuzzyRuleConsequent(); consequent19->addOutput(outputZE); FuzzyRule* rule19 = new FuzzyRule(19, antecedent19, consequent19); fuzzy->addFuzzyRule(rule19);
  FuzzyRuleAntecedent* antecedent20 = new FuzzyRuleAntecedent(); antecedent20->joinWithAND(angleNS, ratePB); FuzzyRuleConsequent* consequent20 = new FuzzyRuleConsequent(); consequent20->addOutput(outputPS); FuzzyRule* rule20 = new FuzzyRule(20, antecedent20, consequent20); fuzzy->addFuzzyRule(rule20);

  // Dòng 5: Khi Angle là NB
  FuzzyRuleAntecedent* antecedent21 = new FuzzyRuleAntecedent(); antecedent21->joinWithAND(angleNB, rateNB); FuzzyRuleConsequent* consequent21 = new FuzzyRuleConsequent(); consequent21->addOutput(outputNB); FuzzyRule* rule21 = new FuzzyRule(21, antecedent21, consequent21); fuzzy->addFuzzyRule(rule21);
  FuzzyRuleAntecedent* antecedent22 = new FuzzyRuleAntecedent(); antecedent22->joinWithAND(angleNB, rateNS); FuzzyRuleConsequent* consequent22 = new FuzzyRuleConsequent(); consequent22->addOutput(outputNB); FuzzyRule* rule22 = new FuzzyRule(22, antecedent22, consequent22); fuzzy->addFuzzyRule(rule22);
  FuzzyRuleAntecedent* antecedent23 = new FuzzyRuleAntecedent(); antecedent23->joinWithAND(angleNB, rateZE); FuzzyRuleConsequent* consequent23 = new FuzzyRuleConsequent(); consequent23->addOutput(outputNB); FuzzyRule* rule23 = new FuzzyRule(23, antecedent23, consequent23); fuzzy->addFuzzyRule(rule23);
  FuzzyRuleAntecedent* antecedent24 = new FuzzyRuleAntecedent(); antecedent24->joinWithAND(angleNB, ratePS); FuzzyRuleConsequent* consequent24 = new FuzzyRuleConsequent(); consequent24->addOutput(outputNS); FuzzyRule* rule24 = new FuzzyRule(24, antecedent24, consequent24); fuzzy->addFuzzyRule(rule24);
  FuzzyRuleAntecedent* antecedent25 = new FuzzyRuleAntecedent(); antecedent25->joinWithAND(angleNB, ratePB); FuzzyRuleConsequent* consequent25 = new FuzzyRuleConsequent(); consequent25->addOutput(outputZE); FuzzyRule* rule25 = new FuzzyRule(25, antecedent25, consequent25); fuzzy->addFuzzyRule(rule25);
}