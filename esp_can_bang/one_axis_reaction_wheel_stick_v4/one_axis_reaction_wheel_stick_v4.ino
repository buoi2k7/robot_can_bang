/*
 * ═══════════════════════════════════════════════════════════════════════════════
 * REACTION WHEEL STICK V4 - STABLE EDITION
 * ✨ Fix drift, divergence, integral windup
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ===== WIFI CONFIG =====
const char *ssid = "link";
const char *password = "buoinha132/";
const char *udpAddress = "192.168.1.19";
const int udpPort = 4210;

IPAddress local_IP(192, 168, 1, 7);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

WiFiUDP udp;

// ===== PINOUT =====
#define BRAKE_PIN 26
#define PWM_PIN 25
#define DIR_PIN 27
#define BUZZER_PIN 14

// MPU6050
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define CONFIG_REG 0x1A

// PWM Config
const int PWM_FREQ = 20000;
const int PWM_RES = 8; // 8-bit như V2 (đơn giản, ổn định)

// ═══════════════════════════════════════════════════════════════════════════════
// PID PARAMETERS - TUNE CHỈ 1 LẦN, KHÔNG AUTO-TUNE
// ═══════════════════════════════════════════════════════════════════════════════

// Giá trị khởi đầu (tune thủ công cho đến khi cứng)
float Kp = 55.0; // Giống V2 ban đầu
float Kd = 22.0; // Giống V2 ban đầu
float Ki = 0.0;  // Tắt Ki ban đầu, bật khi cần
float Kw = 1.2;  // Motor feedback

// Giới hạn cứng - KHÔNG ĐỔI trong runtime
const float KP_MIN = 30.0, KP_MAX = 100.0;
const float KD_MIN = 10.0, KD_MAX = 40.0;
const float KI_MAX = 2.0;
const float INTEGRAL_MAX = 80.0; // Anti-windup

// ═══════════════════════════════════════════════════════════════════════════════
// TIMING
// ═══════════════════════════════════════════════════════════════════════════════
const float LOOP_TIME_MS = 10.0; // 100Hz như V2
const float LOOP_TIME_S = 0.01;
unsigned long currentT = 0, previousT = 0;
unsigned long currentT_py = 0, previousT_py = 0;
const int LOOP_TIME_PY = 50;

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR VARIABLES
// ═══════════════════════════════════════════════════════════════════════════════
int16_t AcX, AcY, AcZ, GyZ;
int16_t GyZ_offset = 0;
int32_t GyZ_offset_sum = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// COMPLEMENTARY FILTER - Đơn giản, không diverge như Kalman
// ═══════════════════════════════════════════════════════════════════════════════
float robot_angle = 0; // Góc cuối cùng
float robot_angle_raw = 0;
float gyroZ_filtered = 0;
const float GYRO_ALPHA = 0.996;      // Tin tưởng gyro 99.6%
const float GYRO_FILTER_ALPHA = 0.4; // EMA cho gyro rate

// ═══════════════════════════════════════════════════════════════════════════════
// BALANCE OFFSET - Fix drift bằng giới hạn cứng
// ═══════════════════════════════════════════════════════════════════════════════
float balance_angle_offset = 0.0;
const float OFFSET_MAX = 3.0;     // Giới hạn offset ±3 độ
const float OFFSET_ALPHA = 0.005; // Update rất chậm
unsigned long balance_stable_start = 0;
bool assist_mode = false;

// Thresholds
const float VERTICAL_ON_THRESHOLD = 0.5;
const float VERTICAL_OFF_THRESHOLD = 9.0;
const float BALANCE_STABLE_ANGLE = 1.5;
const float BALANCE_STABLE_GYRO = 3.0;
const float BALANCE_STABLE_PWM = 20.0;
const unsigned long BALANCE_STABLE_TIME = 2000; // 2s mới update offset
const float FALL_CUTOFF_ANGLE = 12.0;

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR VARIABLES - Fix rung + nóng
// ═══════════════════════════════════════════════════════════════════════════════
int pwm_output = 0;
int pwm_output_prev = 0;
float pwm_smooth = 0;                 // PWM đã làm mượt
float motor_speed = 0;                // Tích phân PWM
const float MOTOR_SPEED_DECAY = 0.95; // Decay hàng loop
const float MOTOR_SPEED_MAX = 5000;

// ⭐ ANTI-JITTER: Deadband + Smoothing
const float DEADBAND_ANGLE = 0.3;   // Vùng chết: không phản ứng nếu góc < 0.3°
const float PWM_SMOOTH_ALPHA = 0.7; // Làm mượt PWM (0.5-0.9, cao = mượt hơn)
const int PWM_MIN_THRESHOLD = 15;   // PWM < 15 thì tắt hẳn (tránh rung nhỏ)

bool vertical = false;

// PID variables
float integral = 0;
float previous_error = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);

  setupWiFi();

  // PWM 8-bit như V2
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  initMPU6050();
  calibrateGyro();

  // Khởi tạo góc ban đầu
  readSensors();
  float acc_angle = atan2((float)AcY, -(float)AcX) * 57.2958f;
  robot_angle = acc_angle;

  beepStartup();

  Serial.println("✅ V4 STABLE EDITION READY!");
  Serial.println("═══════════════════════════════════════");
  Serial.printf("🔧 Kp=%.1f, Kd=%.1f, Ki=%.1f, Kw=%.1f\n", Kp, Kd, Ki, Kw);
  Serial.println("⚠️  TIPS: Nếu drift → tăng BALANCE_STABLE_TIME");
  Serial.println("⚠️  TIPS: Nếu rung → giảm Kp hoặc tăng Kd");
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  currentT = millis();

  if (currentT - previousT >= LOOP_TIME_MS) {
    float dt = (currentT - previousT) / 1000.0;
    previousT = currentT;

    // === READ SENSORS ===
    readSensors();

    // === COMPLEMENTARY FILTER (Không diverge như Kalman) ===
    float acc_angle = atan2((float)AcY, -(float)AcX) * 57.2958f;
    float gyro_rate = GyZ / 131.0f; // ±250°/s

    // EMA filter cho gyro
    gyroZ_filtered = GYRO_FILTER_ALPHA * gyro_rate +
                     (1 - GYRO_FILTER_ALPHA) * gyroZ_filtered;

    // Complementary filter
    robot_angle_raw = GYRO_ALPHA * (robot_angle_raw + gyro_rate * dt) +
                      (1 - GYRO_ALPHA) * acc_angle;

    // === UPDATE OFFSET (với giới hạn cứng) ===
    updateBalanceReference();
    robot_angle = robot_angle_raw - balance_angle_offset;

    // === UPDATE VERTICAL STATE ===
    updateVerticalState();

    // === AUTO-TUNE PID (3 chế độ: nhẹ/vừa/nặng) ===
    updateAssistAutoTune();

    // === CONTROL ===
    if (vertical) {
      digitalWrite(BRAKE_PIN, HIGH);

      // Kiểm tra ngã gấp
      if (abs(robot_angle) > FALL_CUTOFF_ANGLE) {
        emergencyStop();
      } else {
        // ⭐ DEADBAND: Nếu góc rất nhỏ → không làm gì (tránh rung)
        float error = robot_angle;

        if (abs(error) < DEADBAND_ANGLE && abs(gyroZ_filtered) < 1.0) {
          // Trong vùng chết và không quay → giữ yên
          pwm_output = 0;
          integral = 0; // Reset integral tránh tích tụ
        } else {
          // PID Controller bình thường

          // Proportional
          float P_term = Kp * error;

          // Derivative (dùng gyro thay vì d(error)/dt - tránh derivative kick)
          float D_term = Kd * gyroZ_filtered;

          // Integral với anti-windup
          integral += Ki * error * dt;
          integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

          // Motor feedback (wheel speed)
          float W_term = Kw * (-motor_speed / 100.0);

          // Total output
          float output = P_term + integral + D_term + W_term;
          int pwm_raw = (int)constrain(output, -255, 255);

          // ⭐ PWM SMOOTHING: Làm mượt để tránh giật
          pwm_smooth = PWM_SMOOTH_ALPHA * pwm_smooth +
                       (1.0 - PWM_SMOOTH_ALPHA) * pwm_raw;
          pwm_output = (int)pwm_smooth;

          // ⭐ MINIMUM THRESHOLD: PWM quá nhỏ thì tắt hẳn
          if (abs(pwm_output) < PWM_MIN_THRESHOLD) {
            pwm_output = 0;
          }
        }

        // Motor speed integration với DECAY
        if (abs(robot_angle) < 1.0 && abs(pwm_output) < 10) {
          motor_speed = 0;
        } else {
          motor_speed = MOTOR_SPEED_DECAY * motor_speed + pwm_output;
          motor_speed =
              constrain(motor_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
        }

        Motor_control(pwm_output);
      }
    } else {
      // Không ở vị trí cân bằng
      digitalWrite(BRAKE_PIN, LOW);
      Motor_control(0);

      // RESET TẤT CẢ khi ngã (quan trọng!)
      pwm_output = 0;
      motor_speed = 0;
      integral = 0;
      previous_error = 0;
    }

    // === TELEMETRY ===
    currentT_py = millis();
    if (currentT_py - previousT_py >= LOOP_TIME_PY) {
      previousT_py = currentT_py;
      sendTelemetry();
      receiveUDP();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void initMPU6050() {
  writeRegister(MPU6050_ADDR, PWR_MGMT_1, 0x00);
  delay(100);

  // Gyro ±250°/s (131 LSB/°/s) - nhạy hơn ±500°/s
  writeRegister(MPU6050_ADDR, GYRO_CONFIG, 0x00);

  // Accel ±2g
  writeRegister(MPU6050_ADDR, ACCEL_CONFIG, 0x00);

  // DLPF ~20Hz
  writeRegister(MPU6050_ADDR, CONFIG_REG, 0x04);

  Serial.println("✅ MPU6050 Initialized (±250°/s)");
}

void calibrateGyro() {
  Serial.println("🔄 Calibrating gyro... (keep still!)");
  GyZ_offset_sum = 0;

  for (int i = 0; i < 500; i++) {
    readSensors();
    GyZ_offset_sum += GyZ;
    delay(3);
  }

  GyZ_offset = GyZ_offset_sum / 500;
  Serial.printf("✅ GyZ_offset = %d\n", GyZ_offset);
}

void readSensors() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);

  GyZ = (Wire.read() << 8) | Wire.read();
  GyZ -= GyZ_offset;
}

// ═══════════════════════════════════════════════════════════════════════════════
// BALANCE OFFSET - Fix drift bằng giới hạn cứng và update chậm
// ═══════════════════════════════════════════════════════════════════════════════

void updateBalanceReference() {
  bool stable = abs(robot_angle_raw) < BALANCE_STABLE_ANGLE &&
                abs(gyroZ_filtered) < BALANCE_STABLE_GYRO &&
                abs(pwm_output) < BALANCE_STABLE_PWM;

  if (stable) {
    if (balance_stable_start == 0) {
      balance_stable_start = millis();
    }

    // Chỉ update sau 2s ổn định
    if (millis() - balance_stable_start >= BALANCE_STABLE_TIME) {
      float new_offset = (1.0 - OFFSET_ALPHA) * balance_angle_offset +
                         OFFSET_ALPHA * robot_angle_raw;

      // GIỚI HẠN CỨNG offset
      balance_angle_offset = constrain(new_offset, -OFFSET_MAX, OFFSET_MAX);

      assist_mode = true;
    }
  } else {
    balance_stable_start = 0;
    assist_mode = false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// VERTICAL STATE
// ═══════════════════════════════════════════════════════════════════════════════

void updateVerticalState() {
  // Hysteresis để tránh chuyển state liên tục
  if (abs(robot_angle) > VERTICAL_OFF_THRESHOLD) {
    vertical = false;
  }
  if (abs(robot_angle) < VERTICAL_ON_THRESHOLD) {
    vertical = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// AUTO-TUNE PID (3 CHẾ ĐỘ: NHẸ / VỪA / NẶNG)
// ═══════════════════════════════════════════════════════════════════════════════

void updateAssistAutoTune() {
  if (!assist_mode)
    return;

  float abs_angle = abs(robot_angle);
  float abs_rate = abs(gyroZ_filtered);

  // 🔴 NẶNG: Góc > 2.5° → Giảm Kp (tránh overshoot), Tăng Kd (phanh mạnh)
  if (abs_angle > 2.5f) {
    Kp = max(KP_MIN, Kp - 0.2f);
    Kd = min(KD_MAX, Kd + 0.1f);
  }
  // 🟢 NHẸ: Góc < 0.8° và rate < 2° → Tăng Kp (giữ cứng hơn)
  else if (abs_angle < 0.8f && abs_rate < 2.0f) {
    Kp = min(KP_MAX, Kp + 0.1f);
  }

  // 🟡 VỪA: Quay nhanh > 6°/s → Tăng Kd (damping tốt hơn)
  if (abs_rate > 6.0f) {
    Kd = min(KD_MAX, Kd + 0.05f);
  }

  // Đảm bảo giá trị trong giới hạn
  Kp = constrain(Kp, KP_MIN, KP_MAX);
  Kd = constrain(Kd, KD_MIN, KD_MAX);
  Kw = constrain(Kw, 0.5f, 4.0f);
}

void emergencyStop() {
  Motor_control(0);
  motor_speed = 0;
  integral = 0;
  previous_error = 0;
  vertical = false;
  digitalWrite(BRAKE_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void Motor_control(int pwm) {
  if (pwm == 0) {
    ledcWrite(PWM_PIN, 0);
    return;
  }

  if (pwm < 0) {
    digitalWrite(DIR_PIN, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }

  ledcWrite(PWM_PIN, pwm);
}

// ═══════════════════════════════════════════════════════════════════════════════
// WIFI & UDP
// ═══════════════════════════════════════════════════════════════════════════════

void setupWiFi() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("⚠️ Static IP config failed!");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    udp.begin(udpPort);
  } else {
    Serial.println("\n❌ WiFi failed! Running offline.");
  }
}

void receiveUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[128];
    int len = udp.read(incoming, 127);
    if (len > 0)
      incoming[len] = '\0';

    String msg = String(incoming);
    parseCommand(msg);
  }
}

void parseCommand(String msg) {
  // Format: "Kp=55.0,Kd=22.0,Ki=0.0,Kw=1.2"
  if (msg.startsWith("Kp")) {
    float newKp = parseValue(msg, "Kp=");
    float newKd = parseValue(msg, "Kd=");
    float newKi = parseValue(msg, "Ki=");
    float newKw = parseValue(msg, "Kw=");

    // Giới hạn giá trị
    Kp = constrain(newKp, KP_MIN, KP_MAX);
    Kd = constrain(newKd, KD_MIN, KD_MAX);
    Ki = constrain(newKi, 0, KI_MAX);
    Kw = newKw;

    // Reset integral khi đổi tham số
    integral = 0;

    Serial.printf("📩 PID: Kp=%.1f, Kd=%.1f, Ki=%.2f, Kw=%.1f\n", Kp, Kd, Ki,
                  Kw);

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("ACK");
    udp.endPacket();
  }

  if (msg == "RESET") {
    integral = 0;
    motor_speed = 0;
    balance_angle_offset = 0;
    Serial.println("🔄 RESET all states");

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("RESET_OK");
    udp.endPacket();
  }

  if (msg == "GET") {
    // Gửi giá trị hiện tại về Python
    char buffer[100];
    sprintf(buffer, "Kp=%.1f,Kd=%.1f,Ki=%.2f,Kw=%.1f", Kp, Kd, Ki, Kw);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(buffer);
    udp.endPacket();
  }
}

float parseValue(String msg, String key) {
  int start = msg.indexOf(key);
  if (start == -1)
    return 0;
  start += key.length();
  int end = msg.indexOf(',', start);
  if (end == -1)
    end = msg.length();
  return msg.substring(start, end).toFloat();
}

void sendTelemetry() {
  char buffer[120];
  sprintf(buffer, "A:%.2f,G:%.2f,P:%d,W:%.0f,I:%.1f,B:%d,O:%.2f", robot_angle,
          gyroZ_filtered, pwm_output, motor_speed, integral, vertical ? 1 : 0,
          balance_angle_offset);

  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  udp.endPacket();
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

void writeRegister(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beepStartup() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}
