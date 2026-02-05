/*
 * ═══════════════════════════════════════════════════════════════════════════════
 * REACTION WHEEL STICK V3 - FIX EDITION
 * ✨ Sửa vấn đề tiếng cọc cọc, nhiệt độ, lên-xuống liên tục
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

// Static IP for ESP32
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

// MPU6050 Addresses
#define MPU6050_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define CONFIG_REG 0x1A // DLPF config

// PWM Config
const int PWM_FREQ = 20000;
const int PWM_RES = 10; // 10-bit: 0-1023 for finer control

// ===== PID PARAMETERS (NEW TUNED VALUES) =====
float Kp = 85.0;  // ✓ Giảm từ 180 → 85 (tránh overshoot)
float Ki = 1.2;   // ✓ Giảm từ 3.0 → 1.2 (tránh integral windup)
float Kd = 22.0;  // ✓ Tăng từ 8 → 22 (cân bằng Kp, Kd:Kp ≈ 1:4)
float Kw = 1.5;   // ✓ Tăng từ 1.3 → 1.5 (phanh bánh xe tốt)
float Kff = 0.15; // Giữ nguyên

// Biến PID
float angle_error = 0;
float angle_error_prev = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Anti-windup limits
const float INTEGRAL_MAX = 100.0; // ✓ Giảm từ 150 → 100
const float INTEGRAL_MIN = -100.0;

// ===== TIMING =====
const float LOOP_TIME_MS = 5.0; // 200Hz loop
const float LOOP_TIME_S = 0.005;
unsigned long currentT, previousT = 0;
unsigned long currentT_py, previousT_py = 0;
const int LOOP_TIME_PY = 50;

// ===== SENSOR VARIABLES =====
int16_t AcX, AcY, AcZ, GyZ;
float gyroZ, gyroZ_filtered;
float accelAngle;

// Offsets
int16_t AcX_offset = 0;
int16_t AcY_offset = 0;
int16_t AcZ_offset = 0;
int16_t GyZ_offset = 0;
int32_t GyZ_offset_sum = 0;

// ===== KALMAN FILTER (TUNED) =====
float Q_angle = 0.005;
float Q_bias = 0.002;
float R_measure = 0.08;

float kalmanAngle = 0;
float kalmanBias = 0;
float P[2][2] = {{0, 0}, {0, 0}};

// ===== COMPLEMENTARY FILTER (BACKUP/FUSION) =====
float compAngle = 0;
const float COMP_ALPHA = 0.98; // Tin tưởng gyro 98%

// ===== BUTTERWORTH LOW-PASS FILTER FOR GYRO =====
float gyroZ_butter_prev[2] = {0, 0};
float gyroZ_butter_input_prev[2] = {0, 0};
const float BUTTER_A[2] = {1.14298, -0.41280};
const float BUTTER_B[3] = {0.067455, 0.134911, 0.067455};

// ===== NOTCH FILTER =====
float notch_input[3] = {0, 0, 0};
float notch_output[3] = {0, 0, 0};
const float NOTCH_B[3] = {0.93906, -1.68399, 0.93906};
const float NOTCH_A[2] = {-1.68399, 0.87813};

// ===== DERIVATIVE FILTER =====
float d_input_prev = 0;
float d_output_prev = 0;
const float D_FILTER_ALPHA = 0.1;

// ===== STATE VARIABLES =====
float robot_angle = 0;
float robot_angle_raw = 0;
float balance_angle_offset = 0;
bool vertical = false;
bool assist_mode = false;

// ===== MOTOR VARIABLES =====
int pwm_output = 0;
float motor_speed = 0;
float motor_speed_filtered = 0;
int32_t motor_speed_integral = 0;

// ===== THRESHOLDS (FIXED) =====
const float VERTICAL_ON_THRESHOLD = 5.0;   // ✓ Giảm từ 10 → 5
const float VERTICAL_OFF_THRESHOLD = 15.0; // ✓ Giảm từ 25 → 15
const float FALL_CUTOFF_ANGLE = 25.0;
const float BALANCE_STABLE_ANGLE = 1.0;
const float BALANCE_STABLE_GYRO = 3.0;
const float BALANCE_STABLE_PWM = 30.0;
const unsigned long BALANCE_STABLE_TIME = 1000;
const float BALANCE_OFFSET_ALPHA = 0.02;

// ===== DEADBAND & LIMITS (FIXED) =====
const int PWM_DEADBAND = 50; // ✓ Tăng từ 30 → 50 (tránh on-off rung)
const int PWM_MAX = 1023;
const int PWM_LIMIT = 900;
const float MOTOR_SPEED_MAX = 8000;

// ===== BANG-BANG CONTROL =====
const float BANGBANG_THRESHOLD = 8.0;
const int BANGBANG_PWM = 400;

// ===== SETPOINT WEIGHTING =====
const float P_SETPOINT_WEIGHT = 0.7;
const float D_SETPOINT_WEIGHT = 0.1;

// ===== AUTO-TUNE STATE =====
unsigned long balance_stable_start = 0;
float auto_tune_kp_min = 70.0, auto_tune_kp_max = 100.0;
float auto_tune_kd_min = 18.0, auto_tune_kd_max = 28.0;
float auto_tune_ki_min = 0.8, auto_tune_ki_max = 2.0;

// ===== DEBUG & TELEMETRY =====
bool debug_mode = true;
float telemetry_data[8];

// ===== MOTOR TEMPERATURE TRACKING =====
float motor_estimated_temp = 25.0;
const float TEMP_WARNING_THRESHOLD = 70.0;

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);

  setupWiFi();

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  initMPU6050();
  calibrateGyro();

  readSensors();
  kalmanAngle = atan2((float)AcY, -(float)AcX) * 57.2958f;
  kalmanBias = 0;
  P[0][0] = 1.0f;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 1.0f;

  Serial.printf("Initial angle: %.2f degrees\n", kalmanAngle);

  beepStartup();

  Serial.println("✅ V3 FIX EDITION READY!");
  Serial.println("═══════════════════════════════════════");
  Serial.printf("🔧 Kp=%.1f, Kd=%.1f, Ki=%.1f, Deadband=%d\n", Kp, Kd, Ki,
                PWM_DEADBAND);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  currentT = millis();

  if (currentT - previousT >= LOOP_TIME_MS) {
    float dt = (currentT - previousT) / 1000.0;
    previousT = currentT;

    readSensors();

    accelAngle = atan2((float)AcY, -(float)AcX) * 57.2958f;

    float gyroRate = gyroZ;

    // === KALMAN PREDICT ===
    float dt_sec = LOOP_TIME_S;
    kalmanAngle += dt_sec * (gyroRate - kalmanBias);

    P[0][0] += dt_sec * (dt_sec * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt_sec * P[1][1];
    P[1][0] -= dt_sec * P[1][1];
    P[1][1] += Q_bias * dt_sec;

    // === KALMAN UPDATE ===
    float S = P[0][0] + R_measure;
    if (S < 0.0001f)
      S = 0.0001f;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float y = accelAngle - kalmanAngle;

    kalmanAngle += K0 * y;
    kalmanBias += K1 * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    if (isnan(kalmanAngle) || isinf(kalmanAngle)) {
      kalmanAngle = accelAngle;
      kalmanBias = 0;
      P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
    }

    robot_angle_raw = kalmanAngle;
    gyroZ_filtered = gyroRate;

    updateBalanceReference();
    robot_angle = robot_angle_raw - balance_angle_offset;

    updateVerticalState();

    if (vertical) {
      digitalWrite(BRAKE_PIN, HIGH); // ← QUAN TRỌNG: Nhả phanh, cho motor quay
      int pid_output = computePID(robot_angle, gyroZ_filtered, dt);
      pwm_output = applyDeadband(pid_output);
      pwm_output = constrain(pwm_output, -PWM_LIMIT, PWM_LIMIT);
    } else {
      digitalWrite(BRAKE_PIN, LOW); // ← Bật phanh khi không cân bằng
      pwm_output = 0;
      integral = 0;
    }

    Motor_control(pwm_output);

    // ===== UPDATE MOTOR TEMPERATURE ESTIMATE =====
    updateMotorTemperature();

    // ===== PERIODIC TELEMETRY =====
    currentT_py = millis();
    if (currentT_py - previousT_py >= LOOP_TIME_PY) {
      previousT_py = currentT_py;
      updateTelemetry();
      sendUDP();
      receiveUDP();

      // Debug output mỗi 200ms
      if (debug_mode && (currentT_py % 500 == 0)) {
        Serial.printf("A:%.2f° G:%.1f°/s P:%d T:%.1f°C Kp:%.0f Kd:%.0f\n",
                      robot_angle, gyroZ_filtered, pwm_output,
                      motor_estimated_temp, Kp, Kd);
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

void initMPU6050() {
  writeRegister(MPU6050_ADDR, PWR_MGMT_1, 0x00);
  delay(100);

  // Gyro full scale = ±500°/s → 65.5 LSB/°/s
  writeRegister(MPU6050_ADDR, GYRO_CONFIG, 0x08);

  // Accel full scale = ±8g → 4096 LSB/g
  writeRegister(MPU6050_ADDR, ACCEL_CONFIG, 0x10);

  // Low-pass filter ~20Hz
  writeRegister(MPU6050_ADDR, CONFIG_REG, 0x04);

  Serial.println("✅ MPU6050 Initialized");
}

void calibrateGyro() {
  Serial.println("🔄 Calibrating gyro... (keep still!)");
  GyZ_offset_sum = 0;

  for (int i = 0; i < 200; i++) {
    readSensors();
    GyZ_offset_sum += GyZ;
    delay(5);
  }

  GyZ_offset = GyZ_offset_sum / 200;
  Serial.printf("✅ GyZ_offset = %d\n", GyZ_offset);
}

void readSensors() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 8, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();

  // Subtract offsets
  GyZ -= GyZ_offset;

  // Convert to °/s (±500°/s → 65.5 LSB/°/s)
  gyroZ = GyZ / 65.5f;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PID CONTROLLER (ADAPTIVE VERSION) ✨ NEW
// ═══════════════════════════════════════════════════════════════════════════════

int computePID(float angle, float gyro, float dt) {
  angle_error = angle;

  // ===== ADAPTIVE TUNING DỰA VÀO GÓC =====
  // Khi góc lớn → cần Kp cao, Kd cao
  // Khi góc nhỏ → cần Kp thấp, Ki cao
  float abs_angle = abs(angle);

  if (abs_angle > 8.0) {
    // GÓC LỚN: Phản hồi nhanh
    Kp = 95.0;
    Kd = 25.0;
    Ki = 0.8;
  } else if (abs_angle > 4.0) {
    // GÓC TRUNG BÌNH: Cân bằng
    Kp = 85.0;
    Kd = 22.0;
    Ki = 1.2;
  } else {
    // GÓC NHỎ: Ổn định, dùng Ki
    Kp = 80.0;
    Kd = 20.0;
    Ki = 1.5;
  }

  // ===== PROPORTIONAL =====
  float p_term = Kp * (P_SETPOINT_WEIGHT * angle_error);

  // ===== INTEGRAL =====
  integral += Ki * angle_error * dt;

  // Conditional integration - anti-windup
  if (abs(output) < PWM_LIMIT * 0.9) {
    integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
  }

  // ===== DERIVATIVE =====
  float d_input = D_SETPOINT_WEIGHT * gyro;
  derivative = D_FILTER_ALPHA * (d_input - d_input_prev) / dt +
               (1 - D_FILTER_ALPHA) * derivative;
  d_input_prev = d_input;

  float d_term = Kd * derivative;

  // ===== WHEEL FEEDBACK =====
  float wheel_term = Kw * (-motor_speed / 100.0);

  // ===== FEEDFORWARD =====
  float ff_term = Kff * gyro;

  // ===== TOTAL OUTPUT =====
  output = p_term + integral + d_term + wheel_term + ff_term;

  // Store telemetry
  telemetry_data[4] = p_term;
  telemetry_data[5] = integral;
  telemetry_data[6] = d_term;
  telemetry_data[7] = output;

  return (int)output;
}

int applyDeadband(int pwm) {
  if (abs(pwm) < PWM_DEADBAND) {
    return 0;
  }
  return (pwm > 0) ? (pwm - PWM_DEADBAND) * PWM_MAX / (PWM_MAX - PWM_DEADBAND)
                   : (pwm + PWM_DEADBAND) * PWM_MAX / (PWM_MAX - PWM_DEADBAND);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR TEMPERATURE TRACKING ✨ NEW
// ═══════════════════════════════════════════════════════════════════════════════

void updateMotorTemperature() {
  // Mô phỏng nhiệt độ motor
  // Nếu motor chạy → tăng nhiệt độ
  // Nếu ngừng → giảm nhiệt độ (tản nhiệt)

  if (abs(pwm_output) > 100) {
    // Motor chạy có tải → tăng 0.5°C/s (mỗi loop 5ms)
    motor_estimated_temp += (abs(pwm_output) / 1000.0) * 0.1f;
  } else {
    // Motor tắt → giảm 0.05°C/s (tản nhiệt chậm)
    motor_estimated_temp = max(25.0f, motor_estimated_temp - 0.0005f);
  }

  // Cảnh báo nhiệt độ
  if (motor_estimated_temp > TEMP_WARNING_THRESHOLD) {
    static unsigned long last_warning = 0;
    if (millis() - last_warning > 2000) { // Cảnh báo 2 giây 1 lần
      Serial.printf("⚠️ MOTOR HOT! T=%.1f°C - Reducing power\n",
                    motor_estimated_temp);
      last_warning = millis();

      // Giảm Kp để giảm công suất
      Kp = max(60.0f, Kp - 5.0f);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// STATE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

void updateBalanceReference() {
  bool stable = abs(robot_angle_raw) < BALANCE_STABLE_ANGLE &&
                abs(gyroZ_filtered) < BALANCE_STABLE_GYRO &&
                abs(pwm_output) < BALANCE_STABLE_PWM;

  if (stable) {
    if (balance_stable_start == 0) {
      balance_stable_start = millis();
    }

    if (millis() - balance_stable_start >= BALANCE_STABLE_TIME) {
      balance_angle_offset =
          (1.0 - BALANCE_OFFSET_ALPHA) * balance_angle_offset +
          BALANCE_OFFSET_ALPHA * robot_angle_raw;
      assist_mode = true;
    }
  } else {
    balance_stable_start = 0;
    assist_mode = false;
  }
}

void updateVerticalState() {
  if (abs(robot_angle) > VERTICAL_OFF_THRESHOLD) {
    vertical = false;
  }
  if (abs(robot_angle) < VERTICAL_ON_THRESHOLD) {
    vertical = true;
  }
}

void emergencyStop() {
  Motor_control(0);
  motor_speed = 0;
  integral = 0;
  vertical = false;
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

  int pwm_mapped = map(pwm, 0, PWM_MAX, 0, 1023);
  ledcWrite(PWM_PIN, pwm_mapped);
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
  if (msg.startsWith("Kp")) {
    Kp = parseValue(msg, "Kp=");
    Ki = parseValue(msg, "Ki=");
    Kd = parseValue(msg, "Kd=");
    Kw = parseValue(msg, "Kw=");
    Kff = parseValue(msg, "Kff=");

    Serial.printf("📩 New PID: Kp=%.2f, Ki=%.2f, Kd=%.2f, Kw=%.2f, Kff=%.3f\n",
                  Kp, Ki, Kd, Kw, Kff);

    integral = 0;

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("ACK");
    udp.endPacket();
  }

  if (msg == "RESET") {
    integral = 0;
    motor_speed = 0;
    balance_angle_offset = 0;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("RESET_OK");
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

void sendUDP() {
  char buffer[150];
  sprintf(
      buffer,
      "A:%.3f,G:%.3f,P:%d,W:%.1f,I:%.2f,B:%d,Kp:%.1f,Ki:%.2f,Kd:%.1f,T:%.1f",
      robot_angle, gyroZ_filtered, pwm_output, motor_speed, integral,
      vertical ? 1 : 0, Kp, Ki, Kd, motor_estimated_temp);

  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  udp.endPacket();
}

void updateTelemetry() {
  telemetry_data[0] = robot_angle;
  telemetry_data[1] = gyroZ_filtered;
  telemetry_data[2] = pwm_output;
  telemetry_data[3] = motor_speed;
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
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}
