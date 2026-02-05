// ═══════════════════════════════════════════════════════════════
// FUNCTION IMPLEMENTATIONS FOR V2
// Ported from V1 with improvements + Kalman Filter
// ═══════════════════════════════════════════════════════════════

// ===== HÀM NHẬN DỮ LIỆU TỪ PYTHON (V2 FORMAT) ======
void receiveUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[128];
    int len = udp.read(incoming, 127);
    if (len > 0)
      incoming[len] = '\0';
    String msg = String(incoming);

    // Parse V2 format: "Kp=80.0,Kd=15.0,Ki=0.5,Kw=2.0"
    if (msg.startsWith("Kp")) {
      int kpStart = msg.indexOf('=') + 1;
      int kdStart = msg.indexOf("Kd=") + 3;
      int kiStart = msg.indexOf("Ki=") + 3;
      int kwStart = msg.indexOf("Kw=") + 3;

      X1 = msg.substring(kpStart, msg.indexOf(',', kpStart)).toFloat();
      X2 = msg.substring(kdStart, msg.indexOf(',', kdStart)).toFloat();
      // Note: V2 has Ki, Kw but we're using simplified 3-param PID
      // You can add X4 = Ki, X5 = Kw if needed

      Serial.printf("📩 Nhận hệ số mới: Kp=%.2f, Kd=%.2f\\n", X1, X2);

      // Gửi xác nhận lại Python
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("ACK");
      udp.endPacket();
    }
  }
}

// ===== HÀM CẬP NHẬT GIÁ TRỊ ĐẾN PYTHON (V2 FORMAT) =====
void updateToUDP() {
  char buffer[80];
  // V2 format: "A:angle,G:gyro,P:pwm,W:wheel,B:balance"
  sprintf(buffer, "A:%.2f,G:%.2f,P:%d,W:%ld,B:%d", robot_angle, gyroZfilt,
          pwm_s, motor_speed, vertical ? 1 : 0);

  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  udp.endPacket();
}

// ===== I2C HELPER =====
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

// ===== MPU6050 SETUP =====
void angle_setup() {
  Wire.begin();
  delay(100);

  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);

  // ===== DLPF (Digital Low Pass Filter) =====
  // Setting 3 = 44Hz bandwidth, good balance between noise and delay
  writeTo(MPU6050, 0x1A, 3);

  delay(100);

  // Calibrate Gyro offset
  for (int i = 0; i < 1024; i++) {
    Wire.beginTransmission(MPU6050);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 2, true);
    GyZ = Wire.read() << 8 | Wire.read();

    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10;

  // Beep to signal ready
  digitalWrite(BUZZER_PIN, HIGH);
  delay(70);
  digitalWrite(BUZZER_PIN, LOW);
  delay(80);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(70);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.print("GyZ offset value = ");
  Serial.println(GyZ_offset);
}

// ===== KALMAN FILTER ANGLE CALCULATION =====
// 🔥 MAJOR UPGRADE FROM V1 - Much more accurate than complementary filter
void angle_calc() {
  // Read Accelerometer
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();

  // Read Gyroscope
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  // Apply Offsets
  AcX += AcX_offset;
  AcY += AcY_offset;
  GyZ -= GyZ_offset;

  // ===== KALMAN FILTER =====
  // Combines Gyro (fast, drifts) + Accelerometer (slow, noisy)

  static float Q_angle = 0.001f;  // Process noise - angle
  static float Q_bias = 0.003f;   // Process noise - bias
  static float R_measure = 0.03f; // Measurement noise

  static float angle = 0.0f;
  static float bias = 0.0f;
  static float P[2][2] = {{0, 0}, {0, 0}};

  float dt = loop_time / 1000.0f;
  float gyroRate = GyZ / 131.0f; // ±250°/s sensitivity

  // Calculate angle from accelerometer
  Acc_angle = atan2(AcY, -AcX) * 57.2958f;

  // === PREDICT ===
  angle += dt * (gyroRate - bias);

  // Update error covariance
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // === UPDATE ===
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = Acc_angle - angle;

  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  robot_angle_raw = angle;

  Serial.print(Acc_angle);
  Serial.print("__");
  Serial.println(robot_angle_raw);
}

// ===== AUTO BALANCE REFERENCE =====
void updateBalanceReference() {
  bool stable = abs(robot_angle_raw) < balance_stable_angle &&
                abs(gyroZfilt) < balance_stable_gyro &&
                abs(pwm_s) < balance_stable_pwm;

  if (stable) {
    if (balance_stable_start == 0) {
      balance_stable_start = millis();
    }

    if (millis() - balance_stable_start >= balance_stable_time_ms) {
      balance_angle_offset =
          (1.0f - balance_offset_alpha) * balance_angle_offset +
          balance_offset_alpha * robot_angle_raw;
      assist_mode = true;
    }
  } else {
    balance_stable_start = 0;
    assist_mode = false;
  }
}

// ===== VERTICAL STATE DETECTION =====
void updateVerticalState() {
  if (abs(robot_angle) > vertical_off_threshold)
    vertical = false;
  if (abs(robot_angle) < vertical_on_threshold)
    vertical = true;
}

// ===== AUTO-TUNE PID (WITH FIXED FLOAT LITERALS) =====
void updateAssistAutoTune() {
  if (!assist_mode)
    return;

  float abs_angle = abs(robot_angle);
  float abs_rate = abs(gyroZfilt);

  if (abs_angle > 2.5f) {
    X1 = max(k1_min, X1 - 0.2f);
    X2 = min(k2_max, X2 + 0.1f);
  } else if (abs_angle < 0.8f && abs_rate < 2.0f) {
    X1 = min(k1_max, X1 + 0.1f);
  }

  if (abs_rate > 6.0f) {
    X2 = min(k2_max, X2 + 0.05f);
  }

  X3 = constrain(X3, k3_min, k3_max);
}

// ===== PWM CONTROL =====
void setPWM(int dutyCycle) { ledcWrite(PWM_PIN, dutyCycle); }

// ===== MOTOR CONTROL =====
void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIR_PIN, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }

  // Map inversely for specific motor driver
  int pwm_output = map(pwm, 0, 255, 225, 0);

  setPWM(pwm_output);
}
