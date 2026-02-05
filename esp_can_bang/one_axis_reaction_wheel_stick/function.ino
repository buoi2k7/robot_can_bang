// --- CÁC HÀM HỖ TRỢ (Functions) ---

// ===== HÀM NHẬN DỮ LIỆU TỪ PYTHON ======
void receiveUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[128];
    int len = udp.read(incoming, 127);
    if (len > 0)
      incoming[len] = '\0';
    String msg = String(incoming);

    if (msg.startsWith("K1")) {
      int k1Start = msg.indexOf('=') + 1;
      int k2Start = msg.indexOf("K2=") + 3;
      int k3Start = msg.indexOf("K3=") + 3;

      X1 = msg.substring(k1Start, msg.indexOf(',', k1Start)).toFloat();
      X2 = msg.substring(k2Start, msg.indexOf(',', k2Start)).toFloat();
      X3 = msg.substring(k3Start).toFloat();

      Serial.printf("📩 Nhận hệ số mới: K1=%.2f, K2=%.2f, K3=%.2f\n", X1, X2,
                    X3);

      // Gửi xác nhận lại Python
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("KACK");
      udp.endPacket();
    }
  }
}

// ===== HÀM CẬP NHẬT GIÁ TRỊ ĐẾN PYTHON =====
void updateToUDP() {
  char buffer[64];
  sprintf(buffer, "%.2f,%.2f,%.2f", pitch, pwm_s, robot_angle);

  udp.beginPacket(udpAddress, udpPort);
  udp.print(buffer);
  udp.endPacket();
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  // Khởi động I2C: ESP32 mặc định SDA=21, SCL=22
  Wire.begin();
  delay(100);

  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);

  // ===== BỘ LỌC DLPF (Digital Low Pass Filter) =====
  // Register 0x1A - CONFIG
  // Giá trị | Bandwidth | Delay   | Noise
  // --------|-----------|---------|-------
  //    0    | 260 Hz    | 0 ms    | Nhiều nhiễu
  //    1    | 184 Hz    | 2 ms    |
  //    2    | 94 Hz     | 3 ms    |
  //    3    | 44 Hz     | 4.9 ms  | Cân bằng
  //    4    | 21 Hz     | 8.5 ms  |
  //    5    | 10 Hz     | 13.8 ms | Ít nhiễu nhưng chậm
  //    6    | 5 Hz      | 19 ms   | Rất mượt
  writeTo(MPU6050, 0x1A, 3); // DLPF = 3 (44Hz, cân bằng giữa nhiễu và độ trễ)

  delay(100);

  // Calibrate Gyro ban đầu
  for (int i = 0; i < 1024; i++) {
    // Đọc nhanh GyZ để lấy offset
    Wire.beginTransmission(MPU6050);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 2, true);
    GyZ = Wire.read() << 8 | Wire.read();

    GyZ_offset_sum += GyZ;
    delay(3); // Giảm delay chút cho nhanh
  }
  GyZ_offset = GyZ_offset_sum >> 10;

  // Bíp còi báo xong
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

void angle_calc() {
  // Đọc Accelerometer
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();

  // Đọc Gyroscope
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  // Trừ Offset
  AcX += AcX_offset;
  AcY += AcY_offset;
  GyZ -= GyZ_offset;

  // ===== KALMAN FILTER =====
  // Bộ lọc thông minh kết hợp Gyro + Accelerometer

  // Các biến Kalman (static = giữ giá trị giữa các lần gọi)
  static float Q_angle = 0.001;  // Process noise - góc (nhỏ = tin gyro hơn)
  static float Q_bias = 0.003;   // Process noise - bias
  static float R_measure = 0.03; // Measurement noise (nhỏ = tin accel hơn)

  static float angle = 0;                  // Góc ước lượng
  static float bias = 0;                   // Bias ước lượng
  static float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

  float dt = loop_time / 1000.0; // Delta time (seconds)
  float gyroRate =
      GyZ / 131.0; // Convert to deg/s (131 = sensitivity for ±250°/s)

  // Tính góc từ accelerometer
  Acc_angle = atan2(AcY, -AcX) * 57.2958;

  // === PREDICT (Dự đoán) ===
  angle += dt * (gyroRate - bias);

  // Cập nhật error covariance
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // === UPDATE (Cập nhật với đo lường) ===
  float S = P[0][0] + R_measure; // Innovation covariance
  float K[2];                    // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Tính sai lệch (innovation)
  float y = Acc_angle - angle;

  // Cập nhật ước lượng
  angle += K[0] * y;
  bias += K[1] * y;

  // Cập nhật error covariance
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  // Kết quả Kalman (raw, chưa trừ offset cân bằng)
  robot_angle_raw = angle;

  Serial.print(Acc_angle);
  Serial.print("__");
  Serial.println(robot_angle_raw);
}

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
          (1.0 - balance_offset_alpha) * balance_angle_offset +
          balance_offset_alpha * robot_angle_raw;
      assist_mode = true;
    }
  } else {
    balance_stable_start = 0;
    assist_mode = false;
  }
}

void updateVerticalState() {
  if (abs(robot_angle) > vertical_off_threshold)
    vertical = false;
  if (abs(robot_angle) < vertical_on_threshold)
    vertical = true;
}

void updateAssistAutoTune() {
  if (!assist_mode)
    return;

  float abs_angle = abs(robot_angle);
  float abs_rate = abs(gyroZfilt);

  if (abs_angle > 2.5) {
    X1 = max(k1_min, X1 - 0.2);
    X2 = min(k2_max, X2 + 0.1);
  } else if (abs_angle < 0.8 && abs_rate < 2.0) {
    X1 = min(k1_max, X1 + 0.1);
  }

  if (abs_rate > 6.0) {
    X2 = min(k2_max, X2 + 0.05);
  }

  X3 = constrain(X3, k3_min, k3_max);
}

void setPWM(int dutyCycle) {
  // Ghi giá trị PWM vào kênh đã cấu hình
  ledcWrite(PWM_PIN, dutyCycle);
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIR_PIN, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIR_PIN, HIGH);
  }

  // Code cũ của LPT map ngược: map(pwm, 0, 255, PWMVALUE, 0)
  // Tui giữ nguyên logic ngược này cho driver của ông
  int pwm_output = map(pwm, 0, 255, 225, 0);

  setPWM(pwm_output);
}
