// =============================================
// AUTO OFFSET FINDER PRO - CALIBRATE ĐA TRỤC
// =============================================
// Cách dùng:
// 1. Nạp code này vào ESP32
// 2. Đặt robot trên mặt phẳng THẲNG ĐỨNG
// 3. Tool sẽ tự động calibrate TẤT CẢ CÁC TRỤC
// 4. Copy giá trị offset vào code chính

#include <Wire.h>

#define MPU6050 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B

#define BUZZER_PIN 14

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// Tổng để tính trung bình
long AcX_sum = 0, AcY_sum = 0, AcZ_sum = 0;
long GyX_sum = 0, GyY_sum = 0, GyZ_sum = 0;
int sample_count = 0;
const int TOTAL_SAMPLES = 3000; // 3000 mẫu = 30 giây

// Đo độ ổn định
float stability_score = 0;
float prev_angle = 0;
float angle_variance_sum = 0;

// Kết quả
int16_t final_AcX_offset = 0;
int16_t final_AcY_offset = 0;
int16_t final_AcZ_offset = 0;
int16_t final_GyZ_offset = 0;

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, 0);     // ±2g
  writeTo(MPU6050, GYRO_CONFIG, 1 << 3); // ±500°/s
  writeTo(MPU6050, 0x1A, 3);             // DLPF = 44Hz
  delay(100);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("");
  Serial.println("╔═══════════════════════════════════════════════════╗");
  Serial.println("║    🎯 AUTO OFFSET FINDER PRO - CALIBRATE ĐA TRỤC   ║");
  Serial.println("╠═══════════════════════════════════════════════════╣");
  Serial.println("║                                                    ║");
  Serial.println("║  📌 HƯỚNG DẪN:                                     ║");
  Serial.println("║     1. Đặt robot THẲNG ĐỨNG trên mặt phẳng         ║");
  Serial.println("║     2. KHÔNG CHẠM vào robot trong 30 giây          ║");
  Serial.println("║     3. Tool sẽ đo và tính offset TẤT CẢ CÁC TRỤC   ║");
  Serial.println("║                                                    ║");
  Serial.println("╠═══════════════════════════════════════════════════╣");
  Serial.println("║  📊 Đang đo: Accelerometer (AcX, AcY, AcZ)         ║");
  Serial.println("║              Gyroscope (GyZ)                       ║");
  Serial.println("║              Độ ổn định (Stability)                ║");
  Serial.println("╚═══════════════════════════════════════════════════╝");
  Serial.println("");
  Serial.println("⏳ Bắt đầu calibrate trong 3 giây...");
  delay(3000);
  Serial.println("📡 Đang đo...");
  Serial.println("");
}

void loop() {
  // Đọc tất cả dữ liệu từ MPU6050
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B); // Bắt đầu từ ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read(); // Temp (bỏ qua)
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Tích lũy để tính trung bình
  AcX_sum += AcX;
  AcY_sum += AcY;
  AcZ_sum += AcZ;
  GyZ_sum += GyZ;

  // Tính góc hiện tại để đo độ ổn định
  float current_angle = atan2(AcY, -AcX) * 57.2958;
  float angle_diff = current_angle - prev_angle;
  angle_variance_sum += angle_diff * angle_diff;
  prev_angle = current_angle;

  sample_count++;

  // In tiến trình mỗi 300 mẫu (3 giây)
  if (sample_count % 300 == 0) {
    float progress = (float)sample_count / TOTAL_SAMPLES * 100;
    float avg_AcY = (float)AcY_sum / sample_count;
    float avg_angle = atan2(avg_AcY, -(float)AcX_sum / sample_count) * 57.2958;
    float current_variance = sqrt(angle_variance_sum / sample_count);

    Serial.printf(
        "📊 [%3.0f%%] Góc TB: %+6.2f° | Dao động: %.3f° | AcY TB: %.0f\n",
        progress, avg_angle, current_variance, avg_AcY);

    // Cảnh báo nếu không ổn định
    if (current_variance > 0.5) {
      Serial.println("   ⚠️ Robot đang rung! Đặt ổn định hơn!");
    }
  }

  // Hoàn thành calibration
  if (sample_count >= TOTAL_SAMPLES) {
    // Tính trung bình
    float avg_AcX = (float)AcX_sum / sample_count;
    float avg_AcY = (float)AcY_sum / sample_count;
    float avg_AcZ = (float)AcZ_sum / sample_count;
    float avg_GyZ = (float)GyZ_sum / sample_count;

    // Tính góc trung bình
    float avg_angle = atan2(avg_AcY, -avg_AcX) * 57.2958;

    // Tính độ ổn định (variance)
    float stability = sqrt(angle_variance_sum / sample_count);

    // Tính offset cần thiết
    // Mục tiêu: AcY sau offset = 0 khi đứng thẳng
    // Công thức: new_offset = -avg_AcY (để về 0) + adjustment cho góc
    final_AcY_offset = (int16_t)(-avg_AcY + 280 * avg_angle);

    // GyZ offset đơn giản là trung bình (mục tiêu = 0 khi đứng yên)
    final_GyZ_offset = (int16_t)avg_GyZ;

    // Đánh giá chất lượng calibration
    String quality = "❓ KHÔNG XÁC ĐỊNH";
    if (stability < 0.1)
      quality = "🏆 XUẤT SẮC";
    else if (stability < 0.3)
      quality = "✅ TỐT";
    else if (stability < 0.5)
      quality = "⚠️ TRUNG BÌNH";
    else
      quality = "❌ KÉM - Cần calibrate lại!";

    Serial.println("");
    Serial.println("╔═══════════════════════════════════════════════════════╗");
    Serial.println(
        "║                   🎯 KẾT QUẢ CALIBRATION               ║");
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.printf("║  📐 Góc trung bình:     %+7.2f°                       \n",
                  avg_angle);
    Serial.printf("║  📊 Độ ổn định:         %7.3f° (%s)     \n", stability,
                  quality.c_str());
    Serial.printf("║  📈 Số mẫu đo:          %7d                         \n",
                  sample_count);
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.println(
        "║                   📋 GIÁ TRỊ RAW                       ║");
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.printf("║  AcX trung bình:  %+8.0f                             \n",
                  avg_AcX);
    Serial.printf("║  AcY trung bình:  %+8.0f                             \n",
                  avg_AcY);
    Serial.printf("║  AcZ trung bình:  %+8.0f                             \n",
                  avg_AcZ);
    Serial.printf("║  GyZ trung bình:  %+8.0f                             \n",
                  avg_GyZ);
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.println(
        "║                                                        ║");
    Serial.println(
        "║   🔧 COPY CÁC DÒNG SAU VÀO CODE CHÍNH:                 ║");
    Serial.println(
        "║                                                        ║");
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.println("");
    Serial.printf("    int16_t AcX_offset = 0;\n");
    Serial.printf("    int16_t AcY_offset = %d;\n", final_AcY_offset);
    Serial.printf("    int16_t GyZ_offset = %d;\n", final_GyZ_offset);
    Serial.println("");
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.println(
        "║  📁 File: one_axis_reaction_wheel_stick.ino (dòng 65-70)║");
    Serial.println(
        "║  ⭐ Nạp lại code chính với offset mới là xong!         ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝");

    if (stability > 0.5) {
      Serial.println("");
      Serial.println("⚠️ CẢNH BÁO: Độ ổn định thấp!");
      Serial.println("   → Đặt robot ổn định hơn rồi chạy lại tool này.");
    }

    // Bíp 3 lần báo xong
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }

    while (1) {
      delay(1000);
    }
  }

  delay(10); // 100Hz sampling
}
