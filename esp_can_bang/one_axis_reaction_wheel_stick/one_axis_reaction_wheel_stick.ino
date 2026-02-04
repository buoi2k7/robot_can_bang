#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ===== WIFI =====
const char *ssid = "link";
const char *password = "buoinha132/";

// 🖥️ IP máy tính chạy py
const char *udpAddress = "192.168.1.8"; // Thay bằng IP của máy bạn
const int udpPort = 4210;

// 🌐 IP tĩnh cho ESP32
IPAddress local_IP(192, 168, 1, 7); // ✅ Đúng cú pháp
IPAddress gateway(192, 168, 1, 1);  // ✅ Gateway mạng 1.x
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

WiFiUDP udp;

// --- CẤU HÌNH CHÂN (PINOUT) CHO ESP32 ---
// LPT nhớ sửa lại các chân này cho đúng với board của ông nha
#define BRAKE_PIN 26  // Chân phanh
#define PWM_PIN 25    // Chân băm xung động cơ
#define DIR_PIN 27    // Chân chiều động cơ
#define BUZZER_PIN 14 // Chân còi

// Địa chỉ I2C MPU6050
#define MPU6050 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B

// Cấu hình PWM cho ESP32
const int PWM_FREQ = 20000; // 20kHz
// const int PWM_CHANNEL = 0;     // Kênh PWM 0
const int PWM_RES = 8; // Độ phân giải 8-bit (0-255)

// Các biến PID (OPTIMIZED BY AUTO-TUNER)
float X1 = 55.0; // P - phản ứng góc
float X2 = 22.0; // D - phanh (tốc độ góc)
float X3 = 1.2;  // Motor feedback
float loop_time = 10;
float loop_time_py = 50;

float pitch = 0;
float roll = 0;
float yaw = 0;

int pwm_s = 0;
int32_t motor_speed;
long currentT, previousT_1 = 0;
long currentT_py, previousT_1_py = 0;

// Biến cảm biến
int16_t AcX, AcY, AcZ, GyZ;
float gyroZ, gyroZfilt;
#define accSens 0
#define gyroSens 1
#define Gyro_amount 0.996

// Offset (Chạy auto_offset_finder để lấy giá trị chính xác)
int16_t AcX_offset = 0; // Thường = 0
int16_t AcY_offset = 1; // ← Cần calibrate! Chạy auto_offset_finder
int16_t AcZ_offset = -6;
int16_t GyZ_offset = 0; // Tự động calibrate khi khởi động
int32_t GyZ_offset_sum = 0;

float alpha = 0.40;
float robot_angle;
float robot_angle_raw;
float Acc_angle;
bool vertical = false;
float balance_angle_offset = 0.0;
unsigned long balance_stable_start = 0;
bool assist_mode = false;

const float vertical_on_threshold = 0.5;
const float vertical_off_threshold = 9.0;
const float balance_stable_angle = 1.5;
const float balance_stable_gyro = 3.0;
const float balance_stable_pwm = 20.0;
const unsigned long balance_stable_time_ms = 1500;
const float balance_offset_alpha = 0.02;
const float fall_cutoff_angle = 12.0;

const float k1_min = 20.0;
const float k1_max = 120.0;
const float k2_min = 5.0;
const float k2_max = 40.0;
const float k3_min = 0.5;
const float k3_max = 4.0;

// --- SETUP & LOOP ---

void setup() {
  Serial.begin(115200); // ESP32 nên dùng tốc độ cao

  // ===== Cấu hình wifi =====
  Wire.begin();

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("⚠️ Cấu hình IP tĩnh thất bại!");
  }

  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ Kết nối WiFi thành công!");
  udp.begin(udpPort);
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Cấu hình PWM cho ESP32 (thay cho TCCR1A/B cũ)
  // ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(BRAKE_PIN, LOW); // Mới vào phanh lại

  delay(1000);
  angle_setup();
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {

    angle_calc();

    gyroZ = GyZ / 131.0; // Convert to deg/s
    gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;

    updateBalanceReference();
    robot_angle = robot_angle_raw - balance_angle_offset;
    updateVerticalState();
    updateAssistAutoTune();

    if (vertical) {
      digitalWrite(BRAKE_PIN, HIGH);

      // Emergency: Reset motor_speed nếu robot đứng gần 0 nhưng motor quay mạnh
      if (abs(robot_angle) < 0.5 && abs(motor_speed) > 4000) {
        motor_speed *= 1; // Giảm dần motor_speed về 0
      }

      if (abs(robot_angle) > fall_cutoff_angle) {
        Motor_control(0);
        motor_speed = 0;
      } else {
        // Tính PID (Restore K3 nhưng nhẹ nhàng)
        // Dấu - trước motor_speed quan trọng để tạo feedback ngược
        pwm_s =
            constrain(X1 * robot_angle + X2 * gyroZfilt + X3 * -motor_speed,
                      -255, 255);

        Motor_control(pwm_s);
        motor_speed += pwm_s;
        motor_speed = constrain(motor_speed, -6000, 6000);
      }
    } else {
      Motor_control(0);
      digitalWrite(BRAKE_PIN, LOW);
      motor_speed = 0;
    }

    previousT_1 = currentT;
  }

  currentT_py = millis();
  if (currentT_py - previousT_1_py >= loop_time_py) {

    updateToUDP();
    receiveUDP();

    previousT_1_py = currentT;
  }
}
