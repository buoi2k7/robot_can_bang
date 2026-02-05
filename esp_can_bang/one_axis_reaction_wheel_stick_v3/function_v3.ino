/*
 * ═══════════════════════════════════════════════════════════════════════════════
 * FUNCTION V3 - ADVANCED FILTERS & ALGORITHMS
 * Các hàm bổ sung cho Reaction Wheel Stick V3
 * ═══════════════════════════════════════════════════════════════════════════════
 */

// ===== CẤU HÌNH BỘ LỌC NÂNG CAO =====

// Median Filter (cho loại bỏ outlier)
#define MEDIAN_WINDOW 5
float median_buffer[MEDIAN_WINDOW];
int median_index = 0;

// Moving Average Filter
#define MA_WINDOW 10
float ma_buffer[MA_WINDOW];
int ma_index = 0;

// Exponential Moving Average
float ema_value = 0;
const float EMA_ALPHA = 0.2;

// Rate Limiter (giới hạn tốc độ thay đổi)
float rate_limit_prev = 0;
const float RATE_LIMIT_MAX = 50.0; // Max change per loop

// ===== MEDIAN FILTER =====
// Loại bỏ nhiễu xung (impulse noise)
float medianFilter(float new_value) {
  median_buffer[median_index] = new_value;
  median_index = (median_index + 1) % MEDIAN_WINDOW;

  // Copy và sort
  float sorted[MEDIAN_WINDOW];
  for (int i = 0; i < MEDIAN_WINDOW; i++) {
    sorted[i] = median_buffer[i];
  }

  // Bubble sort
  for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
    for (int j = 0; j < MEDIAN_WINDOW - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  return sorted[MEDIAN_WINDOW / 2]; // Return median
}

// ===== MOVING AVERAGE FILTER =====
float movingAverageFilter(float new_value) {
  ma_buffer[ma_index] = new_value;
  ma_index = (ma_index + 1) % MA_WINDOW;

  float sum = 0;
  for (int i = 0; i < MA_WINDOW; i++) {
    sum += ma_buffer[i];
  }

  return sum / MA_WINDOW;
}

// ===== EXPONENTIAL MOVING AVERAGE =====
float emaFilter(float new_value) {
  ema_value = EMA_ALPHA * new_value + (1 - EMA_ALPHA) * ema_value;
  return ema_value;
}

// ===== RATE LIMITER =====
// Ngăn chặn thay đổi đột ngột (good for motor control)
float rateLimiter(float new_value) {
  float delta = new_value - rate_limit_prev;

  if (delta > RATE_LIMIT_MAX) {
    delta = RATE_LIMIT_MAX;
  } else if (delta < -RATE_LIMIT_MAX) {
    delta = -RATE_LIMIT_MAX;
  }

  rate_limit_prev += delta;
  return rate_limit_prev;
}

// ===== HYSTERESIS =====
// Tránh chuyển trạng thái liên tục quanh ngưỡng
class Hysteresis {
private:
  float low_threshold;
  float high_threshold;
  bool state;

public:
  Hysteresis(float low, float high, bool initial = false) {
    low_threshold = low;
    high_threshold = high;
    state = initial;
  }

  bool update(float value) {
    if (value > high_threshold) {
      state = true;
    } else if (value < low_threshold) {
      state = false;
    }
    return state;
  }

  bool getState() { return state; }
};

// ===== DEADBAND WITH HYSTERESIS =====
// Vùng chết có trễ để tránh rung
float deadbandHysteresis(float value, float threshold, float hysteresis) {
  static bool in_deadband = true;

  if (in_deadband) {
    if (abs(value) > threshold + hysteresis) {
      in_deadband = false;
      return value;
    }
    return 0;
  } else {
    if (abs(value) < threshold - hysteresis) {
      in_deadband = true;
      return 0;
    }
    return value;
  }
}

// ===== PID CONTROLLER CLASS =====
// Full-featured PID với mọi tính năng
class PIDController {
private:
  float kp, ki, kd;
  float integral = 0;
  float prev_error = 0;
  float prev_measurement = 0;
  float integral_limit = 100;
  float output_limit = 255;
  float derivative_filter = 0.1;
  bool d_on_measurement = true; // D on measurement thay vì error

public:
  PIDController(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  void setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  void setIntegralLimit(float limit) { integral_limit = limit; }
  void setOutputLimit(float limit) { output_limit = limit; }
  void setDerivativeFilter(float alpha) { derivative_filter = alpha; }

  float compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float P = kp * error;

    // Integral với anti-windup
    integral += ki * error * dt;
    integral = constrain(integral, -integral_limit, integral_limit);
    float I = integral;

    // Derivative
    float D;
    if (d_on_measurement) {
      // D on measurement - tránh derivative kick
      float d_input = (measurement - prev_measurement) / dt;
      d_input =
          derivative_filter * d_input + (1 - derivative_filter) * prev_error;
      D = -kd * d_input;
      prev_error = d_input;
    } else {
      D = kd * (error - prev_error) / dt;
    }

    prev_measurement = measurement;
    prev_error = error;

    float output = P + I + D;
    return constrain(output, -output_limit, output_limit);
  }

  void reset() {
    integral = 0;
    prev_error = 0;
    prev_measurement = 0;
  }
};

// ===== LEAKY INTEGRATOR =====
// Tích phân có rò rỉ - tự động giảm integral khi không cần
float leakyIntegrator(float input, float leak_rate, float dt) {
  static float state = 0;
  state = state * (1 - leak_rate * dt) + input * dt;
  return state;
}

// ===== CASCADED PID =====
// PID 2 vòng: vòng ngoài góc, vòng trong tốc độ
class CascadedPID {
private:
  PIDController outer_pid; // Angle control
  PIDController inner_pid; // Rate control

public:
  CascadedPID(float kp_o, float ki_o, float kd_o, float kp_i, float ki_i,
              float kd_i)
      : outer_pid(kp_o, ki_o, kd_o), inner_pid(kp_i, ki_i, kd_i) {}

  float compute(float angle_setpoint, float angle, float rate, float dt) {
    // Outer loop: angle -> rate setpoint
    float rate_setpoint = outer_pid.compute(angle_setpoint, angle, dt);

    // Inner loop: rate -> output
    return inner_pid.compute(rate_setpoint, rate, dt);
  }

  void reset() {
    outer_pid.reset();
    inner_pid.reset();
  }
};

// ===== FUZZY LOGIC CONTROLLER (đơn giản) =====
// Dùng cho fine-tuning khi gần cân bằng
float fuzzyController(float error, float rate) {
  // Membership functions đơn giản
  float e_neg = max(0.0f, min(1.0f, (-error + 2.0f) / 4.0f));
  float e_zero = max(0.0f, min(1.0f, 1.0f - abs(error) / 2.0f));
  float e_pos = max(0.0f, min(1.0f, (error + 2.0f) / 4.0f));

  float r_neg = max(0.0f, min(1.0f, (-rate + 5.0f) / 10.0f));
  float r_zero = max(0.0f, min(1.0f, 1.0f - abs(rate) / 5.0f));
  float r_pos = max(0.0f, min(1.0f, (rate + 5.0f) / 10.0f));

  // Rules đơn giản
  float output_neg = max(min(e_pos, r_pos), min(e_zero, r_pos));
  float output_zero =
      max(max(min(e_zero, r_zero), min(e_neg, r_pos)), min(e_pos, r_neg));
  float output_pos = max(min(e_neg, r_neg), min(e_zero, r_neg));

  // Defuzzification (centroid)
  float numerator = -50 * output_neg + 0 * output_zero + 50 * output_pos;
  float denominator = output_neg + output_zero + output_pos;

  if (denominator < 0.001)
    return 0;
  return numerator / denominator;
}

// ===== VIBRATION DETECTION =====
// Phát hiện rung động để điều chỉnh Kd
float vibrationLevel() {
  static float prev_gyro = 0;
  static float vibration = 0;

  float diff = abs(gyroZ_filtered - prev_gyro);
  vibration = 0.9 * vibration + 0.1 * diff;
  prev_gyro = gyroZ_filtered;

  return vibration;
}

// ===== MOTOR PROTECTION =====
// Giới hạn duty cycle và nhiệt độ
class MotorProtection {
private:
  float current_limit;
  float temp_limit;
  unsigned long last_check;
  int consecutive_overcurrent;

public:
  MotorProtection(float current, float temp) {
    current_limit = current;
    temp_limit = temp;
    consecutive_overcurrent = 0;
  }

  bool checkSafe(float current, float temp) {
    if (current > current_limit) {
      consecutive_overcurrent++;
      if (consecutive_overcurrent > 10) {
        return false; // Overcurrent trip
      }
    } else {
      consecutive_overcurrent =
          (consecutive_overcurrent > 1) ? consecutive_overcurrent - 1 : 0;
    }

    if (temp > temp_limit) {
      return false; // Overtemp trip
    }

    return true;
  }

  void reset() { consecutive_overcurrent = 0; }
};

// ===== DATA LOGGER =====
// Lưu dữ liệu vào buffer để debug
#define LOG_SIZE 100
struct LogEntry {
  float angle;
  float gyro;
  float pwm;
  float integral;
  unsigned long timestamp;
};

LogEntry log_buffer[LOG_SIZE];
int log_index = 0;
bool log_full = false;

void logData(float angle, float gyro, float pwm, float integral) {
  log_buffer[log_index] = {angle, gyro, pwm, integral, millis()};
  log_index++;
  if (log_index >= LOG_SIZE) {
    log_index = 0;
    log_full = true;
  }
}

void printLog() {
  int count = log_full ? LOG_SIZE : log_index;
  Serial.println("=== DATA LOG ===");
  Serial.println("Time,Angle,Gyro,PWM,Integral");

  for (int i = 0; i < count; i++) {
    LogEntry &e = log_buffer[i];
    Serial.printf("%lu,%.3f,%.3f,%.0f,%.3f\n", e.timestamp, e.angle, e.gyro,
                  e.pwm, e.integral);
  }
}

// ===== SERIAL COMMAND PROCESSOR =====
// Xử lý lệnh từ Serial
void processSerialCommand() {
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "log") {
    printLog();
  } else if (cmd == "reset") {
    integral = 0;
    motor_speed = 0;
    balance_angle_offset = 0;
    Serial.println("✅ Reset done");
  } else if (cmd == "status") {
    Serial.printf("Angle: %.2f, Gyro: %.2f, PWM: %d, I: %.2f\n", robot_angle,
                  gyroZ_filtered, pwm_output, integral);
  } else if (cmd.startsWith("kp ")) {
    Kp = cmd.substring(3).toFloat();
    Serial.printf("Kp = %.2f\n", Kp);
  } else if (cmd.startsWith("ki ")) {
    Ki = cmd.substring(3).toFloat();
    Serial.printf("Ki = %.2f\n", Ki);
  } else if (cmd.startsWith("kd ")) {
    Kd = cmd.substring(3).toFloat();
    Serial.printf("Kd = %.2f\n", Kd);
  } else if (cmd == "help") {
    Serial.println("Commands: log, reset, status, kp X, ki X, kd X");
  }
}
