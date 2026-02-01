#!/usr/bin/env python3
"""
ADVANCED PID AUTO-TUNER cho Robot Cân Bằng
==========================================
Sử dụng Genetic Algorithm + Adaptive Grid Search
Tự động tìm bộ PID tối ưu với nhiều chiến lược

Author: Auto-generated
Version: 2.0
"""

import socket
import time
import json
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
import random
from datetime import datetime

# ============== CẤU HÌNH ==============
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
ESP32_IP = "192.168.1.7"
ESP32_PORT = 4210

# Safety thresholds
MAX_ANGLE = 25.0          # Góc tối đa trước khi coi là ngã
DANGER_ANGLE = 20.0       # Góc cảnh báo
MAX_OSCILLATION = 5.0     # Dao động tối đa cho phép
SETTLE_TIME = 2.0         # Thời gian ổn định

# Tuning parameters
POPULATION_SIZE = 12      # Số cá thể mỗi thế hệ (Genetic)
GENERATIONS = 10          # Số thế hệ tối đa
MUTATION_RATE = 0.3       # Tỷ lệ đột biến
TEST_DURATION = 4.0       # Thời gian test mỗi bộ (giây)

# PID Ranges (dựa trên kinh nghiệm robot cân bằng)
K1_RANGE = (30, 120)      # P gain
K2_RANGE = (5, 40)        # D gain  
K3_RANGE = (0.1, 3.0)     # Motor feedback

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

# ============== DATA CLASSES ==============
@dataclass
class PIDConfig:
    k1: float  # P
    k2: float  # D
    k3: float  # Motor feedback
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.k1, self.k2, self.k3)
    
    def __str__(self):
        return f"K1={self.k1:.2f}, K2={self.k2:.2f}, K3={self.k3:.2f}"

@dataclass
class PerformanceMetrics:
    """Metrics đánh giá hiệu suất"""
    avg_error: float          # Lỗi trung bình
    max_error: float          # Lỗi tối đa
    std_error: float          # Độ dao động
    settle_time: float        # Thời gian ổn định
    overshoot_count: int      # Số lần vọt lố
    stability_score: float    # Điểm tổng hợp (0-100)
    success: bool             # Robot có đứng được không
    
    def __str__(self):
        return (f"Avg:{self.avg_error:.2f}° | Max:{self.max_error:.2f}° | "
                f"Std:{self.std_error:.2f}° | Score:{self.stability_score:.1f}")

# ============== COMMUNICATION ==============
def send_gains(config: PIDConfig):
    """Gửi hệ số PID xuống ESP32"""
    msg = f"K1={config.k1:.2f},K2={config.k2:.2f},K3={config.k3:.2f}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    print(f"  📤 {msg}")
    time.sleep(0.6)  # Đợi ESP32 áp dụng

def clear_buffer():
    """Xóa buffer UDP"""
    try:
        while True:
            sock.recv(1024)
    except:
        pass

# ============== MEASUREMENT ==============
def measure_performance(config: PIDConfig, duration: float = TEST_DURATION) -> Optional[PerformanceMetrics]:
    """
    Đo và đánh giá hiệu suất robot với bộ PID
    Returns None nếu robot ngã
    """
    send_gains(config)
    time.sleep(1.0)  # Đợi ổn định ban đầu
    clear_buffer()
    
    angles = []
    timestamps = []
    start_time = time.time()
    settled = False
    settle_start = None
    overshoot_count = 0
    last_angle = 0
    
    print(f"  ⏳ Testing {duration}s", end="", flush=True)
    
    while time.time() - start_time < duration:
        try:
            data, _ = sock.recvfrom(1024)
            decoded = data.decode().strip()
            
            # Skip acknowledgment messages
            if decoded.startswith("K") or decoded.startswith("KACK"):
                continue
            
            parts = decoded.split(',')
            if len(parts) >= 3:
                angle = float(parts[2])
            else:
                angle = float(parts[0])
            
            # Safety check - ngã
            if abs(angle) > MAX_ANGLE:
                print(f" ❌ FELL! (angle={angle:.1f}°)")
                return None
            
            # Warning check
            if abs(angle) > DANGER_ANGLE:
                print("⚠️", end="", flush=True)
            
            current_time = time.time() - start_time
            angles.append(abs(angle))
            timestamps.append(current_time)
            
            # Đếm overshoot (đổi dấu đột ngột)
            if len(angles) > 1:
                if (last_angle * angle < 0) and abs(angle) > 3.0:
                    overshoot_count += 1
            
            # Kiểm tra thời gian ổn định
            if not settled and abs(angle) < 2.0:
                if settle_start is None:
                    settle_start = current_time
                elif current_time - settle_start > SETTLE_TIME:
                    settled = True
            elif abs(angle) >= 2.0:
                settle_start = None
            
            last_angle = angle
            
        except socket.timeout:
            print(".", end="", flush=True)
        except Exception as e:
            pass
    
    print(" ✓")
    
    # Kiểm tra đủ dữ liệu
    if len(angles) < 20:
        print(f"  ⚠️ Not enough data ({len(angles)} samples)")
        return None
    
    # Tính toán metrics
    avg_error = np.mean(angles)
    max_error = np.max(angles)
    std_error = np.std(angles)
    
    settle_time_val = settle_start if settle_start else duration
    
    # Tính điểm tổng hợp (0-100, càng cao càng tốt)
    # Weights: avg_error (40%), std (30%), settle_time (20%), overshoot (10%)
    score = 100
    score -= min(avg_error * 8, 40)          # Penalty cho lỗi TB
    score -= min(std_error * 10, 30)         # Penalty cho dao động
    score -= min(settle_time_val * 10, 20)   # Penalty cho thời gian ổn định
    score -= min(overshoot_count * 2, 10)    # Penalty cho overshoot
    score = max(score, 0)
    
    metrics = PerformanceMetrics(
        avg_error=avg_error,
        max_error=max_error,
        std_error=std_error,
        settle_time=settle_time_val,
        overshoot_count=overshoot_count,
        stability_score=score,
        success=True
    )
    
    print(f"  📊 {metrics}")
    return metrics

# ============== GENETIC ALGORITHM ==============
class GeneticPIDTuner:
    """Genetic Algorithm để tìm PID tối ưu"""
    
    def __init__(self):
        self.population: List[Tuple[PIDConfig, float]] = []
        self.best_ever: Optional[Tuple[PIDConfig, PerformanceMetrics]] = None
        self.history = []
    
    def random_config(self) -> PIDConfig:
        """Tạo PID config ngẫu nhiên trong range"""
        return PIDConfig(
            k1=random.uniform(*K1_RANGE),
            k2=random.uniform(*K2_RANGE),
            k3=random.uniform(*K3_RANGE)
        )
    
    def initialize_population(self):
        """Khởi tạo quần thể ban đầu"""
        print("\n🧬 Initializing population...")
        
        # Một số config "seed" tốt dựa trên kinh nghiệm
        seeds = [
            PIDConfig(60, 15, 1.5),
            PIDConfig(70, 20, 1.0),
            PIDConfig(50, 25, 1.2),
            PIDConfig(80, 18, 0.8),
        ]
        
        self.population = []
        
        # Thêm seeds
        for seed in seeds[:min(4, POPULATION_SIZE)]:
            self.population.append((seed, 0))
        
        # Thêm random
        while len(self.population) < POPULATION_SIZE:
            self.population.append((self.random_config(), 0))
    
    def evaluate_population(self, generation: int):
        """Đánh giá toàn bộ quần thể"""
        print(f"\n📊 Generation {generation + 1}/{GENERATIONS}")
        print("=" * 70)
        
        evaluated = []
        
        for idx, (config, _) in enumerate(self.population):
            print(f"\n[{idx+1}/{len(self.population)}] Testing: {config}")
            
            metrics = measure_performance(config)
            
            if metrics is None:
                # Robot ngã = điểm 0
                score = 0
                print(f"  ❌ Failed - Score: 0")
            else:
                score = metrics.stability_score
                print(f"  ✅ Success - Score: {score:.1f}")
                
                # Cập nhật best ever
                if self.best_ever is None or score > self.best_ever[1].stability_score:
                    self.best_ever = (config, metrics)
                    print(f"  🏆 NEW BEST!")
            
            evaluated.append((config, score))
            
            # Log history
            self.history.append({
                'generation': generation,
                'config': config.to_tuple(),
                'score': score
            })
        
        self.population = evaluated
        
        # Sort theo score giảm dần
        self.population.sort(key=lambda x: x[1], reverse=True)
        
        print(f"\n📈 Generation {generation + 1} Summary:")
        print(f"  Best score: {self.population[0][1]:.1f}")
        print(f"  Avg score: {np.mean([s for _, s in self.population]):.1f}")
    
    def crossover(self, parent1: PIDConfig, parent2: PIDConfig) -> PIDConfig:
        """Lai ghép 2 config"""
        # Blend crossover
        alpha = random.random()
        return PIDConfig(
            k1=alpha * parent1.k1 + (1 - alpha) * parent2.k1,
            k2=alpha * parent1.k2 + (1 - alpha) * parent2.k2,
            k3=alpha * parent1.k3 + (1 - alpha) * parent2.k3
        )
    
    def mutate(self, config: PIDConfig) -> PIDConfig:
        """Đột biến config"""
        def mutate_value(val, min_val, max_val):
            if random.random() < MUTATION_RATE:
                # Gaussian mutation
                delta = random.gauss(0, (max_val - min_val) * 0.1)
                return np.clip(val + delta, min_val, max_val)
            return val
        
        return PIDConfig(
            k1=mutate_value(config.k1, *K1_RANGE),
            k2=mutate_value(config.k2, *K2_RANGE),
            k3=mutate_value(config.k3, *K3_RANGE)
        )
    
    def evolve(self):
        """Tiến hóa sang thế hệ mới"""
        # Giữ lại top 20% (elitism)
        elite_count = max(2, POPULATION_SIZE // 5)
        new_population = self.population[:elite_count]
        
        # Tạo con cái từ top 50%
        parents_pool = self.population[:max(4, POPULATION_SIZE // 2)]
        
        while len(new_population) < POPULATION_SIZE:
            # Chọn 2 cha mẹ ngẫu nhiên từ pool
            parent1 = random.choice(parents_pool)[0]
            parent2 = random.choice(parents_pool)[0]
            
            # Lai ghép
            child = self.crossover(parent1, parent2)
            
            # Đột biến
            child = self.mutate(child)
            
            new_population.append((child, 0))
        
        self.population = new_population
    
    def run(self) -> Optional[Tuple[PIDConfig, PerformanceMetrics]]:
        """Chạy genetic algorithm"""
        print("\n" + "=" * 70)
        print("  🧬 GENETIC ALGORITHM PID TUNER")
        print("=" * 70)
        print(f"📊 Population: {POPULATION_SIZE}")
        print(f"📊 Generations: {GENERATIONS}")
        print(f"📊 Test duration: {TEST_DURATION}s per config")
        print("=" * 70)
        
        input("\n👉 Đặt robot thẳng đứng, nhấn ENTER để bắt đầu...")
        
        self.initialize_population()
        
        for gen in range(GENERATIONS):
            self.evaluate_population(gen)
            
            # Kiểm tra nếu đã đủ tốt (score > 80)
            if self.best_ever and self.best_ever[1].stability_score > 80:
                print(f"\n✨ Found excellent solution! (Score > 80)")
                break
            
            # Tiến hóa (trừ generation cuối)
            if gen < GENERATIONS - 1:
                print(f"\n🔄 Evolving to generation {gen + 2}...")
                self.evolve()
        
        return self.best_ever

# ============== GRID SEARCH (BACKUP) ==============
def grid_search_fine_tune(initial_config: PIDConfig, range_percent: float = 0.2) -> Optional[Tuple[PIDConfig, PerformanceMetrics]]:
    """
    Fine-tune xung quanh config tốt nhất bằng grid search
    """
    print("\n" + "=" * 70)
    print("  🔍 FINE-TUNING với Grid Search")
    print("=" * 70)
    
    # Tạo grid nhỏ xung quanh config hiện tại
    k1_range = [
        initial_config.k1 * (1 - range_percent),
        initial_config.k1,
        initial_config.k1 * (1 + range_percent)
    ]
    k2_range = [
        initial_config.k2 * (1 - range_percent),
        initial_config.k2,
        initial_config.k2 * (1 + range_percent)
    ]
    k3_range = [
        initial_config.k3 * (1 - range_percent),
        initial_config.k3,
        initial_config.k3 * (1 + range_percent)
    ]
    
    best = None
    best_score = -1
    
    total = len(k1_range) * len(k2_range) * len(k3_range)
    tested = 0
    
    for k1 in k1_range:
        for k2 in k2_range:
            for k3 in k3_range:
                tested += 1
                config = PIDConfig(
                    k1=np.clip(k1, *K1_RANGE),
                    k2=np.clip(k2, *K2_RANGE),
                    k3=np.clip(k3, *K3_RANGE)
                )
                
                print(f"\n[{tested}/{total}] Testing: {config}")
                metrics = measure_performance(config)
                
                if metrics and metrics.stability_score > best_score:
                    best = (config, metrics)
                    best_score = metrics.stability_score
                    print(f"  🏆 NEW BEST! Score: {best_score:.1f}")
    
    return best

# ============== MAIN ==============
def main():
    """Main tuning process"""
    print("\n" + "=" * 70)
    print("  🤖 ADVANCED PID AUTO-TUNER v2.0")
    print("=" * 70)
    print("📌 Chiến lược:")
    print("  1. Genetic Algorithm tìm vùng tối ưu")
    print("  2. Grid Search fine-tune chính xác")
    print("=" * 70)
    
    # Chọn phương pháp
    print("\nChọn phương pháp:")
    print("  1. Genetic Algorithm (khuyến nghị - thông minh)")
    print("  2. Grid Search đơn giản (chậm nhưng chắc chắn)")
    print("  3. Hybrid (GA + Grid fine-tune)")
    
    choice = input("\nLựa chọn (1/2/3) [default=3]: ").strip() or "3"
    
    result = None
    
    if choice == "1":
        # Genetic only
        tuner = GeneticPIDTuner()
        result = tuner.run()
    
    elif choice == "2":
        # Grid search toàn bộ (có thể rất lâu)
        print("\n⚠️ Grid search toàn vùng sẽ mất nhiều thời gian!")
        configs = []
        for k1 in np.linspace(*K1_RANGE, 5):
            for k2 in np.linspace(*K2_RANGE, 4):
                for k3 in np.linspace(*K3_RANGE, 3):
                    configs.append(PIDConfig(k1, k2, k3))
        
        print(f"📊 Tổng số config cần test: {len(configs)}")
        input("Nhấn ENTER để tiếp tục...")
        
        best_score = -1
        for idx, config in enumerate(configs):
            print(f"\n[{idx+1}/{len(configs)}] {config}")
            metrics = measure_performance(config)
            if metrics and metrics.stability_score > best_score:
                result = (config, metrics)
                best_score = metrics.stability_score
    
    else:  # choice == "3" - Hybrid
        # GA first
        tuner = GeneticPIDTuner()
        ga_result = tuner.run()
        
        if ga_result:
            print(f"\n✅ GA completed. Best score: {ga_result[1].stability_score:.1f}")
            print(f"🔍 Now fine-tuning around: {ga_result[0]}")
            
            # Grid search fine-tune
            result = grid_search_fine_tune(ga_result[0], range_percent=0.15)
            
            # So sánh với kết quả GA
            if result[1].stability_score <= ga_result[1].stability_score:
                print("\n📌 GA result was better, keeping it")
                result = ga_result
        else:
            result = None
    
    # ============== KẾT QUẢ ==============
    print("\n" + "=" * 70)
    print("  🏆 KẾT QUẢ CUỐI CÙNG")
    print("=" * 70)
    
    if result:
        config, metrics = result
        print(f"\n✅ BỘ PID TỐI ƯU:")
        print(f"   K1 (P)     = {config.k1:.2f}")
        print(f"   K2 (D)     = {config.k2:.2f}")
        print(f"   K3 (Motor) = {config.k3:.2f}")
        print(f"\n📊 Performance:")
        print(f"   Stability Score: {metrics.stability_score:.1f}/100")
        print(f"   Avg Error: {metrics.avg_error:.2f}°")
        print(f"   Max Error: {metrics.max_error:.2f}°")
        print(f"   Std Dev: {metrics.std_error:.2f}°")
        print(f"   Settle Time: {metrics.settle_time:.2f}s")
        print(f"   Overshoot Count: {metrics.overshoot_count}")
        
        # Lưu file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"best_pid_{timestamp}.txt"
        
        with open(filename, "w", encoding="utf-8") as f:
            f.write(f"=== BEST PID CONFIGURATION ===\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"K1 = {config.k1:.2f}\n")
            f.write(f"K2 = {config.k2:.2f}\n")
            f.write(f"K3 = {config.k3:.2f}\n\n")
            f.write(f"Stability Score: {metrics.stability_score:.1f}/100\n")
            f.write(f"Avg Error: {metrics.avg_error:.2f}°\n")
            f.write(f"Max Error: {metrics.max_error:.2f}°\n")
            f.write(f"Std Dev: {metrics.std_error:.2f}°\n")
            f.write(f"Settle Time: {metrics.settle_time:.2f}s\n")
            f.write(f"Overshoot: {metrics.overshoot_count} times\n\n")
            f.write(f"=== ESP32 CODE ===\n")
            f.write(f"float X1 = {config.k1:.2f};  // P gain\n")
            f.write(f"float X2 = {config.k2:.2f};  // D gain\n")
            f.write(f"float X3 = {config.k3:.2f};  // Motor feedback\n")
        
        print(f"\n💾 Đã lưu vào: {filename}")
        
        # Gửi vào ESP32
        print(f"\n📤 Gửi bộ số tốt nhất vào ESP32...")
        send_gains(config)
        print("✅ Hoàn tất!")
        
        print("\n📋 COPY CODE VÀO ESP32:")
        print("-" * 70)
        print(f"float X1 = {config.k1:.2f};  // P gain")
        print(f"float X2 = {config.k2:.2f};  // D gain")
        print(f"float X3 = {config.k3:.2f};  // Motor feedback")
        print("-" * 70)
        
        # Đánh giá
        if metrics.stability_score >= 80:
            print("\n🎉 XUẤT SẮC! Robot nên đứng rất ổn định!")
        elif metrics.stability_score >= 60:
            print("\n👍 TỐT! Robot có thể đứng được, nhưng có thể cần tinh chỉnh thêm")
        else:
            print("\n⚠️ KHẢ DỤNG. Robot có thể đứng nhưng chưa ổn định lắm")
            print("   → Kiểm tra lại phần cứng, offset, hoặc chạy lại với thời gian test dài hơn")
    
    else:
        print("\n❌ KHÔNG TÌM ĐƯỢC BỘ PID PHÙ HỢP!")
        print("\n🔧 Gợi ý khắc phục:")
        print("  1. Kiểm tra offset góc (robot có đứng thẳng ở 0° không?)")
        print("  2. Kiểm tra kết nối cảm biến MPU6050")
        print("  3. Kiểm tra motor có hoạt động đúng chiều không")
        print("  4. Thử tăng MAX_ANGLE lên 30° để test rộng hơn")
        print("  5. Giảm tốc độ motor hoặc kiểm tra nguồn")
    
    print("\n" + "=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Đã dừng bởi người dùng.")
    except Exception as e:
        print(f"\n❌ LỖI: {e}")
        import traceback
        traceback.print_exc()
