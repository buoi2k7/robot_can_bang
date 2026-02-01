#!/usr/bin/env python3
"""
PID COMPARISON & VALIDATION TOOL
=================================
So sánh nhiều bộ PID và validate kết quả

Dùng để:
1. So sánh config cũ vs mới
2. Validate độ tin cậy của config
3. Test trong nhiều điều kiện khác nhau
"""

import socket
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Optional
import json
from datetime import datetime

# Config
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
ESP32_IP = "192.168.1.7"
ESP32_PORT = 4210

MAX_ANGLE = 25.0
TEST_DURATION = 5.0
NUM_TESTS_PER_CONFIG = 5  # Số lần test mỗi config để lấy TB

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

@dataclass
class PIDConfig:
    name: str
    k1: float
    k2: float
    k3: float
    
    def __str__(self):
        return f"{self.name}: K1={self.k1:.2f}, K2={self.k2:.2f}, K3={self.k3:.2f}"

@dataclass
class TestResult:
    success: bool
    avg_error: float
    max_error: float
    std_error: float
    duration: float
    
def send_gains(config: PIDConfig):
    msg = f"K1={config.k1:.2f},K2={config.k2:.2f},K3={config.k3:.2f}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    print(f"  📤 {msg}")
    time.sleep(0.6)

def clear_buffer():
    try:
        while True:
            sock.recv(1024)
    except:
        pass

def test_config(config: PIDConfig, test_num: int) -> Optional[TestResult]:
    """Test một config và return kết quả"""
    print(f"  [{test_num}] Testing: {config}")
    
    send_gains(config)
    time.sleep(1.0)
    clear_buffer()
    
    angles = []
    start_time = time.time()
    failed = False
    
    while time.time() - start_time < TEST_DURATION:
        try:
            data, _ = sock.recvfrom(1024)
            decoded = data.decode().strip()
            
            if decoded.startswith("K") or decoded.startswith("KACK"):
                continue
            
            parts = decoded.split(',')
            if len(parts) >= 3:
                angle = float(parts[2])
            else:
                angle = float(parts[0])
            
            if abs(angle) > MAX_ANGLE:
                print(f"    ❌ FELL at {len(angles)} samples")
                failed = True
                break
            
            angles.append(abs(angle))
            
        except socket.timeout:
            pass
        except:
            pass
    
    elapsed = time.time() - start_time
    
    if failed or len(angles) < 20:
        print(f"    ❌ Failed")
        return None
    
    result = TestResult(
        success=True,
        avg_error=np.mean(angles),
        max_error=np.max(angles),
        std_error=np.std(angles),
        duration=elapsed
    )
    
    print(f"    ✅ Avg: {result.avg_error:.2f}°, Max: {result.max_error:.2f}°, Std: {result.std_error:.2f}°")
    return result

def compare_configs(configs: List[PIDConfig]):
    """So sánh nhiều config"""
    print("\n" + "=" * 80)
    print("  🔬 PID COMPARISON TEST")
    print("=" * 80)
    print(f"📊 Configs to test: {len(configs)}")
    print(f"📊 Tests per config: {NUM_TESTS_PER_CONFIG}")
    print(f"📊 Total tests: {len(configs) * NUM_TESTS_PER_CONFIG}")
    print("=" * 80)
    
    input("\n👉 Đặt robot thẳng, nhấn ENTER để bắt đầu...")
    
    all_results = {}
    
    for config in configs:
        print(f"\n{'=' * 80}")
        print(f"Testing: {config}")
        print('=' * 80)
        
        results = []
        for i in range(NUM_TESTS_PER_CONFIG):
            result = test_config(config, i + 1)
            if result:
                results.append(result)
            time.sleep(1.0)  # Nghỉ giữa các lần test
        
        all_results[config.name] = results
        
        # Tính statistics
        if results:
            avg_errors = [r.avg_error for r in results]
            max_errors = [r.max_error for r in results]
            std_errors = [r.std_error for r in results]
            
            print(f"\n  📊 Summary for {config.name}:")
            print(f"    Success rate: {len(results)}/{NUM_TESTS_PER_CONFIG} ({len(results)/NUM_TESTS_PER_CONFIG*100:.1f}%)")
            print(f"    Avg error: {np.mean(avg_errors):.2f}° (±{np.std(avg_errors):.2f}°)")
            print(f"    Max error: {np.mean(max_errors):.2f}° (±{np.std(max_errors):.2f}°)")
            print(f"    Std error: {np.mean(std_errors):.2f}° (±{np.std(std_errors):.2f}°)")
        else:
            print(f"\n  ❌ {config.name} FAILED all tests")
    
    # Final comparison
    print("\n\n" + "=" * 80)
    print("  🏆 FINAL COMPARISON")
    print("=" * 80)
    
    # Create comparison table
    print(f"\n{'Config':<20} {'Success %':<12} {'Avg Error':<15} {'Max Error':<15} {'Consistency':<12}")
    print("-" * 80)
    
    rankings = []
    
    for config_name, results in all_results.items():
        if not results:
            print(f"{config_name:<20} {'0%':<12} {'N/A':<15} {'N/A':<15} {'N/A':<12}")
            continue
        
        success_rate = len(results) / NUM_TESTS_PER_CONFIG * 100
        avg_errors = [r.avg_error for r in results]
        max_errors = [r.max_error for r in results]
        
        avg_mean = np.mean(avg_errors)
        avg_std = np.std(avg_errors)
        max_mean = np.mean(max_errors)
        
        # Calculate overall score
        score = 100
        score -= min(avg_mean * 10, 40)  # Penalty for avg error
        score -= min(max_mean * 2, 30)   # Penalty for max error
        score -= min(avg_std * 20, 20)   # Penalty for inconsistency
        score -= (100 - success_rate)    # Penalty for failures
        score = max(score, 0)
        
        rankings.append((config_name, score, success_rate, avg_mean, max_mean, avg_std))
        
        consistency = "High" if avg_std < 0.5 else "Medium" if avg_std < 1.0 else "Low"
        
        print(f"{config_name:<20} {success_rate:>5.1f}%{'':<6} "
              f"{avg_mean:>5.2f}° ±{avg_std:.2f}{'':<5} "
              f"{max_mean:>5.2f}°{'':<8} {consistency:<12}")
    
    # Rankings
    rankings.sort(key=lambda x: x[1], reverse=True)
    
    print("\n" + "=" * 80)
    print("  🥇 RANKINGS (by overall score)")
    print("=" * 80)
    
    for rank, (name, score, success, avg, max_err, std) in enumerate(rankings, 1):
        medal = "🥇" if rank == 1 else "🥈" if rank == 2 else "🥉" if rank == 3 else f"{rank}."
        print(f"{medal} {name:<20} Score: {score:>5.1f}/100")
    
    # Best config
    if rankings:
        best_name = rankings[0][0]
        best_config = next(c for c in configs if c.name == best_name)
        
        print("\n" + "=" * 80)
        print("  ✅ RECOMMENDED CONFIG")
        print("=" * 80)
        print(f"\n{best_config}")
        print(f"\nESP32 Code:")
        print(f"float X1 = {best_config.k1:.2f};")
        print(f"float X2 = {best_config.k2:.2f};")
        print(f"float X3 = {best_config.k3:.2f};")
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"comparison_results_{timestamp}.json"
        
        results_data = {
            'timestamp': datetime.now().isoformat(),
            'configs': [
                {
                    'name': c.name,
                    'k1': c.k1,
                    'k2': c.k2,
                    'k3': c.k3
                } for c in configs
            ],
            'results': {
                name: [
                    {
                        'avg_error': r.avg_error,
                        'max_error': r.max_error,
                        'std_error': r.std_error,
                        'duration': r.duration
                    } for r in results
                ] for name, results in all_results.items()
            },
            'rankings': [
                {
                    'rank': i + 1,
                    'name': name,
                    'score': score,
                    'success_rate': success,
                    'avg_error': avg,
                    'max_error': max_err,
                    'consistency_std': std
                } for i, (name, score, success, avg, max_err, std) in enumerate(rankings)
            ],
            'recommended': {
                'name': best_name,
                'k1': best_config.k1,
                'k2': best_config.k2,
                'k3': best_config.k3
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(results_data, f, indent=2)
        
        print(f"\n💾 Saved results to: {filename}")

def main():
    print("\n" + "=" * 80)
    print("  🔬 PID COMPARISON & VALIDATION TOOL")
    print("=" * 80)
    
    # Predefined configs to test
    test_configs = [
        PIDConfig("Old Config", 60.0, 15.0, 1.5),
        PIDConfig("GA Result", 67.5, 18.2, 1.35),
        PIDConfig("Conservative", 55.0, 20.0, 1.0),
        PIDConfig("Aggressive", 75.0, 16.0, 1.8),
    ]
    
    print("\nDefault test configs:")
    for i, cfg in enumerate(test_configs, 1):
        print(f"  {i}. {cfg}")
    
    print("\nOptions:")
    print("  1. Test all default configs")
    print("  2. Add custom config")
    print("  3. Load configs from file")
    
    choice = input("\nChoice (1/2/3) [default=1]: ").strip() or "1"
    
    if choice == "2":
        print("\nEnter custom config:")
        name = input("  Name: ").strip()
        k1 = float(input("  K1: "))
        k2 = float(input("  K2: "))
        k3 = float(input("  K3: "))
        test_configs.append(PIDConfig(name, k1, k2, k3))
        print(f"\n✅ Added: {test_configs[-1]}")
    
    elif choice == "3":
        filename = input("  Filename: ").strip()
        try:
            with open(filename) as f:
                data = json.load(f)
                for cfg in data.get('configs', []):
                    test_configs.append(PIDConfig(
                        cfg['name'], cfg['k1'], cfg['k2'], cfg['k3']
                    ))
            print(f"\n✅ Loaded {len(test_configs)} configs")
        except Exception as e:
            print(f"❌ Error loading file: {e}")
            return
    
    compare_configs(test_configs)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user")
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
