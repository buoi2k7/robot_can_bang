#!/usr/bin/env python3
"""
GUI PID AUTO-TUNER với Real-time Visualization
=============================================
Theo dõi quá trình tìm kiếm PID với biểu đồ trực quan

Version: 2.0 GUI
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.animation import FuncAnimation
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple
import threading
import random

# ============== CẤU HÌNH ==============
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
ESP32_IP = "192.168.1.7"
ESP32_PORT = 4210

# Tuning configs
MAX_ANGLE = 25.0
TEST_DURATION = 4.0
POPULATION_SIZE = 8
GENERATIONS = 8

# PID Ranges
K1_RANGE = (30, 120)
K2_RANGE = (5, 40)
K3_RANGE = (0.1, 3.0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

# ============== DATA STRUCTURES ==============
@dataclass
class PIDConfig:
    k1: float
    k2: float
    k3: float
    
    def __str__(self):
        return f"K1={self.k1:.1f}, K2={self.k2:.1f}, K3={self.k3:.1f}"

@dataclass
class TestResult:
    config: PIDConfig
    score: float
    avg_error: float
    max_error: float
    generation: int

# ============== GLOBAL STATE ==============
class TunerState:
    def __init__(self):
        self.running = False
        self.current_config = None
        self.current_angles = deque(maxlen=200)
        self.all_results: List[TestResult] = []
        self.best_result: Optional[TestResult] = None
        self.current_gen = 0
        self.status = "Idle"
        self.lock = threading.Lock()

state = TunerState()

# ============== COMMUNICATION ==============
def send_gains(config: PIDConfig):
    msg = f"K1={config.k1:.2f},K2={config.k2:.2f},K3={config.k3:.2f}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    time.sleep(0.5)

def clear_buffer():
    try:
        while True:
            sock.recv(1024)
    except:
        pass

# ============== MEASUREMENT ==============
def measure_performance(config: PIDConfig) -> Optional[float]:
    """Đo hiệu suất, return score hoặc None nếu ngã"""
    with state.lock:
        state.current_config = config
        state.current_angles.clear()
        state.status = f"Testing {config}"
    
    send_gains(config)
    time.sleep(1.0)
    clear_buffer()
    
    angles = []
    start_time = time.time()
    
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
                return None  # Ngã
            
            angles.append(abs(angle))
            
            with state.lock:
                state.current_angles.append(angle)
        
        except socket.timeout:
            pass
        except:
            pass
    
    if len(angles) < 20:
        return None
    
    avg_error = np.mean(angles)
    max_error = np.max(angles)
    std_error = np.std(angles)
    
    # Calculate score
    score = 100 - min(avg_error * 8, 40) - min(std_error * 10, 30)
    score = max(score, 0)
    
    return score, avg_error, max_error

# ============== GENETIC ALGORITHM ==============
def random_config() -> PIDConfig:
    return PIDConfig(
        k1=random.uniform(*K1_RANGE),
        k2=random.uniform(*K2_RANGE),
        k3=random.uniform(*K3_RANGE)
    )

def crossover(p1: PIDConfig, p2: PIDConfig) -> PIDConfig:
    alpha = random.random()
    return PIDConfig(
        k1=alpha * p1.k1 + (1 - alpha) * p2.k1,
        k2=alpha * p1.k2 + (1 - alpha) * p2.k2,
        k3=alpha * p1.k3 + (1 - alpha) * p2.k3
    )

def mutate(config: PIDConfig, rate=0.3) -> PIDConfig:
    def mut(val, min_v, max_v):
        if random.random() < rate:
            delta = random.gauss(0, (max_v - min_v) * 0.1)
            return np.clip(val + delta, min_v, max_v)
        return val
    
    return PIDConfig(
        k1=mut(config.k1, *K1_RANGE),
        k2=mut(config.k2, *K2_RANGE),
        k3=mut(config.k3, *K3_RANGE)
    )

def run_genetic_tuner():
    """Chạy genetic algorithm trong thread riêng"""
    state.running = True
    
    # Initialize population
    seeds = [
        PIDConfig(60, 15, 1.5),
        PIDConfig(70, 20, 1.0),
        PIDConfig(50, 25, 1.2),
        PIDConfig(80, 18, 0.8),
    ]
    
    population = seeds[:min(4, POPULATION_SIZE)]
    while len(population) < POPULATION_SIZE:
        population.append(random_config())
    
    for gen in range(GENERATIONS):
        if not state.running:
            break
        
        with state.lock:
            state.current_gen = gen + 1
            state.status = f"Generation {gen + 1}/{GENERATIONS}"
        
        # Evaluate
        evaluated = []
        for idx, config in enumerate(population):
            if not state.running:
                break
            
            result = measure_performance(config)
            if result is None:
                score, avg_err, max_err = 0, 999, 999
            else:
                score, avg_err, max_err = result
            
            test_result = TestResult(
                config=config,
                score=score,
                avg_error=avg_err,
                max_error=max_err,
                generation=gen + 1
            )
            
            with state.lock:
                state.all_results.append(test_result)
                if state.best_result is None or score > state.best_result.score:
                    state.best_result = test_result
            
            evaluated.append((config, score))
        
        if not state.running:
            break
        
        # Sort
        evaluated.sort(key=lambda x: x[1], reverse=True)
        
        # Check early stop
        if evaluated[0][1] > 80:
            print(f"Found excellent solution at gen {gen+1}!")
            break
        
        # Evolve
        if gen < GENERATIONS - 1:
            elite = evaluated[:max(2, POPULATION_SIZE // 5)]
            new_pop = [c for c, _ in elite]
            
            parents_pool = evaluated[:max(4, POPULATION_SIZE // 2)]
            
            while len(new_pop) < POPULATION_SIZE:
                p1 = random.choice(parents_pool)[0]
                p2 = random.choice(parents_pool)[0]
                child = crossover(p1, p2)
                child = mutate(child)
                new_pop.append(child)
            
            population = new_pop
    
    with state.lock:
        state.status = "Completed!"
        state.running = False
        
        # Send best config
        if state.best_result:
            send_gains(state.best_result.config)

# ============== GUI ==============
class PIDTunerGUI:
    def __init__(self):
        self.fig = plt.figure(figsize=(14, 8))
        self.setup_layout()
        self.tuner_thread = None
        
    def setup_layout(self):
        # Layout: 2 rows, 3 columns
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3,
                                   left=0.08, right=0.95, top=0.93, bottom=0.08)
        
        # 1. Real-time angle plot
        self.ax_angle = self.fig.add_subplot(gs[0, :])
        self.line_angle, = self.ax_angle.plot([], [], 'b-', linewidth=1.5)
        self.ax_angle.set_xlim(0, 200)
        self.ax_angle.set_ylim(-20, 20)
        self.ax_angle.set_title("Current Test - Real-time Angle", fontweight='bold')
        self.ax_angle.set_ylabel("Angle (°)")
        self.ax_angle.grid(True, alpha=0.3)
        self.ax_angle.axhline(0, color='green', linestyle='--', alpha=0.5)
        self.ax_angle.axhline(MAX_ANGLE, color='red', linestyle='--', alpha=0.5, label='Danger')
        self.ax_angle.axhline(-MAX_ANGLE, color='red', linestyle='--', alpha=0.5)
        
        # 2. Score history
        self.ax_score = self.fig.add_subplot(gs[1, 0])
        self.ax_score.set_title("Score History", fontweight='bold')
        self.ax_score.set_xlabel("Test #")
        self.ax_score.set_ylabel("Score")
        self.ax_score.grid(True, alpha=0.3)
        
        # 3. Best config evolution
        self.ax_evolution = self.fig.add_subplot(gs[1, 1])
        self.ax_evolution.set_title("Best Score per Generation", fontweight='bold')
        self.ax_evolution.set_xlabel("Generation")
        self.ax_evolution.set_ylabel("Score")
        self.ax_evolution.grid(True, alpha=0.3)
        
        # 4. PID parameters scatter
        self.ax_params = self.fig.add_subplot(gs[1, 2], projection='3d')
        self.ax_params.set_title("PID Space Exploration", fontweight='bold')
        self.ax_params.set_xlabel("K1")
        self.ax_params.set_ylabel("K2")
        self.ax_params.set_zlabel("K3")
        
        # 5. Status and results
        self.ax_status = self.fig.add_subplot(gs[2, :])
        self.ax_status.axis('off')
        self.text_status = self.ax_status.text(0.05, 0.9, "", fontsize=10,
                                               verticalalignment='top', family='monospace')
        
        # Buttons
        ax_start = plt.axes([0.3, 0.01, 0.15, 0.04])
        ax_stop = plt.axes([0.5, 0.01, 0.15, 0.04])
        
        self.btn_start = Button(ax_start, 'Start Tuning', color='lightgreen')
        self.btn_stop = Button(ax_stop, 'Stop', color='lightcoral')
        
        self.btn_start.on_clicked(self.start_tuning)
        self.btn_stop.on_clicked(self.stop_tuning)
        
    def start_tuning(self, event):
        if not state.running:
            print("\n🚀 Starting auto-tuner...")
            state.all_results.clear()
            state.best_result = None
            self.tuner_thread = threading.Thread(target=run_genetic_tuner, daemon=True)
            self.tuner_thread.start()
    
    def stop_tuning(self, event):
        if state.running:
            print("\n🛑 Stopping...")
            state.running = False
    
    def update(self, frame):
        with state.lock:
            # Update angle plot
            if len(state.current_angles) > 0:
                self.line_angle.set_data(range(len(state.current_angles)), 
                                        list(state.current_angles))
            
            # Update score history
            if len(state.all_results) > 0:
                self.ax_score.clear()
                scores = [r.score for r in state.all_results]
                self.ax_score.plot(scores, 'o-', markersize=4)
                self.ax_score.set_title("Score History", fontweight='bold')
                self.ax_score.set_xlabel("Test #")
                self.ax_score.set_ylabel("Score")
                self.ax_score.grid(True, alpha=0.3)
                
                # Best score per generation
                self.ax_evolution.clear()
                gen_best = {}
                for r in state.all_results:
                    if r.generation not in gen_best:
                        gen_best[r.generation] = r.score
                    else:
                        gen_best[r.generation] = max(gen_best[r.generation], r.score)
                
                if gen_best:
                    gens = sorted(gen_best.keys())
                    scores = [gen_best[g] for g in gens]
                    self.ax_evolution.plot(gens, scores, 'o-', color='green', markersize=8)
                    self.ax_evolution.set_title("Best Score per Generation", fontweight='bold')
                    self.ax_evolution.set_xlabel("Generation")
                    self.ax_evolution.set_ylabel("Score")
                    self.ax_evolution.grid(True, alpha=0.3)
                
                # 3D scatter of all tested configs
                self.ax_params.clear()
                k1s = [r.config.k1 for r in state.all_results]
                k2s = [r.config.k2 for r in state.all_results]
                k3s = [r.config.k3 for r in state.all_results]
                scores = [r.score for r in state.all_results]
                
                scatter = self.ax_params.scatter(k1s, k2s, k3s, c=scores, 
                                                cmap='RdYlGn', s=50, alpha=0.6)
                
                # Mark best
                if state.best_result:
                    self.ax_params.scatter([state.best_result.config.k1],
                                          [state.best_result.config.k2],
                                          [state.best_result.config.k3],
                                          c='red', s=200, marker='*', 
                                          edgecolors='black', linewidths=2)
                
                self.ax_params.set_title("PID Space Exploration", fontweight='bold')
                self.ax_params.set_xlabel("K1")
                self.ax_params.set_ylabel("K2")
                self.ax_params.set_zlabel("K3")
            
            # Update status text
            status_text = f"Status: {state.status}\n"
            status_text += f"Generation: {state.current_gen}/{GENERATIONS}\n"
            
            if state.current_config:
                status_text += f"Current Test: {state.current_config}\n"
            
            if state.best_result:
                status_text += "\n" + "=" * 60 + "\n"
                status_text += "🏆 BEST RESULT SO FAR:\n"
                status_text += f"   Config: {state.best_result.config}\n"
                status_text += f"   Score: {state.best_result.score:.1f}/100\n"
                status_text += f"   Avg Error: {state.best_result.avg_error:.2f}°\n"
                status_text += f"   Max Error: {state.best_result.max_error:.2f}°\n"
                status_text += "=" * 60
            
            self.text_status.set_text(status_text)
        
        return self.line_angle,
    
    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=100, 
                                blit=False, cache_frame_data=False)
        plt.show()

# ============== MAIN ==============
if __name__ == "__main__":
    print("=" * 70)
    print("  🎨 GUI PID AUTO-TUNER v2.0")
    print("=" * 70)
    print("📊 Mở giao diện đồ thị...")
    print("📌 Click 'Start Tuning' để bắt đầu")
    print("📌 Robot cần được đặt thẳng đứng trước khi start")
    print("=" * 70)
    
    gui = PIDTunerGUI()
    gui.run()
