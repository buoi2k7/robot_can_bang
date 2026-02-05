"""
═══════════════════════════════════════════════════════════════
    REACTION WHEEL PID TUNER v4 - Simple Edition
    ✨ Giao diện đơn giản như V2, tương thích V4 firmware
═══════════════════════════════════════════════════════════════
"""

import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import collections
import json
import os

# ═══════════════════════════════════════════════════════════════
# CẤU HÌNH
# ═══════════════════════════════════════════════════════════════

UDP_IP_LISTEN = "0.0.0.0"
UDP_PORT_LISTEN = 4210
ESP_IP = "192.168.1.7"  # IP của ESP32
ESP_PORT = 4210

CONFIG_FILE = "pid_config_v4.json"

# ═══════════════════════════════════════════════════════════════
# DỮ LIỆU TOÀN CỤC
# ═══════════════════════════════════════════════════════════════

BUFFER_SIZE = 300
angle_buffer = collections.deque(maxlen=BUFFER_SIZE)
gyro_buffer = collections.deque(maxlen=BUFFER_SIZE)
pwm_buffer = collections.deque(maxlen=BUFFER_SIZE)
time_buffer = collections.deque(maxlen=BUFFER_SIZE)

# PID values - Giống V4 firmware
current_pid = {
    "Kp": 55.0,
    "Kd": 22.0,
    "Ki": 0.0,
    "Kw": 1.2
}

# Trạng thái
running = True
connected = False
last_data_time = 0
balance_state = False
current_offset = 0.0

# Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP_LISTEN, UDP_PORT_LISTEN))
sock.settimeout(0.1)

# ═══════════════════════════════════════════════════════════════
# LOAD/SAVE CONFIG
# ═══════════════════════════════════════════════════════════════

def load_config():
    global current_pid
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                loaded = json.load(f)
                for key in current_pid:
                    if key in loaded:
                        current_pid[key] = loaded[key]
                print(f"✅ Loaded: {current_pid}")
        except:
            pass

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(current_pid, f, indent=2)
    except:
        pass

# ═══════════════════════════════════════════════════════════════
# UDP COMMUNICATION
# ═══════════════════════════════════════════════════════════════

def send_pid():
    """Format V4: Kp=55.0,Kd=22.0,Ki=0.0,Kw=1.2"""
    msg = f"Kp={current_pid['Kp']:.2f},Kd={current_pid['Kd']:.2f},Ki={current_pid['Ki']:.2f},Kw={current_pid['Kw']:.2f}"
    print(f"📤 Gửi: {msg}")
    try:
        sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
        save_config()
    except Exception as e:
        print(f"❌ Lỗi gửi: {e}")

def send_command(cmd):
    try:
        sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))
        print(f"📤 Command: {cmd}")
    except:
        pass

def udp_listener():
    global running, connected, last_data_time, balance_state, current_offset
    
    start_time = time.time()
    
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode().strip()
            
            # Format V4: "A:angle,G:gyro,P:pwm,W:wheel,I:integral,B:balance,O:offset"
            if text.startswith("A:"):
                parts = text.split(',')
                try:
                    angle = float(parts[0].split(':')[1])
                    gyro = float(parts[1].split(':')[1])
                    pwm = int(parts[2].split(':')[1])
                    balance = int(parts[5].split(':')[1]) if len(parts) > 5 else 0
                    offset = float(parts[6].split(':')[1]) if len(parts) > 6 else 0
                    
                    current_time = time.time() - start_time
                    
                    angle_buffer.append(angle)
                    gyro_buffer.append(gyro)
                    pwm_buffer.append(pwm)
                    time_buffer.append(current_time)
                    
                    connected = True
                    last_data_time = time.time()
                    balance_state = balance == 1
                    current_offset = offset
                    
                except (IndexError, ValueError):
                    pass
            elif text == "ACK":
                print("✅ ESP32 đã nhận PID mới!")
            elif text == "RESET_OK":
                print("✅ ESP32 đã reset!")
                
        except socket.timeout:
            if time.time() - last_data_time > 2:
                connected = False
        except:
            pass

# ═══════════════════════════════════════════════════════════════
# GUI - ĐƠN GIẢN NHƯ V2
# ═══════════════════════════════════════════════════════════════

class PIDTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Reaction Wheel PID Tuner V4")
        self.root.geometry("1000x700")
        self.root.configure(bg='#1a1a2e')
        
        self.create_widgets()
        self.update_plots()
        self.update_status()
        
    def create_widgets(self):
        main_frame = tk.Frame(self.root, bg='#1a1a2e', padx=10, pady=10)
        main_frame.pack(fill='both', expand=True)
        
        # === LEFT PANEL: Controls ===
        left_frame = tk.Frame(main_frame, bg='#1a1a2e')
        left_frame.pack(side='left', fill='y', padx=(0, 10))
        
        # Status
        self.status_frame = tk.Frame(left_frame, bg='#1a1a2e')
        self.status_frame.pack(fill='x', pady=(0, 10))
        
        self.status_dot = tk.Canvas(self.status_frame, width=20, height=20, 
                                     bg='#1a1a2e', highlightthickness=0)
        self.status_dot.pack(side='left')
        self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
        
        self.status_label = tk.Label(self.status_frame, text="Đang kết nối...",
                                     bg='#1a1a2e', fg='white', font=('Segoe UI', 10))
        self.status_label.pack(side='left', padx=5)
        
        self.balance_label = tk.Label(self.status_frame, text="", 
                                      bg='#1a1a2e', fg='yellow', font=('Segoe UI', 10, 'bold'))
        self.balance_label.pack(side='right')
        
        # Offset display
        self.offset_label = tk.Label(left_frame, text="Offset: 0.00°", 
                                     bg='#1a1a2e', fg='#888', font=('Consolas', 9))
        self.offset_label.pack(anchor='w', pady=(0, 10))
        
        # === PID SLIDERS ===
        self.sliders = {}
        pid_configs = [
            ("Kp", "🎯 Kp (Độ cứng)", 0, 150, 1, "#ff7b72"),
            ("Kd", "🛑 Kd (Giảm rung)", 0, 50, 0.5, "#79c0ff"),
            ("Ki", "📊 Ki (Chống drift)", 0, 5, 0.1, "#56d364"),
            ("Kw", "⚙️ Kw (Phanh bánh)", 0, 5, 0.1, "#d2a8ff"),
        ]
        
        for key, label, min_val, max_val, resolution, color in pid_configs:
            frame = tk.Frame(left_frame, bg='#16213e', padx=10, pady=8)
            frame.pack(fill='x', pady=5)
            
            header = tk.Frame(frame, bg='#16213e')
            header.pack(fill='x')
            
            tk.Label(header, text=label, bg='#16213e', fg='white', 
                    font=('Segoe UI', 10, 'bold')).pack(side='left')
            
            value_label = tk.Label(header, text=f"{current_pid[key]:.2f}", 
                                   bg='#16213e', fg=color, 
                                   font=('Consolas', 12, 'bold'))
            value_label.pack(side='right')
            
            slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=resolution,
                             orient='horizontal', length=250, bg='#16213e', fg='white',
                             highlightthickness=0, troughcolor='#0f3460',
                             activebackground=color, sliderrelief='flat',
                             showvalue=False)
            slider.set(current_pid[key])
            slider.pack(fill='x', pady=(5, 0))
            
            # Bind để cập nhật value label
            slider.config(command=lambda v, k=key, lbl=value_label: self.on_slider_change(k, v, lbl))
            
            self.sliders[key] = (slider, value_label)
        
        # === BUTTONS ===
        btn_frame = tk.Frame(left_frame, bg='#1a1a2e')
        btn_frame.pack(fill='x', pady=15)
        
        send_btn = tk.Button(btn_frame, text="📤 GỬI XUỐNG ROBOT", 
                            bg='#238636', fg='white',
                            font=('Segoe UI', 11, 'bold'),
                            command=self.on_send_click,
                            cursor='hand2', relief='flat')
        send_btn.pack(fill='x', pady=5)
        
        reset_btn = tk.Button(btn_frame, text="🔄 Reset ESP32",
                             bg='#da3633', fg='white',
                             font=('Segoe UI', 10),
                             command=lambda: send_command("RESET"),
                             cursor='hand2', relief='flat')
        reset_btn.pack(fill='x', pady=2)
        
        clear_btn = tk.Button(btn_frame, text="🧹 Xóa đồ thị",
                             bg='#484f58', fg='white',
                             font=('Segoe UI', 10),
                             command=self.clear_buffers,
                             cursor='hand2', relief='flat')
        clear_btn.pack(fill='x', pady=2)
        
        # === QUICK PRESETS ===
        preset_frame = tk.LabelFrame(left_frame, text="⚡ Quick Presets", 
                                     bg='#1a1a2e', fg='white',
                                     font=('Segoe UI', 9, 'bold'))
        preset_frame.pack(fill='x', pady=10)
        
        presets = [
            ("🔹 Soft", {"Kp": 40, "Kd": 15, "Ki": 0.0, "Kw": 1.0}),
            ("🔸 Medium", {"Kp": 55, "Kd": 22, "Ki": 0.0, "Kw": 1.2}),
            ("🔥 Hard", {"Kp": 70, "Kd": 28, "Ki": 0.3, "Kw": 1.5}),
        ]
        
        for name, values in presets:
            btn = tk.Button(preset_frame, text=name, bg='#21262d', fg='white',
                           font=('Segoe UI', 9),
                           command=lambda v=values: self.apply_preset(v),
                           relief='flat', cursor='hand2')
            btn.pack(side='left', expand=True, fill='x', padx=2, pady=5)
        
        # === TIPS ===
        tips_frame = tk.Frame(left_frame, bg='#0d1117')
        tips_frame.pack(fill='x', pady=10)
        
        tk.Label(tips_frame, text="💡 Tips:", bg='#0d1117', fg='#8b949e', 
                font=('Segoe UI', 9, 'bold')).pack(anchor='w')
        tips = [
            "• Rung → tăng Kd hoặc giảm Kp",
            "• Drift → tăng Ki nhẹ (0.1-0.3)",
            "• Yếu → tăng Kp",
            "• Motor nóng → giảm Kp"
        ]
        for tip in tips:
            tk.Label(tips_frame, text=tip, bg='#0d1117', fg='#6e7681', 
                    font=('Segoe UI', 8)).pack(anchor='w')
        
        # === RIGHT PANEL: Plots ===
        right_frame = tk.Frame(main_frame, bg='#1a1a2e')
        right_frame.pack(side='right', fill='both', expand=True)
        
        # Angle plot
        angle_frame = tk.LabelFrame(right_frame, text="📐 Góc Robot (°)", 
                                    bg='#1a1a2e', fg='#58a6ff',
                                    font=('Segoe UI', 10, 'bold'))
        angle_frame.pack(fill='both', expand=True, pady=(0, 5))
        
        self.angle_canvas = tk.Canvas(angle_frame, bg='#0d1117', highlightthickness=0)
        self.angle_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # PWM plot
        pwm_frame = tk.LabelFrame(right_frame, text="⚡ PWM Output", 
                                   bg='#1a1a2e', fg='#f0883e',
                                   font=('Segoe UI', 10, 'bold'))
        pwm_frame.pack(fill='both', expand=True)
        
        self.pwm_canvas = tk.Canvas(pwm_frame, bg='#0d1117', highlightthickness=0)
        self.pwm_canvas.pack(fill='both', expand=True, padx=5, pady=5)
    
    def on_slider_change(self, key, value, label):
        val = float(value)
        current_pid[key] = val
        label.config(text=f"{val:.2f}")
    
    def on_send_click(self):
        for key, (slider, _) in self.sliders.items():
            current_pid[key] = slider.get()
        send_pid()
        
    def clear_buffers(self):
        angle_buffer.clear()
        gyro_buffer.clear()
        pwm_buffer.clear()
        time_buffer.clear()
        
    def apply_preset(self, values):
        for key, value in values.items():
            current_pid[key] = value
            if key in self.sliders:
                slider, label = self.sliders[key]
                slider.set(value)
                label.config(text=f"{value:.2f}")
        send_pid()
        
    def update_status(self):
        if not running:
            return
            
        if connected:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='#3fb950', outline='')
            self.status_label.config(text=f"✅ Đã kết nối ({ESP_IP})", fg='#3fb950')
            
            if balance_state:
                self.balance_label.config(text="🟢 CÂN BẰNG", fg='#3fb950')
            else:
                self.balance_label.config(text="🔴 CHỜ/NGÃ", fg='#f85149')
            
            self.offset_label.config(text=f"Offset: {current_offset:.2f}°")
        else:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
            self.status_label.config(text="❌ Mất kết nối...", fg='red')
            self.balance_label.config(text="")
            
        self.root.after(500, self.update_status)
        
    def update_plots(self):
        if not running:
            return
            
        self.draw_angle_plot()
        self.draw_pwm_plot()
        
        self.root.after(50, self.update_plots)
        
    def draw_angle_plot(self):
        self.angle_canvas.delete('all')
        w = self.angle_canvas.winfo_width()
        h = self.angle_canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
            
        cy = h / 2
        
        # Grid lines (±30 degrees)
        for i in range(-30, 31, 10):
            y = cy - (i * h / 80)
            if 0 <= y <= h:
                color = '#30363d' if i != 0 else '#58a6ff'
                width = 2 if i == 0 else 1
                self.angle_canvas.create_line(0, y, w, y, fill=color, dash=(2, 4), width=width)
                if i % 20 == 0:
                    self.angle_canvas.create_text(25, y, text=f"{i}°", 
                                                 fill='#6e7681', font=('Consolas', 9))
        
        points = list(angle_buffer)
        if len(points) < 2:
            self.angle_canvas.create_text(w/2, h/2, text="Đang chờ dữ liệu...", 
                                          fill='#484f58', font=('Segoe UI', 11))
            return
        
        y_scale = h / 80
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, angle in enumerate(points):
            x = i * x_step
            y = cy - (angle * y_scale)
            y = max(0, min(h, y))
            coords.extend([x, y])
        
        if len(coords) >= 4:
            self.angle_canvas.create_line(coords, fill='#58a6ff', width=2, smooth=True)
        
        current = points[-1]
        color = '#3fb950' if abs(current) < 3 else '#f85149'
        self.angle_canvas.create_text(w - 50, 25, text=f"{current:.2f}°", 
                                      fill=color, font=('Consolas', 16, 'bold'))
        
    def draw_pwm_plot(self):
        self.pwm_canvas.delete('all')
        w = self.pwm_canvas.winfo_width()
        h = self.pwm_canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
        
        cy = h / 2
        
        # Grid (±300 PWM)
        for i in [-300, -150, 0, 150, 300]:
            y = cy - (i * h / 700)
            if 0 <= y <= h:
                color = '#58a6ff' if i == 0 else '#30363d'
                width = 2 if i == 0 else 1
                self.pwm_canvas.create_line(0, y, w, y, fill=color, dash=(2, 4), width=width)
        
        points = list(pwm_buffer)
        if len(points) < 2:
            return
        
        y_scale = h / 700
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, pwm_val in enumerate(points):
            x = i * x_step
            y = cy - (pwm_val * y_scale)
            y = max(0, min(h, y))
            coords.extend([x, y])
        
        if len(coords) >= 4:
            self.pwm_canvas.create_line(coords, fill='#f0883e', width=2, smooth=True)
        
        current = points[-1]
        self.pwm_canvas.create_text(w - 50, 25, text=f"{current}", 
                                    fill='#f0883e', font=('Consolas', 16, 'bold'))

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("═══════════════════════════════════════════════════════════════")
    print("  🎛️ Reaction Wheel PID Tuner V4 - Simple Edition")
    print("═══════════════════════════════════════════════════════════════")
    
    load_config()
    
    # Start UDP listener
    listener_thread = threading.Thread(target=udp_listener, daemon=True)
    listener_thread.start()
    
    # Start GUI
    root = tk.Tk()
    app = PIDTunerGUI(root)
    
    try:
        root.mainloop()
    finally:
        running = False
        sock.close()
        print("👋 Đã thoát!")
