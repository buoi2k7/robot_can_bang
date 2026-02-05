"""
═══════════════════════════════════════════════════════════════
    REACTION WHEEL PID TUNER v2 - Advanced Edition
    Hỗ trợ: Kp, Kd, Ki, Kw với đồ thị real-time
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

# Lưu config
CONFIG_FILE = "pid_config.json"

# ═══════════════════════════════════════════════════════════════
# DỮ LIỆU TOÀN CỤC
# ═══════════════════════════════════════════════════════════════

# Circular buffers cho đồ thị
BUFFER_SIZE = 300
angle_buffer = collections.deque(maxlen=BUFFER_SIZE)
gyro_buffer = collections.deque(maxlen=BUFFER_SIZE)
pwm_buffer = collections.deque(maxlen=BUFFER_SIZE)
time_buffer = collections.deque(maxlen=BUFFER_SIZE)

# PID values
current_pid = {
    "Kp": 80.0,
    "Kd": 15.0,
    "Ki": 0.5,
    "Kw": 2.0
}

# Trạng thái
running = True
connected = False
last_data_time = 0
balance_state = False
auto_send_timer = None  # Timer for debounced auto-send

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
                current_pid = json.load(f)
                print(f"✅ Loaded config: {current_pid}")
        except:
            pass

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(current_pid, f, indent=2)
        print(f"💾 Saved config: {current_pid}")
    except Exception as e:
        print(f"❌ Save error: {e}")

# ═══════════════════════════════════════════════════════════════
# UDP COMMUNICATION
# ═══════════════════════════════════════════════════════════════

def send_pid():
    msg = f"Kp={current_pid['Kp']:.2f},Kd={current_pid['Kd']:.2f},Ki={current_pid['Ki']:.2f},Kw={current_pid['Kw']:.2f}"
    print(f"📤 Gửi: {msg}")
    try:
        sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
        save_config()
    except Exception as e:
        print(f"❌ Lỗi gửi: {e}")

def udp_listener():
    global running, connected, last_data_time, balance_state
    
    start_time = time.time()
    
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode().strip()
            
            # Parse: "A:angle,G:gyro,P:pwm,W:wheel,B:balance"
            if text.startswith("A:"):
                parts = text.split(',')
                try:
                    angle = float(parts[0].split(':')[1])
                    gyro = float(parts[1].split(':')[1])
                    pwm = int(parts[2].split(':')[1])
                    balance = int(parts[4].split(':')[1]) if len(parts) > 4 else 0
                    
                    current_time = time.time() - start_time
                    
                    angle_buffer.append(angle)
                    gyro_buffer.append(gyro)
                    pwm_buffer.append(pwm)
                    time_buffer.append(current_time)
                    
                    connected = True
                    last_data_time = time.time()
                    balance_state = balance == 1
                    
                except (IndexError, ValueError) as e:
                    pass
            elif text == "ACK":
                print("✅ ESP32 đã nhận PID mới!")
                
        except socket.timeout:
            if time.time() - last_data_time > 2:
                connected = False
        except Exception as e:
            print(f"❌ Lỗi nhận: {e}")

# ═══════════════════════════════════════════════════════════════
# GUI
# ═══════════════════════════════════════════════════════════════

class PIDTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Reaction Wheel PID Tuner v2")
        self.root.geometry("1000x700")
        self.root.configure(bg='#1a1a2e')
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#1a1a2e')
        style.configure('TLabel', background='#1a1a2e', foreground='white', font=('Segoe UI', 10))
        style.configure('Header.TLabel', font=('Segoe UI', 12, 'bold'))
        style.configure('TButton', font=('Segoe UI', 10, 'bold'))
        style.configure('TScale', background='#1a1a2e')
        
        self.create_widgets()
        self.update_plots()
        self.update_status()
        
    def create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill='both', expand=True)
        
        # === LEFT PANEL: PID Controls ===
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side='left', fill='y', padx=(0, 10))
        
        # Status indicator
        self.status_frame = tk.Frame(left_frame, bg='#1a1a2e')
        self.status_frame.pack(fill='x', pady=(0, 10))
        
        self.status_dot = tk.Canvas(self.status_frame, width=20, height=20, 
                                     bg='#1a1a2e', highlightthickness=0)
        self.status_dot.pack(side='left')
        self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
        
        self.status_label = ttk.Label(self.status_frame, text="Đang kết nối...")
        self.status_label.pack(side='left', padx=5)
        
        self.balance_label = ttk.Label(self.status_frame, text="", foreground='yellow')
        self.balance_label.pack(side='right')
        
        # PID Sliders
        self.sliders = {}
        pid_configs = [
            ("Kp", "🎯 Kp (Độ cứng góc)", 0, 200, 1),
            ("Kd", "🛑 Kd (Giảm chấn)", 0, 100, 0.5),
            ("Ki", "📊 Ki (Tích phân)", 0, 5, 0.1),
            ("Kw", "⚙️ Kw (Phanh bánh)", 0, 10, 0.1),
        ]
        
        for key, label, min_val, max_val, resolution in pid_configs:
            frame = tk.Frame(left_frame, bg='#16213e', padx=10, pady=8)
            frame.pack(fill='x', pady=5)
            
            header = tk.Frame(frame, bg='#16213e')
            header.pack(fill='x')
            
            tk.Label(header, text=label, bg='#16213e', fg='white', 
                    font=('Segoe UI', 10, 'bold')).pack(side='left')
            
            value_label = tk.Label(header, text=f"{current_pid[key]:.2f}", 
                                   bg='#16213e', fg='#00ff88', 
                                   font=('Consolas', 12, 'bold'))
            value_label.pack(side='right')
            
            slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=resolution,
                             orient='horizontal', length=250, bg='#16213e', fg='white',
                             highlightthickness=0, troughcolor='#0f3460',
                             activebackground='#00ff88', sliderrelief='flat')
            slider.set(current_pid[key])
            slider.pack(fill='x', pady=(5, 0))
            # INSTANT SEND: Gửi ngay khi kéo slider (không delay)
            slider.bind('<Motion>', lambda e, k=key, lbl=value_label, s=slider: 
                       self.on_slider_move_instant(k, lbl, s))
            
            self.sliders[key] = (slider, value_label)
        
        # Buttons
        btn_frame = tk.Frame(left_frame, bg='#1a1a2e')
        btn_frame.pack(fill='x', pady=15)
        
        send_btn = tk.Button(btn_frame, text="📤 GỬI XUỐNG ROBOT", 
                            bg='#00ff88', fg='black',
                            font=('Segoe UI', 11, 'bold'),
                            command=self.on_send_click,
                            cursor='hand2')
        send_btn.pack(fill='x', pady=5)
        
        reset_btn = tk.Button(btn_frame, text="🔄 Reset về mặc định",
                             bg='#ff6b6b', fg='white',
                             font=('Segoe UI', 10),
                             command=self.on_reset_click,
                             cursor='hand2')
        reset_btn.pack(fill='x')
        
        # Quick presets
        preset_frame = tk.LabelFrame(left_frame, text="⚡ Quick Presets", 
                                     bg='#1a1a2e', fg='white',
                                     font=('Segoe UI', 9, 'bold'))
        preset_frame.pack(fill='x', pady=10)
        
        presets = [
            ("Mềm", {"Kp": 40, "Kd": 10, "Ki": 0.2, "Kw": 1}),
            ("Vừa", {"Kp": 80, "Kd": 15, "Ki": 0.5, "Kw": 2}),
            ("Cứng", {"Kp": 120, "Kd": 25, "Ki": 1.0, "Kw": 3}),
        ]
        
        for name, values in presets:
            btn = tk.Button(preset_frame, text=name, bg='#0f3460', fg='white',
                           command=lambda v=values: self.apply_preset(v))
            btn.pack(side='left', expand=True, fill='x', padx=2, pady=5)
        
        # === RIGHT PANEL: Plots ===
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side='right', fill='both', expand=True)
        
        # Angle plot
        angle_frame = tk.LabelFrame(right_frame, text="📐 Góc Robot (°)", 
                                    bg='#1a1a2e', fg='white',
                                    font=('Segoe UI', 10, 'bold'))
        angle_frame.pack(fill='both', expand=True, pady=(0, 5))
        
        self.angle_canvas = tk.Canvas(angle_frame, bg='#0d1b2a', highlightthickness=0)
        self.angle_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # PWM plot
        pwm_frame = tk.LabelFrame(right_frame, text="⚡ PWM Output", 
                                   bg='#1a1a2e', fg='white',
                                   font=('Segoe UI', 10, 'bold'))
        pwm_frame.pack(fill='both', expand=True)
        
        self.pwm_canvas = tk.Canvas(pwm_frame, bg='#0d1b2a', highlightthickness=0)
        self.pwm_canvas.pack(fill='both', expand=True, padx=5, pady=5)
    
    def on_slider_move_instant(self, key, label, slider):
        """INSTANT SEND: Update value and send immediately"""
        value = slider.get()
        current_pid[key] = value
        label.config(text=f"{value:.2f}")
        
        # Gửi ngay lập tức (không delay)
        send_pid()
        
    def on_send_click(self):
        for key, (slider, _) in self.sliders.items():
            current_pid[key] = slider.get()
        send_pid()
        
    def on_reset_click(self):
        defaults = {"Kp": 80.0, "Kd": 15.0, "Ki": 0.5, "Kw": 2.0}
        for key, value in defaults.items():
            current_pid[key] = value
            slider, label = self.sliders[key]
            slider.set(value)
            label.config(text=f"{value:.2f}")
            
    def apply_preset(self, values):
        for key, value in values.items():
            current_pid[key] = value
            slider, label = self.sliders[key]
            slider.set(value)
            label.config(text=f"{value:.2f}")
        send_pid()
        
    def update_status(self):
        if not running:
            return
            
        if connected:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='#00ff88', outline='')
            self.status_label.config(text=f"Đã kết nối ({ESP_IP})")
            
            if balance_state:
                self.balance_label.config(text="🟢 ĐANG CÂN BẰNG", foreground='#00ff88')
            else:
                self.balance_label.config(text="🔴 CHỜ / NGÃ", foreground='#ff6b6b')
        else:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
            self.status_label.config(text="Mất kết nối...")
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
        
        # Grid lines
        for i in range(-20, 21, 10):
            y = cy - (i * h / 60)
            color = '#2a4a6a' if i != 0 else '#4a6a8a'
            self.angle_canvas.create_line(0, y, w, y, fill=color, dash=(2, 4))
            if i != 0:
                self.angle_canvas.create_text(25, y, text=f"{i}°", fill='#6a8aaa', font=('Consolas', 8))
        
        # Data
        points = list(angle_buffer)
        if len(points) < 2:
            self.angle_canvas.create_text(w/2, h/2, text="Đang chờ dữ liệu...", 
                                          fill='#4a6a8a', font=('Segoe UI', 12))
            return
            
        y_scale = h / 60  # ±30 degrees
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, angle in enumerate(points):
            x = i * x_step
            y = cy - (angle * y_scale)
            y = max(5, min(h-5, y))
            coords.extend([x, y])
            
        if len(coords) >= 4:
            self.angle_canvas.create_line(coords, fill='#00d4ff', width=2, smooth=True)
            
        # Current value
        current_angle = points[-1]
        color = '#00ff88' if abs(current_angle) < 5 else '#ff6b6b'
        self.angle_canvas.create_text(w - 50, 20, text=f"{current_angle:.1f}°", 
                                      fill=color, font=('Consolas', 14, 'bold'))
        
    def draw_pwm_plot(self):
        self.pwm_canvas.delete('all')
        w = self.pwm_canvas.winfo_width()
        h = self.pwm_canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
            
        cy = h / 2
        
        # Grid
        self.pwm_canvas.create_line(0, cy, w, cy, fill='#4a6a8a', dash=(2, 4))
        
        points = list(pwm_buffer)
        if len(points) < 2:
            return
            
        y_scale = h / 600  # ±300 PWM
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, pwm_val in enumerate(points):
            x = i * x_step
            y = cy - (pwm_val * y_scale)
            y = max(5, min(h-5, y))
            coords.extend([x, y])
            
        if len(coords) >= 4:
            # Gradient effect: positive = green, negative = red
            self.pwm_canvas.create_line(coords, fill='#ff9f43', width=2, smooth=True)
            
        # Current value
        current_pwm = points[-1]
        self.pwm_canvas.create_text(w - 50, 20, text=f"{current_pwm}", 
                                    fill='#ff9f43', font=('Consolas', 14, 'bold'))

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    load_config()
    
    # Start UDP listener thread
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
