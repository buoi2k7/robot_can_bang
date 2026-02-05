"""
════════════════════════════════════════════════════════════════════════════════
    REACTION WHEEL PID TUNER v3 - IMPROVED UI EDITION
    ✨ Giao diện dễ nhìn, dễ điều chỉnh, biểu đồ ổn định
════════════════════════════════════════════════════════════════════════════════
"""

import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import collections
import json
import os
import math
from datetime import datetime

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

UDP_IP_LISTEN = "0.0.0.0"
UDP_PORT_LISTEN = 4210
ESP_IP = "192.168.1.7"
ESP_PORT = 4210

CONFIG_FILE = "pid_config_v3.json"
LOG_FILE = "balance_log.csv"

# ═══════════════════════════════════════════════════════════════════════════════
# GLOBAL DATA
# ═══════════════════════════════════════════════════════════════════════════════

BUFFER_SIZE = 400
angle_buffer = collections.deque(maxlen=BUFFER_SIZE)
gyro_buffer = collections.deque(maxlen=BUFFER_SIZE)
pwm_buffer = collections.deque(maxlen=BUFFER_SIZE)
integral_buffer = collections.deque(maxlen=BUFFER_SIZE)
time_buffer = collections.deque(maxlen=BUFFER_SIZE)

# PID values
pid_values = {
    "Kp": 75.0,
    "Ki": 0.8,
    "Kd": 25.0,
    "Kw": 2.5,
    "Kff": 0.15
}

# State
running = True
connected = False
last_data_time = 0
balance_state = False
logging_enabled = False

# Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP_LISTEN, UDP_PORT_LISTEN))
sock.settimeout(0.1)

# ═══════════════════════════════════════════════════════════════════════════════
# LOAD/SAVE CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

def load_config():
    global pid_values
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                loaded = json.load(f)
                pid_values.update(loaded)
                print(f"✅ Loaded config: {pid_values}")
        except Exception as e:
            print(f"⚠️ Load error: {e}")

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(pid_values, f, indent=2)
        print(f"💾 Saved config")
    except Exception as e:
        print(f"❌ Save error: {e}")

# ═══════════════════════════════════════════════════════════════════════════════
# UDP COMMUNICATION
# ═══════════════════════════════════════════════════════════════════════════════

def send_pid():
    msg = f"Kp={pid_values['Kp']:.2f},Ki={pid_values['Ki']:.2f},Kd={pid_values['Kd']:.2f},Kw={pid_values['Kw']:.2f},Kff={pid_values['Kff']:.3f}"
    print(f"📤 Send: {msg}")
    try:
        sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
        save_config()
    except Exception as e:
        print(f"❌ Send error: {e}")

def send_command(cmd):
    try:
        sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))
        print(f"📤 Command: {cmd}")
    except Exception as e:
        print(f"❌ Command error: {e}")

def udp_listener():
    global running, connected, last_data_time, balance_state
    
    start_time = time.time()
    
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode().strip()
            
            if text.startswith("A:"):
                parts = text.split(',')
                try:
                    angle = float(parts[0].split(':')[1])
                    gyro = float(parts[1].split(':')[1])
                    pwm = int(parts[2].split(':')[1])
                    wheel = float(parts[3].split(':')[1])
                    integral = float(parts[4].split(':')[1])
                    balance = int(parts[5].split(':')[1])
                    
                    current_time = time.time() - start_time
                    
                    angle_buffer.append(angle)
                    gyro_buffer.append(gyro)
                    pwm_buffer.append(pwm)
                    integral_buffer.append(integral)
                    time_buffer.append(current_time)
                    
                    connected = True
                    last_data_time = time.time()
                    balance_state = balance == 1
                    
                    if logging_enabled:
                        log_to_file(current_time, angle, gyro, pwm, integral)
                    
                except (IndexError, ValueError):
                    pass
            elif text == "ACK":
                print("✅ ESP32 received PID!")
            elif text == "RESET_OK":
                print("✅ ESP32 reset done!")
                
        except socket.timeout:
            if time.time() - last_data_time > 2:
                connected = False
        except:
            pass

def log_to_file(t, angle, gyro, pwm, integral):
    try:
        with open(LOG_FILE, 'a') as f:
            f.write(f"{t:.3f},{angle:.4f},{gyro:.4f},{pwm},{integral:.4f}\n")
    except:
        pass

# ═══════════════════════════════════════════════════════════════════════════════
# IMPROVED GUI
# ═══════════════════════════════════════════════════════════════════════════════

class PIDTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🎛️ Reaction Wheel PID Tuner - Easy Control")
        self.root.geometry("1500x900")
        self.root.configure(bg='#0a0e1a')
        
        # Main container
        main_container = tk.Frame(root, bg='#0a0e1a')
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # === LEFT PANEL: Controls (Fixed width) ===
        left_panel = tk.Frame(main_container, bg='#0a0e1a', width=450)
        left_panel.pack(side='left', fill='y', padx=(0, 10))
        left_panel.pack_propagate(False)
        
        # === RIGHT PANEL: Graphs ===
        right_panel = tk.Frame(main_container, bg='#0a0e1a')
        right_panel.pack(side='left', fill='both', expand=True)
        
        # Build UI sections
        self.create_status_section(left_panel)
        self.create_pid_controls(left_panel)
        self.create_quick_actions(left_panel)
        self.create_stats_section(left_panel)
        self.create_graphs(right_panel)
        
        # Start updates
        self.update_status()
        self.update_stats()
        self.update_plots()
        
    # ═══════════════════════════════════════════════════════════════════════════
    # STATUS SECTION
    # ═══════════════════════════════════════════════════════════════════════════
    
    def create_status_section(self, parent):
        frame = tk.Frame(parent, bg='#141a2e', relief='ridge', bd=2)
        frame.pack(fill='x', pady=(0, 10))
        
        # Title
        tk.Label(frame, text="🔗 CONNECTION STATUS", 
                bg='#141a2e', fg='#00d4ff', 
                font=('Segoe UI', 12, 'bold')).pack(pady=(8, 5))
        
        # Status indicator
        status_frame = tk.Frame(frame, bg='#141a2e')
        status_frame.pack(pady=5)
        
        self.status_dot = tk.Canvas(status_frame, width=20, height=20, 
                                   bg='#141a2e', highlightthickness=0)
        self.status_dot.pack(side='left', padx=5)
        self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
        
        self.status_label = tk.Label(status_frame, text="❌ Disconnected", 
                                     bg='#141a2e', fg='red', 
                                     font=('Segoe UI', 10))
        self.status_label.pack(side='left')
        
        # Balance status
        self.balance_label = tk.Label(frame, text="", 
                                      bg='#141a2e', fg='#888', 
                                      font=('Segoe UI', 11, 'bold'))
        self.balance_label.pack(pady=(0, 8))
        
    # ═══════════════════════════════════════════════════════════════════════════
    # PID CONTROLS - IMPROVED
    # ═══════════════════════════════════════════════════════════════════════════
    
    def create_pid_controls(self, parent):
        frame = tk.Frame(parent, bg='#141a2e', relief='ridge', bd=2)
        frame.pack(fill='both', expand=True, pady=(0, 10))
        
        # Title
        tk.Label(frame, text="⚙️ PID PARAMETERS", 
                bg='#141a2e', fg='#00d4ff', 
                font=('Segoe UI', 12, 'bold')).pack(pady=(8, 10))
        
        # Container for parameters
        params_container = tk.Frame(frame, bg='#141a2e')
        params_container.pack(fill='both', expand=True, padx=15, pady=(0, 10))
        
        self.sliders = {}
        self.entries = {}
        
        # Define parameters with better ranges and steps
        params = [
            ("Kp", "Proportional", 0, 300, 1, "#ff6b9d"),
            ("Ki", "Integral", 0, 10, 0.1, "#4ecdc4"),
            ("Kd", "Derivative", 0, 50, 0.5, "#ffe66d"),
            ("Kw", "Wheel Speed", 0, 10, 0.1, "#95e1d3"),
            ("Kff", "Feedforward", 0, 1, 0.01, "#f38181")
        ]
        
        for param_name, description, min_val, max_val, step, color in params:
            self.create_param_control(params_container, param_name, description, 
                                     min_val, max_val, step, color)
        
        # Send button
        send_btn = tk.Button(frame, text="📤 SEND TO ESP32", 
                           command=self.send_all_pid,
                           bg='#00d4ff', fg='#0a0e1a', 
                           font=('Segoe UI', 11, 'bold'),
                           relief='flat', cursor='hand2',
                           activebackground='#00a8cc')
        send_btn.pack(pady=(10, 8), ipadx=20, ipady=8)
        
    def create_param_control(self, parent, name, desc, min_val, max_val, step, color):
        """Create an improved parameter control with slider + entry + buttons"""
        
        container = tk.Frame(parent, bg='#1a2332', relief='groove', bd=1)
        container.pack(fill='x', pady=6)
        
        # Header with name and value
        header = tk.Frame(container, bg='#1a2332')
        header.pack(fill='x', padx=8, pady=(6, 4))
        
        tk.Label(header, text=name, 
                bg='#1a2332', fg=color, 
                font=('Segoe UI', 10, 'bold')).pack(side='left')
        
        value_label = tk.Label(header, text=f"{pid_values[name]:.2f}", 
                              bg='#1a2332', fg='white', 
                              font=('Consolas', 11, 'bold'))
        value_label.pack(side='right')
        
        # Description
        tk.Label(container, text=desc, 
                bg='#1a2332', fg='#888', 
                font=('Segoe UI', 8)).pack(anchor='w', padx=8)
        
        # Control row: [-] [Entry] [+] [Slider]
        control_frame = tk.Frame(container, bg='#1a2332')
        control_frame.pack(fill='x', padx=8, pady=(4, 6))
        
        # Minus button
        minus_btn = tk.Button(control_frame, text="−", 
                             command=lambda: self.adjust_value(name, -step, value_label, slider),
                             bg='#2a3a4a', fg='white', 
                             font=('Arial', 10, 'bold'),
                             width=2, relief='flat', cursor='hand2')
        minus_btn.pack(side='left', padx=(0, 4))
        
        # Entry box for direct input
        entry = tk.Entry(control_frame, width=8, 
                        bg='#0a0e1a', fg='white', 
                        font=('Consolas', 10),
                        insertbackground='white',
                        justify='center', relief='flat')
        entry.insert(0, f"{pid_values[name]:.2f}")
        entry.pack(side='left', padx=(0, 4))
        entry.bind('<Return>', lambda e: self.on_entry_change(name, entry, value_label, slider))
        entry.bind('<FocusOut>', lambda e: self.on_entry_change(name, entry, value_label, slider))
        
        # Plus button
        plus_btn = tk.Button(control_frame, text="+", 
                            command=lambda: self.adjust_value(name, step, value_label, slider),
                            bg='#2a3a4a', fg='white', 
                            font=('Arial', 10, 'bold'),
                            width=2, relief='flat', cursor='hand2')
        plus_btn.pack(side='left', padx=(0, 6))
        
        # Slider
        slider = tk.Scale(control_frame, from_=min_val, to=max_val, 
                         resolution=step,
                         orient='horizontal',
                         bg='#1a2332', fg=color,
                         troughcolor='#0a0e1a',
                         highlightthickness=0,
                         showvalue=False,
                         command=lambda v: self.on_slider_change(name, v, value_label, entry))
        slider.set(pid_values[name])
        slider.pack(side='left', fill='x', expand=True)
        
        self.sliders[name] = (slider, value_label)
        self.entries[name] = entry
    
    def adjust_value(self, name, delta, value_label, slider):
        """Adjust value by step amount"""
        current = pid_values[name]
        new_val = current + delta
        
        # Clamp to slider range
        min_val = slider.cget('from')
        max_val = slider.cget('to')
        new_val = max(min_val, min(max_val, new_val))
        
        pid_values[name] = new_val
        slider.set(new_val)
        value_label.config(text=f"{new_val:.2f}")
        self.entries[name].delete(0, 'end')
        self.entries[name].insert(0, f"{new_val:.2f}")
    
    def on_slider_change(self, name, value, value_label, entry):
        """Handle slider change"""
        val = float(value)
        pid_values[name] = val
        value_label.config(text=f"{val:.2f}")
        entry.delete(0, 'end')
        entry.insert(0, f"{val:.2f}")
    
    def on_entry_change(self, name, entry, value_label, slider):
        """Handle direct entry input"""
        try:
            val = float(entry.get())
            min_val = slider.cget('from')
            max_val = slider.cget('to')
            val = max(min_val, min(max_val, val))
            
            pid_values[name] = val
            slider.set(val)
            value_label.config(text=f"{val:.2f}")
            entry.delete(0, 'end')
            entry.insert(0, f"{val:.2f}")
        except ValueError:
            # Reset to current value if invalid
            entry.delete(0, 'end')
            entry.insert(0, f"{pid_values[name]:.2f}")
    
    def send_all_pid(self):
        """Send all PID values to ESP32"""
        send_pid()
        messagebox.showinfo("Success", "PID values sent to ESP32!")
    
    # ═══════════════════════════════════════════════════════════════════════════
    # QUICK ACTIONS
    # ═══════════════════════════════════════════════════════════════════════════
    
    def create_quick_actions(self, parent):
        frame = tk.Frame(parent, bg='#141a2e', relief='ridge', bd=2)
        frame.pack(fill='x', pady=(0, 10))
        
        tk.Label(frame, text="⚡ QUICK ACTIONS", 
                bg='#141a2e', fg='#00d4ff', 
                font=('Segoe UI', 11, 'bold')).pack(pady=(8, 8))
        
        btn_frame = tk.Frame(frame, bg='#141a2e')
        btn_frame.pack(pady=(0, 8))
        
        # Presets
        presets = [
            ("🔹 Soft", {"Kp": 50, "Ki": 0.5, "Kd": 15, "Kw": 2.0, "Kff": 0.1}),
            ("🔸 Medium", {"Kp": 100, "Ki": 1.5, "Kd": 30, "Kw": 3.0, "Kff": 0.2}),
            ("🔥 Aggressive", {"Kp": 150, "Ki": 3.0, "Kd": 40, "Kw": 4.0, "Kff": 0.3})
        ]
        
        for i, (label, values) in enumerate(presets):
            btn = tk.Button(btn_frame, text=label, 
                          command=lambda v=values: self.apply_preset(v),
                          bg='#2a3a4a', fg='white', 
                          font=('Segoe UI', 9),
                          width=12, relief='flat', cursor='hand2')
            btn.grid(row=i//2, column=i%2, padx=4, pady=3)
        
        # Commands
        cmd_frame = tk.Frame(frame, bg='#141a2e')
        cmd_frame.pack(pady=(5, 8))
        
        tk.Button(cmd_frame, text="🔄 Reset ESP32", 
                 command=lambda: send_command("RESET"),
                 bg='#ff6b6b', fg='white', 
                 font=('Segoe UI', 9),
                 width=15, relief='flat', cursor='hand2').pack(pady=2)
        
        tk.Button(cmd_frame, text="📊 Clear Graphs", 
                 command=self.clear_buffers,
                 bg='#4a6a8a', fg='white', 
                 font=('Segoe UI', 9),
                 width=15, relief='flat', cursor='hand2').pack(pady=2)
    
    def apply_preset(self, values):
        """Apply preset PID values"""
        for key, value in values.items():
            pid_values[key] = value
            if key in self.sliders:
                slider, label = self.sliders[key]
                slider.set(value)
                label.config(text=f"{value:.2f}")
            if key in self.entries:
                self.entries[key].delete(0, 'end')
                self.entries[key].insert(0, f"{value:.2f}")
        send_pid()
    
    def clear_buffers(self):
        """Clear all data buffers"""
        angle_buffer.clear()
        gyro_buffer.clear()
        pwm_buffer.clear()
        integral_buffer.clear()
        time_buffer.clear()
    
    # ═══════════════════════════════════════════════════════════════════════════
    # STATS SECTION
    # ═══════════════════════════════════════════════════════════════════════════
    
    def create_stats_section(self, parent):
        frame = tk.Frame(parent, bg='#141a2e', relief='ridge', bd=2)
        frame.pack(fill='x')
        
        tk.Label(frame, text="📈 STATISTICS", 
                bg='#141a2e', fg='#00d4ff', 
                font=('Segoe UI', 11, 'bold')).pack(pady=(8, 8))
        
        stats_grid = tk.Frame(frame, bg='#141a2e')
        stats_grid.pack(pady=(0, 8), padx=10)
        
        self.stat_labels = {}
        stats = [
            ("Max Angle", "°", 0, 0),
            ("Avg Angle", "°", 0, 1),
            ("RMS", "", 1, 0),
            ("Freq", "Hz", 1, 1)
        ]
        
        for name, unit, row, col in stats:
            container = tk.Frame(stats_grid, bg='#1a2332', relief='groove', bd=1)
            container.grid(row=row, column=col, padx=4, pady=4, sticky='ew')
            
            tk.Label(container, text=name, 
                    bg='#1a2332', fg='#888', 
                    font=('Segoe UI', 8)).pack()
            
            value_label = tk.Label(container, text="0.00", 
                                  bg='#1a2332', fg='#00ff88', 
                                  font=('Consolas', 13, 'bold'))
            value_label.pack()
            
            if unit:
                tk.Label(container, text=unit, 
                        bg='#1a2332', fg='#666', 
                        font=('Segoe UI', 7)).pack()
            
            self.stat_labels[name] = value_label
        
        stats_grid.columnconfigure(0, weight=1)
        stats_grid.columnconfigure(1, weight=1)
    
    # ═══════════════════════════════════════════════════════════════════════════
    # GRAPHS - FIXED VIEWPORT
    # ═══════════════════════════════════════════════════════════════════════════
    
    def create_graphs(self, parent):
        """Create graph section with fixed viewports"""
        
        # Angle graph
        angle_frame = tk.LabelFrame(parent, text="📐 Angle (degrees)", 
                                   bg='#141a2e', fg='#00d4ff', 
                                   font=('Segoe UI', 10, 'bold'),
                                   relief='ridge', bd=2)
        angle_frame.pack(fill='both', expand=True, pady=(0, 8))
        
        self.angle_canvas = tk.Canvas(angle_frame, bg='#0a0e1a', 
                                     highlightthickness=0, height=220)
        self.angle_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # PWM graph
        pwm_frame = tk.LabelFrame(parent, text="⚡ PWM Output", 
                                 bg='#141a2e', fg='#ff9f43', 
                                 font=('Segoe UI', 10, 'bold'),
                                 relief='ridge', bd=2)
        pwm_frame.pack(fill='both', expand=True, pady=(0, 8))
        
        self.pwm_canvas = tk.Canvas(pwm_frame, bg='#0a0e1a', 
                                   highlightthickness=0, height=200)
        self.pwm_canvas.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Integral graph
        int_frame = tk.LabelFrame(parent, text="∫ Integral Term", 
                                 bg='#141a2e', fg='#00ff88', 
                                 font=('Segoe UI', 10, 'bold'),
                                 relief='ridge', bd=2)
        int_frame.pack(fill='both', expand=True)
        
        self.int_canvas = tk.Canvas(int_frame, bg='#0a0e1a', 
                                   highlightthickness=0, height=180)
        self.int_canvas.pack(fill='both', expand=True, padx=5, pady=5)
    
    # ═══════════════════════════════════════════════════════════════════════════
    # UPDATE FUNCTIONS
    # ═══════════════════════════════════════════════════════════════════════════
    
    def update_status(self):
        if not running:
            return
            
        if connected:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='#00ff88', outline='')
            self.status_label.config(text=f"✅ Connected ({ESP_IP})", fg='#00ff88')
            
            if balance_state:
                self.balance_label.config(text="🟢 BALANCED", fg='#00ff88')
            else:
                self.balance_label.config(text="🔴 NOT BALANCED", fg='#ff4444')
        else:
            self.status_dot.delete('all')
            self.status_dot.create_oval(2, 2, 18, 18, fill='red', outline='')
            self.status_label.config(text="❌ Disconnected", fg='red')
            self.balance_label.config(text="", fg='#888888')
            
        self.root.after(500, self.update_status)
    
    def update_stats(self):
        if not running:
            return
            
        angles = list(angle_buffer)
        if len(angles) > 10:
            max_angle = max(abs(a) for a in angles)
            self.stat_labels["Max Angle"].config(text=f"{max_angle:.2f}")
            
            avg_angle = sum(abs(a) for a in angles) / len(angles)
            self.stat_labels["Avg Angle"].config(text=f"{avg_angle:.2f}")
            
            rms = math.sqrt(sum(a*a for a in angles) / len(angles))
            self.stat_labels["RMS"].config(text=f"{rms:.2f}")
            
            # Simple frequency estimation
            if len(angles) > 50:
                zero_crossings = 0
                for i in range(1, len(angles)):
                    if angles[i-1] * angles[i] < 0:
                        zero_crossings += 1
                freq = zero_crossings / (len(angles) * 0.005) / 2
                self.stat_labels["Freq"].config(text=f"{freq:.2f}")
        
        self.root.after(200, self.update_stats)
    
    def update_plots(self):
        if not running:
            return
            
        self.draw_angle_plot()
        self.draw_pwm_plot()
        self.draw_integral_plot()
        
        self.root.after(50, self.update_plots)
    
    def draw_angle_plot(self):
        """Draw angle plot with FIXED viewport"""
        canvas = self.angle_canvas
        canvas.delete('all')
        
        # Force update to get actual size
        canvas.update_idletasks()
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
            
        cy = h / 2
        
        # Grid lines (±40 degrees range - FIXED)
        for i in range(-40, 41, 10):
            y = cy - (i * h / 100)  # Fixed scale: 100 = ±50 degrees
            if 0 <= y <= h:
                color = '#2a3a4a' if i != 0 else '#4a6a8a'
                width = 2 if i == 0 else 1
                canvas.create_line(0, y, w, y, fill=color, dash=(2, 4), width=width)
                if i % 20 == 0:
                    canvas.create_text(30, y, text=f"{i}°", 
                                     fill='#6a8aaa', font=('Consolas', 9))
        
        # Data
        points = list(angle_buffer)
        if len(points) < 2:
            canvas.create_text(w/2, h/2, text="Waiting for data...", 
                              fill='#4a6a8a', font=('Segoe UI', 11))
            return
        
        # FIXED scale
        y_scale = h / 100  # ±50 degrees viewport
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, angle in enumerate(points):
            x = i * x_step
            y = cy - (angle * y_scale)
            y = max(0, min(h, y))  # Clamp to canvas
            coords.extend([x, y])
        
        if len(coords) >= 4:
            canvas.create_line(coords, fill='#00d4ff', width=2, smooth=True)
        
        # Current value display
        current = points[-1]
        color = '#00ff88' if abs(current) < 3 else '#ff6b6b'
        canvas.create_text(w - 50, 25, text=f"{current:.2f}°", 
                          fill=color, font=('Consolas', 16, 'bold'))
    
    def draw_pwm_plot(self):
        """Draw PWM plot with FIXED viewport"""
        canvas = self.pwm_canvas
        canvas.delete('all')
        
        canvas.update_idletasks()
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
        
        cy = h / 2
        
        # Grid
        for i in [-400, -200, 0, 200, 400]:
            y = cy - (i * h / 1000)  # Fixed: ±500 PWM range
            if 0 <= y <= h:
                color = '#4a6a8a' if i == 0 else '#2a3a4a'
                width = 2 if i == 0 else 1
                canvas.create_line(0, y, w, y, fill=color, dash=(2, 4), width=width)
        
        points = list(pwm_buffer)
        if len(points) < 2:
            return
        
        # FIXED scale
        y_scale = h / 1000  # ±500 PWM
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, pwm_val in enumerate(points):
            x = i * x_step
            y = cy - (pwm_val * y_scale)
            y = max(0, min(h, y))
            coords.extend([x, y])
        
        if len(coords) >= 4:
            canvas.create_line(coords, fill='#ff9f43', width=2, smooth=True)
        
        current = points[-1]
        canvas.create_text(w - 50, 25, text=f"{current}", 
                          fill='#ff9f43', font=('Consolas', 16, 'bold'))
    
    def draw_integral_plot(self):
        """Draw integral plot with AUTO-SCALE but stable viewport"""
        canvas = self.int_canvas
        canvas.delete('all')
        
        canvas.update_idletasks()
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        
        if w < 10 or h < 10:
            return
        
        cy = h / 2
        
        # Grid
        canvas.create_line(0, cy, w, cy, fill='#4a6a8a', dash=(2, 4), width=2)
        
        points = list(integral_buffer)
        if len(points) < 2:
            return
        
        # Smooth auto-scale with minimum range
        max_val = max(abs(p) for p in points) if points else 50
        max_val = max(max_val, 50)  # Minimum 50 range
        max_val = math.ceil(max_val / 50) * 50  # Round up to nearest 50
        
        y_scale = h / (max_val * 2.2)
        x_step = w / BUFFER_SIZE
        
        coords = []
        for i, val in enumerate(points):
            x = i * x_step
            y = cy - (val * y_scale)
            y = max(0, min(h, y))
            coords.extend([x, y])
        
        if len(coords) >= 4:
            canvas.create_line(coords, fill='#00ff88', width=2, smooth=True)
        
        current = points[-1]
        canvas.create_text(w - 70, 25, text=f"I={current:.1f}", 
                          fill='#00ff88', font=('Consolas', 14, 'bold'))

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    load_config()
    
    # Start UDP listener
    listener_thread = threading.Thread(target=udp_listener, daemon=True)
    listener_thread.start()
    
    # Create GUI
    root = tk.Tk()
    app = PIDTunerGUI(root)
    
    print("═══════════════════════════════════════════════════════════════")
    print("  🎛️ Reaction Wheel PID Tuner - IMPROVED UI")
    print("═══════════════════════════════════════════════════════════════")
    print(f"\n✅ ESP32 IP: {ESP_IP}")
    print(f"✅ Listening on: {UDP_IP_LISTEN}:{UDP_PORT_LISTEN}")
    print("\n🎨 UI Improvements:")
    print("  • Fixed graph viewports - no more auto-shifting!")
    print("  • Easy value adjustment: +/- buttons + direct input")
    print("  • Cleaner layout with better organization")
    print("  • Improved color scheme for readability")
    print("═══════════════════════════════════════════════════════════════\n")
    
    try:
        root.mainloop()
    finally:
        running = False
        sock.close()
        print("\n👋 Goodbye!")