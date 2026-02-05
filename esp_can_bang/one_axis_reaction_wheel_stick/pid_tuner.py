import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
import collections

# Cấu hình IP/Port
UDP_IP_LISTEN = "0.0.0.0"  # Nghe trên tất cả các IP máy tính
UDP_PORT_LISTEN = 4210     # Port mà ESP32 gửi dữ liệu đến

ESP_IP = "192.168.1.7"     # IP của ESP32 (đã set tĩnh trong code arduino)
ESP_PORT = 4210            # Port của ESP32

# Biến toàn cục dữ liệu
data_buffer = collections.deque(maxlen=200) # Lưu 200 điểm dữ liệu góc
current_k1 = 50.0
current_k2 = 20.0
current_k3 = 1.5

running = True

# --- Xử lý mạng UDP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP_LISTEN, UDP_PORT_LISTEN))
sock.settimeout(0.1)

def send_pid():
    global current_k1, current_k2, current_k3
    msg = f"K1={current_k1:.2f},K2={current_k2:.2f},K3={current_k3:.2f}"
    print(f"Gửi: {msg}")
    try:
        sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
    except Exception as e:
        print(f"Lỗi gửi: {e}")

def udp_listener():
    global running
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode().strip()
            # Dự kiến format: "pitch,pwm_s,robot_angle" (theo function.ino)
            parts = text.split(',')
            if len(parts) >= 3:
                # Giá trị thứ 3 là robot_angle
                angle = float(parts[2])
                data_buffer.append(angle)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Lỗi nhận: {e}")

# --- Giao diện đồ họa (GUI) ---
root = tk.Tk()
root.title("Cân Bằng Robot - Tuning Tool")
root.geometry("800x600")

# Frame chỉnh PID
frame_controls = tk.LabelFrame(root, text="PID Parameters")
frame_controls.pack(fill="x", padx=10, pady=5)

def on_slider_change(event=None):
    global current_k1, current_k2, current_k3
    current_k1 = sld_k1.get()
    current_k2 = sld_k2.get()
    current_k3 = sld_k3.get()
    lbl_k1.config(text=f"K1 (Angle P): {current_k1:.2f}")
    lbl_k2.config(text=f"K2 (Gyro D): {current_k2:.2f}")
    lbl_k3.config(text=f"K3 (Speed D): {current_k3:.2f}")

def on_send_click():
    send_pid()

# Slider K1
tk.Label(frame_controls, text="K1 (Độ cứng góc - P)").pack(anchor="w")
sld_k1 = tk.Scale(frame_controls, from_=0, to=200, resolution=0.5, orient="horizontal", command=on_slider_change)
sld_k1.set(current_k1)
sld_k1.pack(fill="x")
lbl_k1 = tk.Label(frame_controls, text="...")
lbl_k1.pack()

# Slider K2
tk.Label(frame_controls, text="K2 (Chống rung/Dập tắt - D)").pack(anchor="w")
sld_k2 = tk.Scale(frame_controls, from_=0, to=100, resolution=0.5, orient="horizontal", command=on_slider_change)
sld_k2.set(current_k2)
sld_k2.pack(fill="x")
lbl_k2 = tk.Label(frame_controls, text="...")
lbl_k2.pack()

# Slider K3
tk.Label(frame_controls, text="K3 (Phanh motor/Ma sát ảo)").pack(anchor="w")
sld_k3 = tk.Scale(frame_controls, from_=0, to=10, resolution=0.1, orient="horizontal", command=on_slider_change)
sld_k3.set(current_k3)
sld_k3.pack(fill="x")
lbl_k3 = tk.Label(frame_controls, text="...")
lbl_k3.pack()

btn_send = tk.Button(frame_controls, text="Gửi Xuống Robot (UPDATE)", bg="green", fg="white", command=on_send_click)
btn_send.pack(pady=5)

# Canvas vẽ đồ thị
frame_plot = tk.LabelFrame(root, text="Góc Robot (Real-time)")
frame_plot.pack(fill="both", expand=True, padx=10, pady=5)
canvas = tk.Canvas(frame_plot, bg="black")
canvas.pack(fill="both", expand=True)

def draw_plot():
    if not running: return
    canvas.delete("all")
    w = canvas.winfo_width()
    h = canvas.winfo_height()
    cy = h / 2
    
    # Vẽ trục 0
    canvas.create_line(0, cy, w, cy, fill="gray", dash=(2, 4))
    
    points = list(data_buffer)
    if len(points) < 2:
        root.after(50, draw_plot)
        return

    # Scale: Giả sử góc từ -20 đến 20 độ hiển thị đầy màn hình
    y_scale = h / 40.0 
    x_step = w / 200.0

    coords = []
    for i, angle in enumerate(points):
        x = i * x_step
        y = cy - (angle * y_scale) # Y ngược
        coords.append(x)
        coords.append(y)
    
    if len(coords) >= 4:
        canvas.create_line(coords, fill="cyan", width=2)
        # Hiển thị giá trị hiện tại
        canvas.create_text(w-50, 20, text=f"{points[-1]:.2f}°", fill="white", font=("Arial", 14))

    root.after(50, draw_plot)

# Start threads and loop
t = threading.Thread(target=udp_listener)
t.daemon = True
t.start()

on_slider_change() # Init labels
draw_plot()

root.mainloop()
running = False
sock.close()
