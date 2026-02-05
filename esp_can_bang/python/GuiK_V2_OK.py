import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox
from collections import deque

# --- Cấu hình UDP ---
UDP_IP_PC = "0.0.0.0"
UDP_PORT_PC = 4210     # Python lắng nghe

# 🟢 ĐỔI IP ESP32 SAU KHI XEM SERIAL MONITOR!
# ESP32 sẽ in ra IP khi khởi động, ví dụ: "ESP32 IP: 192.168.137.2"
# Windows Mobile Hotspot thường dùng dải 192.168.137.x
ESP32_IP = "192.168.137.2"   # ← Thay bằng IP thật của ESP32
ESP32_PORT = 4210            # Cổng UDP của ESP32

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP_PC, UDP_PORT_PC))
sock.settimeout(0.1)

angles_z = deque(maxlen=200)
# Giá trị mặc định PID
K1, K2, K3 = 60.0, 15.0, 1.5

# --- Hàm gửi hệ số xuống ESP32 ---
def send_gains(k1, k2, k3):
    msg = f"K1={k1:.2f},K2={k2:.2f},K3={k3:.2f}"
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    print(f"📤 Gửi hệ số: {msg}")

# --- Gửi hệ số khởi tạo ban đầu ---
send_gains(K1, K2, K3)

# --- Giao diện đồ thị ---
plt.style.use('seaborn-v0_8-darkgrid')
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.42)  # chừa chỗ cho sliders + nút + giá trị

line, = ax.plot([], [], color='b', label='Z-axis')
ax.legend()
ax.set_ylim(-10, 10)
ax.set_xlim(0, 200)
ax.set_xlabel("Samples")
ax.set_ylabel("Angle (°)")
ax.set_title("Real-time Z-axis Angle + PID gains control")

# --- Tạo thanh trượt ---
axcolor = 'lightgoldenrodyellow'
axK1 = plt.axes([0.15, 0.28, 0.55, 0.03], facecolor=axcolor)
axK2 = plt.axes([0.15, 0.22, 0.55, 0.03], facecolor=axcolor)
axK3 = plt.axes([0.15, 0.16, 0.55, 0.03], facecolor=axcolor)

sliderK1 = Slider(axK1, 'K1', 0.0, 150.0, valinit=K1, valstep=1.0)  # P: 0-150
sliderK2 = Slider(axK2, 'K2', 0.0, 30.0, valinit=K2, valstep=0.5)   # D: 0-30
sliderK3 = Slider(axK3, 'K3', 0.0, 5.0, valinit=K3, valstep=0.1)    # Motor: 0-5

# --- Hiển thị giá trị số ---
valBoxK1 = plt.axes([0.76, 0.28, 0.08, 0.03])
valBoxK2 = plt.axes([0.76, 0.22, 0.08, 0.03])
valBoxK3 = plt.axes([0.76, 0.16, 0.08, 0.03])
textK1 = TextBox(valBoxK1, "", initial=f"{K1:.2f}")
textK2 = TextBox(valBoxK2, "", initial=f"{K2:.2f}")
textK3 = TextBox(valBoxK3, "", initial=f"{K3:.2f}")

# --- Nút + và – ---
btn_minusK1 = plt.axes([0.05, 0.28, 0.05, 0.03])
btn_plusK1 = plt.axes([0.84, 0.28, 0.05, 0.03])
btn_minusK2 = plt.axes([0.05, 0.22, 0.05, 0.03])
btn_plusK2 = plt.axes([0.84, 0.22, 0.05, 0.03])
btn_minusK3 = plt.axes([0.05, 0.16, 0.05, 0.03])
btn_plusK3 = plt.axes([0.84, 0.16, 0.05, 0.03])

b_mK1 = Button(btn_minusK1, '–')
b_pK1 = Button(btn_plusK1, '+')
b_mK2 = Button(btn_minusK2, '–')
b_pK2 = Button(btn_plusK2, '+')
b_mK3 = Button(btn_minusK3, '–')
b_pK3 = Button(btn_plusK3, '+')

# --- Hàm cập nhật hệ số và hiển thị ---
def update_display():
    textK1.set_val(f"{sliderK1.val:.2f}")
    textK2.set_val(f"{sliderK2.val:.2f}")
    textK3.set_val(f"{sliderK3.val:.2f}")
    send_gains(sliderK1.val, sliderK2.val, sliderK3.val)

def update_sliders(val):
    update_display()

sliderK1.on_changed(update_sliders)
sliderK2.on_changed(update_sliders)
sliderK3.on_changed(update_sliders)

# --- Hàm nút tăng giảm ---
def make_adjuster(slider, delta):
    def adjust(event):
        new_val = min(max(slider.val + delta, slider.valmin), slider.valmax)
        slider.set_val(new_val)
        update_display()
    return adjust

b_mK1.on_clicked(make_adjuster(sliderK1, -0.05))
b_pK1.on_clicked(make_adjuster(sliderK1, 0.05))
b_mK2.on_clicked(make_adjuster(sliderK2, -0.1))
b_pK2.on_clicked(make_adjuster(sliderK2, 0.1))
b_mK3.on_clicked(make_adjuster(sliderK3, -0.1))
b_pK3.on_clicked(make_adjuster(sliderK3, 0.1))

# --- Cập nhật dữ liệu nhận từ ESP32 ---
def update(frame):
    try:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode().strip()
        print(data)
        if decoded.startswith("KACK"):
            print("✅ ESP32 xác nhận hệ số")
            return line,

        values = decoded.split(',')
        if len(values) == 3:
            _, _, z = map(float, values)
        else:
            z = float(values[0])
        angles_z.append(z)
        line.set_data(range(len(angles_z)), list(angles_z))
    except socket.timeout:
        pass
    except Exception as e:
        print("⚠️ Lỗi:", e)
    return line,

ani = animation.FuncAnimation(fig, update, interval=30, blit=True, cache_frame_data=False)
plt.show()
