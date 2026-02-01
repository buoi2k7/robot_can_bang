import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from collections import deque

# --- Cấu hình UDP ---
UDP_IP_PC = "0.0.0.0"
UDP_PORT_PC = 4210     # Python lắng nghe
ESP32_IP = "192.168.1.89"   # 🟢 Địa chỉ ESP32 của bạn
ESP32_PORT = 4210            # 🟢 Cổng UDP của ESP32

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP_PC, UDP_PORT_PC))
sock.settimeout(0.1)

angles_z = deque(maxlen=200)
K1, K2, K3 = 1.0, 1.0, 1.0   # Giá trị mặc định ban đầu

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
plt.subplots_adjust(bottom=0.35)  # chừa chỗ cho sliders

line, = ax.plot([], [], color='b', label='Z-axis')
ax.legend()
ax.set_ylim(-50, 50)
ax.set_xlim(0, 200)
ax.set_xlabel("Samples")
ax.set_ylabel("Angle (°)")
ax.set_title("Real-time Z-axis Angle + PID gains control")

# --- Tạo thanh trượt ---
axcolor = 'lightgoldenrodyellow'
axK1 = plt.axes([0.15, 0.20, 0.65, 0.03], facecolor=axcolor)
axK2 = plt.axes([0.15, 0.14, 0.65, 0.03], facecolor=axcolor)
axK3 = plt.axes([0.15, 0.08, 0.65, 0.03], facecolor=axcolor)

sliderK1 = Slider(axK1, 'K1', 0.0, 5.0, valinit=K1, valstep=0.1)
sliderK2 = Slider(axK2, 'K2', 0.0, 5.0, valinit=K2, valstep=0.1)
sliderK3 = Slider(axK3, 'K3', 0.0, 5.0, valinit=K3, valstep=0.1)

# --- Cập nhật khi kéo thanh ---
def update_sliders(val):
    global K1, K2, K3
    K1 = sliderK1.val
    K2 = sliderK2.val
    K3 = sliderK3.val
    send_gains(K1, K2, K3)

sliderK1.on_changed(update_sliders)
sliderK2.on_changed(update_sliders)
sliderK3.on_changed(update_sliders)

# --- Cập nhật dữ liệu nhận ---
def update(frame):
    try:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode().strip()

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
