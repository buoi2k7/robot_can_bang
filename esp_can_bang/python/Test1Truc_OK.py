import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Cấu hình UDP ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

# --- Bộ đệm dữ liệu ---
angles_z = deque(maxlen=200)

# --- Thiết lập đồ thị ---
fig, ax = plt.subplots()
line, = ax.plot([], [], color='b', label='Z-axis')
ax.legend()
ax.set_ylim(-50, 50)
ax.set_xlim(0, 200)
ax.set_xlabel("Samples")
ax.set_ylabel("Angle (°)")
ax.set_title("Real-time Z-axis Angle from ESP32 (MPU6050)")

# --- Hàm cập nhật khung hình ---
def update(frame):
    try:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode().strip()
        values = decoded.split(',')

        # Hỗ trợ cả chuỗi "x,y,z" hoặc chỉ "z"
        if len(values) == 3:
            _, _, z = map(float, values)
        else:
            z = float(values[0])

        print(f"Z = {z:.2f}")
        angles_z.append(z)

        line.set_data(range(len(angles_z)), list(angles_z))

    except socket.timeout:
        pass
    except Exception as e:
        print("⚠️ Error:", e)

    return line,

# --- Animation (30 FPS, cực mượt) ---
ani = animation.FuncAnimation(
    fig, update,
    interval=30,
    blit=True,
    cache_frame_data=False
)

plt.show()
