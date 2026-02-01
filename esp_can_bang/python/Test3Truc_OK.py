import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

UDP_IP = "0.0.0.0"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

# Dùng deque để lưu trữ giới hạn — tránh trễ do mảng dài
angles_x = deque(maxlen=100)
angles_y = deque(maxlen=100)
angles_z = deque(maxlen=100)

fig, ax = plt.subplots()
line1, = ax.plot([], [], label='X', color='r')
line2, = ax.plot([], [], label='Y', color='g')
line3, = ax.plot([], [], label='Z', color='b')
ax.legend()
ax.set_ylim(-180, 180)
ax.set_xlim(0, 100)

def update(frame):
    try:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode().strip()
        x, y, z = map(float, decoded.split(','))
        print(decoded)

        angles_x.append(x)
        angles_y.append(y)
        angles_z.append(z)

        line1.set_data(range(len(angles_x)), list(angles_x))
        line2.set_data(range(len(angles_y)), list(angles_y))
        line3.set_data(range(len(angles_z)), list(angles_z))

    except socket.timeout:
        pass
    except Exception as e:
        print("⚠️ Error:", e)

    return line1, line2, line3

ani = animation.FuncAnimation(
    fig, update,
    interval=30,        # 30 ms/lần (≈33 FPS)
    blit=True,          # chỉ vẽ phần thay đổi
    cache_frame_data=False
)

plt.show()
