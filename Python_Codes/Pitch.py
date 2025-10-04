import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from collections import deque

# ----- CONFIG -----
PORT = 'COM20'  # Change to your Arduino port
BAUD = 230400                 # Match your Arduino baud rate
WINDOW = 200                   # Number of samples shown
UPDATE_INTERVAL = 30           # ms between updates (~33 Hz)

# Serial connection
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# Buffers
pitch_buf = deque(maxlen=WINDOW)
x_idx = deque(maxlen=WINDOW)

# Matplotlib figure
fig = plt.figure(figsize=(8,5))

# Top: Pitch time-series
ax1 = fig.add_subplot(2,1,1)
(line_pitch,) = ax1.plot([], [], label="Pitch (°)", color='tab:blue', linewidth=2)
ax1.set_xlim(0, WINDOW)
ax1.set_ylim(-90, 90)
ax1.set_xlabel("Samples")
ax1.set_ylabel("Pitch (°)")
ax1.set_title("MPU6050 Pitch (Y-axis)")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)

# Bottom: Pitch-driven seesaw bar
ax2 = fig.add_subplot(2,1,2)
ax2.set_xlim(-2, 2)
ax2.set_ylim(-1.2, 1.2)
ax2.set_aspect('equal', adjustable='box')
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_title("Pitch-driven Tilt")
bar = Rectangle((-1.5, -0.1), 3.0, 0.2, angle=0)
ax2.add_patch(bar)

def update_bar_angle(angle_deg):
    """Rotate bar around its center according to pitch"""
    t = plt.matplotlib.transforms.Affine2D().rotate_deg_around(0, 0, angle_deg) + ax2.transData
    bar.set_transform(t)

def parse_line(line):
    """Extract pitch from CSV line, ignore roll/yaw if present"""
    try:
        parts = line.strip().split(',')
        pitch = float(parts[0])
        return pitch
    except:
        return None

def init():
    line_pitch.set_data([], [])
    update_bar_angle(0)
    return (line_pitch, bar)

def update(frame):
    # Read multiple lines quickly to prevent backlog
    for _ in range(5):
        raw = ser.readline().decode(errors='ignore')
        if not raw:
            break
        pitch = parse_line(raw)
        if pitch is None:
            continue
        pitch_buf.append(pitch)
        x_idx.append(len(x_idx) + 1 if x_idx else 1)

    # Update pitch time-series
    xs = list(range(len(x_idx)))
    line_pitch.set_data(xs, list(pitch_buf))
    ax1.set_xlim(max(0, len(xs)-WINDOW), max(WINDOW, len(xs)))

    # Update seesaw tilt
    if pitch_buf:
        update_bar_angle(pitch_buf[-1])

    return (line_pitch, bar)

# Animation
ani = animation.FuncAnimation(fig, update, init_func=init, interval=UPDATE_INTERVAL, blit=True)
plt.tight_layout()
plt.show()
