
import sys
import math
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from collections import deque
import numpy as np

# ----- CONFIG -----
PORT = 'COM20'  # Adjust as needed
BAUD = 230400   # Match Arduino baud rate
WINDOW = 200    # Number of samples shown
UPDATE_INTERVAL = 20  # ms (~50 Hz)

ser = serial.Serial(PORT, BAUD, timeout=0.1)

pitch_buf = deque(maxlen=WINDOW)
roll_buf  = deque(maxlen=WINDOW)
x_idx     = deque(maxlen=WINDOW)

fig = plt.figure(figsize=(14,6))

# Left: time-series of pitch & roll
ax1 = fig.add_subplot(1,2,1)
line_pitch, = ax1.plot([], [], label="Pitch (°)", color='tab:blue', linewidth=2)
line_roll,  = ax1.plot([], [], label="Roll (°)", color='tab:orange', linewidth=2)
ax1.set_xlim(0, WINDOW)
ax1.set_ylim(-180, 180)
ax1.set_xlabel("Samples")
ax1.set_ylabel("Angle (°)")
ax1.set_title("MPU6050 Pitch & Roll (Yaw ignored)")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)

# Right: 3D table visualization (ignoring yaw)
ax2 = fig.add_subplot(1,2,2, projection='3d')
ax2.set_xlim(-3,3)
ax2.set_ylim(-3,3)
ax2.set_zlim(-1,3.5)
ax2.set_xlabel('X (Roll axis)')
ax2.set_ylabel('Y (Pitch axis)')
ax2.set_zlabel('Z (Up)')
ax2.set_title("3D Table - Pitch & Roll Only")
ax2.view_init(elev=20, azim=45)

def create_table_vertices(pitch_deg, roll_deg):
    """Create vertices for a table rotated by pitch and roll (yaw=0)"""
    pitch_rad = np.radians(pitch_deg)
    roll_rad  = np.radians(roll_deg)
    
    # Rotation matrices: Pitch -> Roll
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    Rx = np.array([
        [1,0,0],
        [0,np.cos(roll_rad), -np.sin(roll_rad)],
        [0,np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    R = Rx @ Ry
    
    # Tabletop
    tw, td, th = 3.0, 2.0, 0.15
    tabletop = np.array([
        [-tw/2, -td/2, 0], [tw/2, -td/2, 0], [tw/2, td/2, 0], [-tw/2, td/2, 0],
        [-tw/2, -td/2, th], [tw/2, -td/2, th], [tw/2, td/2, th], [-tw/2, td/2, th]
    ])
    
    # Legs
    leg_w, leg_h = 0.15, 1.2
    leg_inset = 0.3
    leg_positions = [
        (-tw/2+leg_inset, -td/2+leg_inset),
        (tw/2-leg_inset, -td/2+leg_inset),
        (tw/2-leg_inset, td/2-leg_inset),
        (-tw/2+leg_inset, td/2-leg_inset)
    ]
    
    legs = []
    for lx, ly in leg_positions:
        leg = np.array([
            [lx-leg_w/2, ly-leg_w/2, -leg_h], [lx+leg_w/2, ly-leg_w/2, -leg_h],
            [lx+leg_w/2, ly+leg_w/2, -leg_h], [lx-leg_w/2, ly+leg_w/2, -leg_h],
            [lx-leg_w/2, ly-leg_w/2, 0], [lx+leg_w/2, ly-leg_w/2, 0],
            [lx+leg_w/2, ly+leg_w/2, 0], [lx-leg_w/2, ly+leg_w/2, 0]
        ])
        legs.append(leg)
    
    # Apply rotation
    tabletop_rot = tabletop @ R.T
    legs_rot = [leg @ R.T for leg in legs]
    
    # Lift table
    tabletop_rot[:,2] += 1.5
    for leg in legs_rot:
        leg[:,2] += 1.5
        
    return tabletop_rot, legs_rot

def create_table_faces(tabletop, legs):
    faces = []
    faces.append([tabletop[0], tabletop[1], tabletop[2], tabletop[3]])  # bottom
    faces.append([tabletop[4], tabletop[5], tabletop[6], tabletop[7]])  # top
    faces.append([tabletop[0], tabletop[1], tabletop[5], tabletop[4]])  # front
    faces.append([tabletop[2], tabletop[3], tabletop[7], tabletop[6]])  # back
    faces.append([tabletop[0], tabletop[3], tabletop[7], tabletop[4]])  # left
    faces.append([tabletop[1], tabletop[2], tabletop[6], tabletop[5]])  # right
    
    for leg in legs:
        faces.append([leg[0], leg[1], leg[2], leg[3]])  # bottom
        faces.append([leg[4], leg[5], leg[6], leg[7]])  # top
        faces.append([leg[0], leg[1], leg[5], leg[4]])  # front
        faces.append([leg[2], leg[3], leg[7], leg[6]])  # back
        faces.append([leg[0], leg[3], leg[7], leg[4]])  # left
        faces.append([leg[1], leg[2], leg[6], leg[5]])  # right
    return faces

# Initial table
tabletop_verts, legs_verts = create_table_vertices(0,0)
table_faces = create_table_faces(tabletop_verts, legs_verts)
table = Poly3DCollection(table_faces, alpha=0.95, facecolor='#8B4513', edgecolor='#654321', linewidth=1)
ax2.add_collection3d(table)

angle_text = ax2.text2D(0.05, 0.95, '', transform=ax2.transAxes, 
                        fontsize=11, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

def update_3d_table(pitch_deg, roll_deg):
    tabletop_verts, legs_verts = create_table_vertices(pitch_deg, roll_deg)
    table_faces = create_table_faces(tabletop_verts, legs_verts)
    table.set_verts(table_faces)
    angle_text.set_text(f'Pitch: {pitch_deg:+.1f}°\nRoll: {roll_deg:+.1f}°\nYaw: ignored')

def parse_line(line):
    try:
        parts = line.strip().split(',')
        if len(parts) >= 2:
            pitch = float(parts[0])
            roll  = float(parts[1])
            return pitch, roll
        else:
            return None, None
    except:
        return None, None

def init():
    line_pitch.set_data([], [])
    line_roll.set_data([], [])
    update_3d_table(0,0)
    return (line_pitch, line_roll, table)

def update(frame):
    for _ in range(20):
        raw = ser.readline().decode(errors='ignore')
        if not raw: break
        pitch, roll = parse_line(raw)
        if pitch is None: continue
        pitch_buf.append(pitch)
        roll_buf.append(roll)
        x_idx.append(len(x_idx)+1 if x_idx else 1)
        
    xs = list(range(len(x_idx)))
    line_pitch.set_data(xs, list(pitch_buf))
    line_roll.set_data(xs, list(roll_buf))
    ax1.set_xlim(max(0, len(xs)-WINDOW), max(WINDOW, len(xs)))
    
    if pitch_buf:
        update_3d_table(pitch_buf[-1], roll_buf[-1])
        
    return (line_pitch, line_roll, table)

ani = animation.FuncAnimation(fig, update, init_func=init, interval=UPDATE_INTERVAL, blit=False)
plt.tight_layout()
plt.show()
