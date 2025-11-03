import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. กำหนดพารามิเตอร์คงที่ของหุ่นยนต์ (หน่วย mm) ---
L1 = 105.0
L2 = 105.0
L3 = 145.0
L4 = 145.0
D_HIP = 85.0

M1_X = -D_HIP / 2.0  # มอเตอร์ซ้าย (-42.5)
M1_Y = 0.0
M2_X = D_HIP / 2.0   # มอเตอร์ขวา (42.5)
M2_Y = 0.0

def calculate_ik_crossed_reversed(Px, Py):
    # (ฟังก์ชัน IK เหมือนเดิมทุกประการ)
    try:
        D1_sq = (Px - M1_X)**2 + (Py - M1_Y)**2
        D2_sq = (Px - M2_X)**2 + (Py - M2_Y)**2
        D1 = np.sqrt(D1_sq)
        D2 = np.sqrt(D2_sq)

        if (D1 > L1 + L3) or (D1 < abs(L1 - L3)) or \
           (D2 > L2 + L4) or (D2 < abs(L2 - L4)):
            return np.nan, np.nan, np.nan, np.nan

        alpha1 = np.arctan2(Py - M1_Y, Px - M1_X)
        alpha2 = np.arctan2(Py - M2_Y, Px - M2_X)

        cos_beta1_arg = np.clip((L1**2 + D1_sq - L3**2) / (2 * L1 * D1), -1.0, 1.0)
        cos_beta2_arg = np.clip((L2**2 + D2_sq - L4**2) / (2 * L2 * D2), -1.0, 1.0)
        
        beta1 = np.arccos(cos_beta1_arg)
        beta2 = np.arccos(cos_beta2_arg)

        # L1 (M1) ใช้ท่า Knee Back (ชี้ไปหลัง)
        theta1 = alpha1 - beta1
        # L2 (M2) ใช้ท่า Knee Front (ชี้ไปหน้า)
        theta2 = alpha2 + beta2

        K1_x = M1_X + L1 * np.cos(theta1)
        K1_y = M1_Y + L1 * np.sin(theta1)
        K2_x = M2_X + L2 * np.cos(theta2)
        K2_y = M2_Y + L2 * np.sin(theta2)
        
        return theta1, theta2, (K1_x, K1_y), (K2_x, K2_y)
    
    except (ValueError, ZeroDivisionError):
        return np.nan, np.nan, np.nan, np.nan

# --- 2. สร้าง Walking Trajectory (วิถีการเดิน) ---
def generate_walking_trajectory(num_steps=60, lift_height=50, drag_depth=20, step_forward=50):
    """
    สร้างวิถีการเดินแบบ Sine Path ประกบบน-ล่าง
    Y เป็นฟังก์ชัน sine ของ X (Y = f(X))
    อ้างอิงจากท่า Home (0, -200)
    
    num_steps: จำนวน frame ในแอนิเมชัน
    lift_height: ยกเท้าสูงกว่า home (mm) - ค่าเริ่มต้น 50mm
    drag_depth: ลากเท้าต่ำกว่า home (mm) - ค่าเริ่มต้น 20mm  
    step_forward: ก้าวไปหน้า-หลัง (mm) - ค่าเริ่มต้น 50mm
    """
    trajectory = []
    home_y = -200  # ท่ายืนอ้างอิง
    
    for i in range(num_steps):
        # t เป็นพารามิเตอร์สำหรับการเคลื่อนที่ (0 → 2π)
        t = 2 * np.pi * i / num_steps
        
        # แกน X: เคลื่อนที่เป็นเส้นตรง จาก -step_forward → 0 → +step_forward → 0 → -step_forward
        # ใช้ cosine เพื่อเริ่มที่ -step_forward
        px = -step_forward * np.cos(t)
        
        # แกน Y: เป็น sine wave ตามค่า X
        # คำนวณ sine phase จาก X position
        # เมื่อ px = -50 → 0 → +50 → 0 → -50 (1 รอบ)
        # sine_phase จะวิ่ง 0 → π → 2π → 3π → 4π แต่เราต้องการแค่ 0 → 2π
        
        # ใช้ t โดยตรงสำหรับ Y axis
        sin_y = np.sin(t)
        
        if sin_y >= 0:
            # ครึ่งบน: ยกเท้าขึ้น
            py = home_y + lift_height * sin_y
        else:
            # ครึ่งล่าง: ลากเท้าลง
            py = home_y + drag_depth * sin_y
        
        trajectory.append((px, py))
    
    return trajectory

# --- 3. ทดสอบ IK ที่ท่ายืน (Home Position) ---
print("--- 🤖 ทดสอบ IK ที่ท่ายืน P=(0, -200) ---")
print("--- (ใช้การตั้งค่าแบบ 'ไขว้-สลับด้าน' L1-Back, L2-Front) ---")
target_Px, target_Py = 0, -200

(th1_rad, th2_rad, 
 K1_pos, K2_pos) = calculate_ik_crossed_reversed(target_Px, target_Py)

if not np.isnan(th1_rad):
    print(f"เป้าหมาย: ({target_Px}, {target_Py}) mm")
    print(f"  Theta 1 (M1): {np.rad2deg(th1_rad):.2f} องศา (Knee Back config)")
    print(f"  Theta 2 (M2): {np.rad2deg(th2_rad):.2f} องศา (Knee Front config)")

# --- 4. สร้าง Walking Trajectory ---
print("\n--- 🚶 กำลังสร้างวิถีการเดิน... ---")
walking_path = generate_walking_trajectory(num_steps=60, lift_height=50, drag_depth=20, step_forward=50)
print(f"สร้างวิถีการเดินเสร็จสิ้น: {len(walking_path)} frames")
print(f"  - ยกเท้าสูงสุด: -150 mm (สูงกว่า home 50mm)")
print(f"  - ลากเท้าต่ำสุด: -220 mm (ต่ำกว่า home 20mm)")
print(f"  - ก้าวหน้า-หลัง: ±50 mm")

# --- 5. สร้างข้อมูลสำหรับพล็อตกราฟ Workspace ---
# (ส่วนนี้เหมือนเดิม)
print("\n--- 📈 กำลังสร้างกราฟ Workspace... ---")
x_range = np.linspace(-250, 250, 100) 
y_range = np.linspace(-300, 50, 100)
reachable_x = []
reachable_y = []
for px in x_range:
    for py in y_range:
        t1, t2, _, _ = calculate_ik_crossed_reversed(px, py)
        if not np.isnan(t1):
            reachable_x.append(px)
            reachable_y.append(py)

# --- 5. สร้างข้อมูลสำหรับพล็อตกราฟ Workspace ---
# (ส่วนนี้เหมือนเดิม)
print("\n--- 📈 กำลังสร้างกราฟ Workspace... ---")
x_range = np.linspace(-250, 250, 100) 
y_range = np.linspace(-300, 50, 100)
reachable_x = []
reachable_y = []
for px in x_range:
    for py in y_range:
        t1, t2, _, _ = calculate_ik_crossed_reversed(px, py)
        if not np.isnan(t1):
            reachable_x.append(px)
            reachable_y.append(py)

# --- 6. สร้างแอนิเมชันด้วย Matplotlib (⭐ อัปเกรดใหม่) ---
print("\n--- 🎬 กำลังสร้างแอนิเมชันการเดิน... ---")

fig, ax = plt.subplots(figsize=(12, 12))

# พล็อต Workspace (พื้นหลัง)
ax.scatter(reachable_x, reachable_y, s=2, alpha=0.15, color='lightblue', label='Reachable Workspace')
ax.plot(M1_X, M1_Y, 'ro', markersize=12, label=f'Motor 1 ({M1_X}, {M1_Y})', zorder=5)
ax.plot(M2_X, M2_Y, 'bo', markersize=12, label=f'Motor 2 ({M2_X}, {M2_Y})', zorder=5)

# วาดวิถีการเดินทั้งหมด (เส้นประสีเขียว)
path_x = [p[0] for p in walking_path]
path_y = [p[1] for p in walking_path]
ax.plot(path_x, path_y, 'g-', alpha=0.6, linewidth=2.5, label='Sine Walking Path')

# วาดท่า Home Position
ax.plot(0, -200, 'mo', markersize=10, label='Home (0, -200)', zorder=5)
ax.axhline(-200, color='magenta', linewidth=1, linestyle=':', alpha=0.5, label='Home Level')

# สร้างออบเจ็กต์สำหรับแอนิเมชัน
link1, = ax.plot([], [], 'r-', linewidth=4, label='L1 (Back)', zorder=4)
link2, = ax.plot([], [], 'b-', linewidth=4, label='L2 (Front)', zorder=4)
link3, = ax.plot([], [], 'darkred', linestyle='--', linewidth=3, label='L3', zorder=3)
link4, = ax.plot([], [], 'darkblue', linestyle='--', linewidth=3, label='L4', zorder=3)
knee1, = ax.plot([], [], 'rx', markersize=10, label='Knee 1', zorder=5)
knee2, = ax.plot([], [], 'bx', markersize=10, label='Knee 2', zorder=5)
foot, = ax.plot([], [], 'g*', markersize=20, label='Foot Position', zorder=6)

# Text สำหรับแสดงข้อมูล
info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                    fontsize=12, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# ตั้งค่ากราฟ
ax.set_title('🚶 Sine Path Walking Animation - Five Bar Linkage Robot', fontsize=16, weight='bold')
ax.set_xlabel('Px (mm) - (front/back)', fontsize=12)
ax.set_ylabel('Py (mm) - (up/down)', fontsize=12)
ax.grid(True, linestyle='--', alpha=0.3)
ax.axhline(0, color='black', linewidth=1.0)
ax.axvline(0, color='black', linewidth=1.0)
ax.legend(loc='lower left', fontsize=9)
ax.axis('equal')
ax.set_xlim(-250, 250)
ax.set_ylim(-350, 100)

# ฟังก์ชัน init สำหรับ animation
def init():
    link1.set_data([], [])
    link2.set_data([], [])
    link3.set_data([], [])
    link4.set_data([], [])
    knee1.set_data([], [])
    knee2.set_data([], [])
    foot.set_data([], [])
    info_text.set_text('')
    return link1, link2, link3, link4, knee1, knee2, foot, info_text

# ฟังก์ชัน animate สำหรับอัพเดตแต่ละ frame
def animate(frame):
    # ดึงตำแหน่งเป้าหมายจากวิถีการเดิน
    px, py = walking_path[frame]
    
    # คำนวณ IK
    th1, th2, K1_pos, K2_pos = calculate_ik_crossed_reversed(px, py)
    
    if not np.isnan(th1):
        # อัพเดตลิงก์และจุดต่าง ๆ
        link1.set_data([M1_X, K1_pos[0]], [M1_Y, K1_pos[1]])
        link2.set_data([M2_X, K2_pos[0]], [M2_Y, K2_pos[1]])
        link3.set_data([K1_pos[0], px], [K1_pos[1], py])
        link4.set_data([K2_pos[0], px], [K2_pos[1], py])
        knee1.set_data([K1_pos[0]], [K1_pos[1]])
        knee2.set_data([K2_pos[0]], [K2_pos[1]])
        foot.set_data([px], [py])
        
        # อัพเดตข้อมูลแสดง
        info_text.set_text(
            f'Frame: {frame+1}/{len(walking_path)}\n'
            f'Position: ({px:.1f}, {py:.1f}) mm\n'
            f'θ1: {np.rad2deg(th1):.1f}°\n'
            f'θ2: {np.rad2deg(th2):.1f}°'
        )
    
    return link1, link2, link3, link4, knee1, knee2, foot, info_text

# สร้างแอนิเมชัน
anim = FuncAnimation(fig, animate, init_func=init, 
                     frames=len(walking_path), 
                     interval=50,  # 50ms ต่อ frame (20 FPS)
                     blit=True, 
                     repeat=True)

print("--- ✅ แอนิเมชันพร้อมแล้ว! กำลังแสดงผล... ---")
plt.show()

print("--- สร้างแอนิเมชันเสร็จสิ้น ---")