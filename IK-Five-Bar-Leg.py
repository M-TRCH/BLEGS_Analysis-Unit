import numpy as np
import matplotlib.pyplot as plt

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

# --- 2. ทดสอบ IK ที่ท่ายืน (Home Position) ---
print("--- 🤖 ทดสอบ IK ที่ท่ายืน P=(0, -200) ---")
print("--- (ใช้การตั้งค่าแบบ 'ไขว้-สลับด้าน' L1-Back, L2-Front) ---")
target_Px, target_Py = 0, -200

(th1_rad, th2_rad, 
 K1_pos, K2_pos) = calculate_ik_crossed_reversed(target_Px, target_Py)

if not np.isnan(th1_rad):
    print(f"เป้าหมาย: ({target_Px}, {target_Py}) mm")
    print(f"  Theta 1 (M1): {np.rad2deg(th1_rad):.2f} องศา (Knee Back config)")
    print(f"  Theta 2 (M2): {np.rad2deg(th2_rad):.2f} องศา (Knee Front config)")

# --- 3. สร้างข้อมูลสำหรับพล็อตกราฟ Workspace ---
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

# --- 4. พล็อตกราฟด้วย Matplotlib (⭐ อัปเกรดแล้ว) ---
plt.figure(figsize=(12, 12))
plt.scatter(reachable_x, reachable_y, s=2, alpha=0.2, label='Reachable Workspace', color='lightblue')
plt.plot(M1_X, M1_Y, 'ro', markersize=10, label=f'Motor 1 ({M1_X}, {M1_Y})')
plt.plot(M2_X, M2_Y, 'bo', markersize=10, label=f'Motor 2 ({M2_X}, {M2_Y})')
plt.plot(target_Px, target_Py, 'g*', markersize=15, label=f'Home Position (0, -200)')

if not np.isnan(th1_rad):
    # ⭐ 1. อัปเกรด Legend ให้บอกความยาวลิงก์
    plt.plot([M1_X, K1_pos[0]], [M1_Y, K1_pos[1]], color='red', linewidth=3, label=f'L1 (Back) [{L1}mm]')
    plt.plot([M2_X, K2_pos[0]], [M2_Y, K2_pos[1]], color='blue', linewidth=3, label=f'L2 (Front) [{L2}mm]')
    plt.plot([K1_pos[0], target_Px], [K1_pos[1], target_Py], color='darkred', linestyle='--', linewidth=2, label=f'L3 [{L3}mm]')
    plt.plot([K2_pos[0], target_Px], [K2_pos[1], target_Py], color='darkblue', linestyle='--', linewidth=2, label=f'L4 [{L4}mm]')
    
    # ⭐ 2. อัปเกรด Legend ให้บอกจุดเข่า
    plt.plot(K1_pos[0], K1_pos[1], 'rx', markersize=8, label='Knee 1 (K1)')
    plt.plot(K2_pos[0], K2_pos[1], 'bx', markersize=8, label='Knee 2 (K2)')
    
    # ⭐ 3. เพิ่ม Text แสดงค่ามุมที่คำนวณได้
    theta1_deg = np.rad2deg(th1_rad)
    theta2_deg = np.rad2deg(th2_rad)
    plt.text(M1_X, M1_Y + 10, f'θ1: {theta1_deg:.2f}°', color='red', ha='center', fontsize=12, weight='bold')
    plt.text(M2_X, M2_Y + 10, f'θ2: {theta2_deg:.2f}°', color='blue', ha='center', fontsize=12, weight='bold')

# ตั้งค่ากราฟ
plt.title(f'IK Workspace (L1-Back, L2-Front) @ Home (0,-200)')
plt.xlabel('Px (mm) - (หน้า/หลัง)')
plt.ylabel('Py (mm) - (ขึ้น/ลง)')
plt.grid(True, linestyle='--', alpha=0.5)
plt.axhline(0, color='black', linewidth=1.0)
plt.axvline(0, color='black', linewidth=1.0)
plt.legend(loc='lower left') # ย้าย Legend ไปมุมล่างซ้าย
plt.axis('equal')
plt.show()

print("--- สร้างกราฟเสร็จสิ้น ---")