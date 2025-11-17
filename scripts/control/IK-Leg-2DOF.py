
import matplotlib.pyplot as plt
import numpy as np

# สร้างฟังก์ชัน IK
def ik_2dof(x, y, l1, l2, elbow_up=True):
    r = np.sqrt(x**2 + y**2)
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2*l1*l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # ป้องกัน error จาก float
    theta2 = np.arccos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2
    theta1 = np.arctan2(y, x) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))
    return theta1, theta2

# พารามิเตอร์ลิงก์
l1 = 140.0
l2 = 129.0

# จุดทดสอบ (x, y)
x_target = 5.0
y_target = -200.0

# คำนวณ IK
theta1, theta2 = ik_2dof(x_target, y_target, l1, l2, elbow_up=True)

# หาพิกัด p1, p2, p3
p1 = np.array([0, 0])
p2 = p1 + [l1*np.cos(theta1), l1*np.sin(theta1)]
p3 = p2 + [l2*np.cos(theta1+theta2), l2*np.sin(theta1+theta2)]

# วาดภาพ
plt.figure(figsize=(6,6))
# ลิงก์ 1
plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'o-', linewidth=3, color='blue', label='Link 1')
# ลิงก์ 2
plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'o-', linewidth=3, color='green', label='Link 2')
# จุดเป้าหมาย
plt.plot(x_target, y_target, 'rx', markersize=10, label='Target (x,y)')

# กำหนดจุด
plt.text(p1[0], p1[1]-0.05, 'p1', ha='center')
plt.text(p2[0], p2[1]+0.05, 'p2', ha='center')
plt.text(p3[0], p3[1]+0.05, 'p3', ha='center')

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title(f"IK Result: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°")
plt.show()
