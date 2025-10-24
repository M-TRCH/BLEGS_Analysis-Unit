import numpy as np
import matplotlib.pyplot as plt

# Parameters
L1 = L2 = 105  # Upper links (mm)
L3 = L4 = 145  # Lower links (mm)
d = 85  # Motor base spacing (mm)
M1 = np.array([-d/2, 0])
M2 = np.array([d/2, 0])

# Gait profile parameters
stride_length = 120  # Total foot x travel
step_height = 60     # Foot lift height
n_points = 200

# Generate foot trajectory (shifted to Q3/Q4: y <= 0)
half = n_points // 2
# Lift phase (sinusoidal arc)
tx1 = np.linspace(-stride_length/2, stride_length/2, half)
# Keep foot below x-axis: range from -step_height up to 0
ty1 = -step_height + step_height * np.sin(np.pi * np.linspace(0, 1, half))
# Return phase (straight line)
tx2 = np.linspace(stride_length/2, -stride_length/2, half)
ty2 = -step_height * np.ones_like(tx2)

foot_x = np.concatenate([tx1, tx2])
foot_y = np.concatenate([ty1, ty2])
foot_path = np.vstack((foot_x, foot_y)).T

# Plot
fig, ax = plt.subplots(figsize=(10, 6))

# Draw axes
ax.axhline(0, color='green', linewidth=1)  # x-axis
ax.axvline(0, color='blue', linewidth=1)   # y-axis

# Draw trajectory
ax.plot(tx1, ty1, color='black', label='Lift phase (sine)')
ax.plot(tx2, ty2, color='red', label='Return phase (line)')

# Mark motor bases
ax.plot(*M1, 'ko')
ax.plot(*M2, 'ko')
ax.text(*M1 + np.array([-10, 5]), 'M1', fontsize=10)
ax.text(*M2 + np.array([5, 5]), 'M2', fontsize=10)

# Draw linkage lines for a sample step (ensure a negative y point)
idx = n_points // 6
foot = foot_path[idx]

# Inverse kinematics function
def solve_ik(x, y):
    P = np.array([x, y], dtype=float)

    def circle_intersection(p0, r0, p1, r1, eps=1e-9):
        v = p1 - p0
        d = np.linalg.norm(v)
        # ศูนย์กลางซ้อน/เกือบซ้อน หรือซ้อนแบบไม่ตัด
        if d < eps:
            return None
        if d > r0 + r1 or d < abs(r0 - r1):
            return None

        a = (r0**2 - r1**2 + d**2) / (2.0 * d)
        inside = r0**2 - a**2
        if inside < 0:
            inside = 0.0  # ป้องกันลบเล็กๆ จาก floating error
        h = np.sqrt(inside)

        mid = p0 + (a / d) * v
        # เวกเตอร์ตั้งฉากกับ v
        offset = np.array([-v[1], v[0]]) / d
        return mid + h * offset, mid - h * offset

    sol1 = circle_intersection(M1, L1, P, L3)
    sol2 = circle_intersection(M2, L2, P, L4)
    if sol1 is None or sol2 is None:
        return None

    # เลือก elbow-down (y ต่ำกว่า)
    A = sol1[0] if sol1[0][1] < sol1[1][1] else sol1[1]
    B = sol2[0] if sol2[0][1] < sol2[1][1] else sol2[1]
    return A, B

res = solve_ik(*foot)
if res:
    A, B = res
    ax.plot([M1[0], A[0]], [M1[1], A[1]], 'b-')  # L1
    ax.plot([M2[0], B[0]], [M2[1], B[1]], 'r-')  # L2
    ax.plot([A[0], foot[0]], [A[1], foot[1]], 'b--')  # L3
    ax.plot([B[0], foot[0]], [B[1], foot[1]], 'r--')  # L4
    ax.plot(*foot, 'go', label='Foot position')

# Plot settings
ax.set_aspect('equal')
ax.set_xlim(-100, 100)
ax.set_ylim(-160, 40)
ax.set_title('5-Bar Separated-Shaft Linkage Foot Trajectory')
ax.legend()
plt.tight_layout()
plt.show()