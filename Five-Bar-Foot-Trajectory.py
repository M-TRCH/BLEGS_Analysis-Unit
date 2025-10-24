import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

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

# Additional downward shift for the entire foot path (mm)
# Previously -40; move down by another 50 => -90
foot_y_offset = -90

# Generate foot trajectory (shifted to Q3/Q4: y <= 0)
half = n_points // 2
# Lift phase (sinusoidal arc)
tx1 = np.linspace(-stride_length/2, stride_length/2, half)
# Keep foot below x-axis: range from -step_height up to 0
ty1 = -step_height + step_height * np.sin(np.pi * np.linspace(0, 1, half))
# Apply global downward offset
ty1 = ty1 + foot_y_offset
# Return phase (straight line)
tx2 = np.linspace(stride_length/2, -stride_length/2, half)
ty2 = -step_height * np.ones_like(tx2)
# Apply global downward offset
ty2 = ty2 + foot_y_offset

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

## Animation over the whole trajectory
# We'll animate the linkage across all points to visualize reachability.

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

## Prepare artists for animation
# Link lines: L1 (M1->A), L3 (A->foot), L2 (M2->B), L4 (B->foot)
line_L1, = ax.plot([], [], 'b-', lw=2, label='L1/L3 (left)')
line_L3, = ax.plot([], [], 'b--', lw=1)
line_L2, = ax.plot([], [], 'r-', lw=2, label='L2/L4 (right)')
line_L4, = ax.plot([], [], 'r--', lw=1)

# Foot marker and trace
foot_pt, = ax.plot([], [], 'o', color='green', markersize=6, label='Foot (reachable)')
trace_line, = ax.plot([], [], color='gray', alpha=0.5, lw=1, label='Trace')

# Unreachable proxy for legend
unreach_proxy, = ax.plot([], [], 'o', color='crimson', markersize=6, label='Foot (unreachable)')

def init_anim():
    for ln in (line_L1, line_L3, line_L2, line_L4):
        ln.set_data([], [])
    foot_pt.set_data([], [])
    trace_line.set_data([], [])
    return line_L1, line_L3, line_L2, line_L4, foot_pt, trace_line

def update_anim(i):
    x, y = foot_path[i]
    res = solve_ik(x, y)

    # Update trace up to current i
    trace_line.set_data(foot_x[: i+1], foot_y[: i+1])

    if res is None:
        # Hide links, mark foot as unreachable
        for ln in (line_L1, line_L3, line_L2, line_L4):
            ln.set_data([np.nan], [np.nan])
        foot_pt.set_data([x], [y])
        foot_pt.set_color('crimson')
        ax.set_title(f'5-Bar Separated-Shaft Linkage Foot Trajectory  |  i={i+1}/{len(foot_path)}  |  unreachable')
    else:
        A, B = res
        # Set link segments
        line_L1.set_data([M1[0], A[0]], [M1[1], A[1]])
        line_L3.set_data([A[0], x], [A[1], y])
        line_L2.set_data([M2[0], B[0]], [M2[1], B[1]])
        line_L4.set_data([B[0], x], [B[1], y])
        # Foot
        foot_pt.set_data([x], [y])
        foot_pt.set_color('green')
        ax.set_title(f'5-Bar Separated-Shaft Linkage Foot Trajectory  |  i={i+1}/{len(foot_path)}  |  reachable')

    return line_L1, line_L3, line_L2, line_L4, foot_pt, trace_line

# Plot settings
ax.set_aspect('equal')
ax.set_xlim(-100, 100)
ax.set_ylim(-160, 40)
ax.set_title('5-Bar Separated-Shaft Linkage Foot Trajectory')
# Build a stable legend with proxies so colors remain meaningful during updates
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles + [unreach_proxy], labels + ['Foot (unreachable)'])

# Create and run animation
anim = animation.FuncAnimation(
    fig,
    update_anim,
    init_func=init_anim,
    frames=len(foot_path),
    interval=40,
    blit=False,
    repeat=True,
)

plt.tight_layout()
plt.show()