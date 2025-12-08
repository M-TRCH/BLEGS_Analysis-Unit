# BLEGS Analysis Unit

โปรเจ็กต์วิเคราะห์จลนศาสตร์และควบคุมสำหรับหุ่นยนต์ขาสองขา (Bipedal Legged Robot)

# BLEGS Analysis Unit

โปรเจ็กต์วิเคราะห์จลนศาสตร์และควบคุมสำหรับหุ่นยนต์ขาสองขา (Bipedal Legged Robot)

## 📁 โครงสร้างโฟลเดอร์

```
BLEGS_Analysis-Unit/
├── docs/                          # 📚 เอกสาร LaTeX
│   ├── forward-kinematics-5bar.tex           # Forward Kinematics
│   ├── inverse-kinematics-analytical.tex     # Inverse Kinematics (Analytical)
│   ├── figures/                   # รูปภาพสำหรับเอกสาร
│   └── output/                    # ไฟล์ PDF ที่ compile แล้ว
│
├── scripts/                       # 🐍 Python Scripts
│   ├── kinematics/               # จลนศาสตร์
│   │   ├── IK-Five-Bar-Leg-Analytical.py    # IK แบบ Analytical (4 configs)
│   │   ├── IK-Five-Bar-Leg-Numerical.py     # IK แบบ Numerical
│   │   ├── IK-Five-Bar-Leg-Animation.py     # Animation
│   │   ├── IK-Five-Bar-Leg.py
│   │   ├── Five-Bar-Workspace.py            # Workspace Analysis
│   │   └── __init__.py
│   │
│   ├── vision/                   # ประมวลผลภาพ
│   │   ├── AR-Tag-Detection.py
│   │   ├── Color-Blob-Detection.py
│   │   ├── Create-AR-Tag.py
│   │   ├── Hought-Circle.py
│   │   └── Open-Cam.py
│   │
│   ├── control/                  # ควบคุม
│   │   ├── Leg-Station-Control.py
│   │   └── IK-Leg-2DOF.py
│   │
│   └── analysis/                 # วิเคราะห์ข้อมูล
│       ├── Actuation-Weight-Scatter.py
│       ├── DOF-Weight-Scatter.py
│       ├── TW-Weight-Scatter.py
│       ├── TW-Bar-Chart.py
│       └── Sensor-Usage-Matrix.py
│
├── test/                         # 🧪 สคริปต์ทดสอบ
│   ├── test_adaptive_blob.py
│   ├── test_camera_info.py
│   ├── check_video.py
│   ├── check_demo_video.py
│   ├── compare_recording_methods.py
│   └── analyze_results.py
│
├── .vscode/                      # ⚙️ การตั้งค่า VS Code
│   └── settings.json             # LaTeX Workshop + Python
│
├── __pycache__/                  # Python cache (ignored)
│
├── .gitignore                    # ไฟล์ที่ไม่ต้อง commit
└── README.md                     # ไฟล์นี้
```

## 📚 เอกสาร LaTeX

### 1. Forward Kinematics (FK)
- **ไฟล์:** `docs/forward-kinematics-5bar.tex`
- **เนื้อหา:** การวิเคราะห์จลนศาสตร์เชิงหน้าสำหรับ 5-Bar Linkage
  - พารามิเตอร์ทางจลนศาสตร์
  - การคำนวณ FK (3 ขั้นตอน)
  - การคำนวณ Jacobian Matrix
- **คอมไพล์:** XeLaTeX (รองรับภาษาไทย)
- **Output:** `docs/output/forward-kinematics-5bar.pdf`

### 2. Inverse Kinematics (IK) - Analytical Method
- **ไฟล์:** `docs/inverse-kinematics-analytical.tex`
- **เนื้อหา:** การวิเคราะห์ IK แบบ Analytical
  - วิธีการแก้สมการทางเรขาคณิต (5 ขั้นตอน)
  - ทั้ง 4 Configurations (Down-Down, Down-Up, Up-Down, Up-Up)
  - ผลการทดสอบและการเปรียบเทียบ
  - ความแม่นยำระดับ $10^{-14}$ mm
- **คอมไพล์:** XeLaTeX
- **Output:** `docs/output/inverse-kinematics-analytical.pdf`

### การคอมไพล์เอกสาร

**วิธีที่ 1: ใช้ VS Code LaTeX Workshop**
1. เปิดไฟล์ `.tex` ใน VS Code
2. กด `Ctrl+S` (Save) - จะ auto-build
3. หรือกด `Ctrl+Alt+B` (Build)
4. ดู PDF ด้วย `Ctrl+Alt+V` (View)

**วิธีที่ 2: ใช้ Terminal**
```powershell
cd docs
xelatex -interaction=nonstopmode fk.tex
```

## 🐍 Python Scripts

### 1. Kinematics (จลนศาสตร์)

#### IK-Five-Bar-Leg-Analytical.py ⭐
- **วัตถุประสงค์:** คำนวณ Inverse Kinematics แบบ Analytical
- **Features:**
  - รองรับ **4 Configurations** (Elbow C/D: Up/Down)
  - ความแม่นยำ: **6.36×10⁻¹⁴ mm** (machine precision)
  - **ไม่ต้องค่าเดาเริ่มต้น**
  - แสดงผลทั้ง 4 configurations พร้อมตรวจสอบ FK
  - ระบุ Valid/Invalid solutions
- **การใช้งาน:**
  ```python
  python scripts/kinematics/IK-Five-Bar-Leg-Analytical.py
  ```

#### IK-Five-Bar-Leg-Numerical.py
- **วัตถุประสงค์:** คำนวณ IK แบบ Numerical (Newton-Raphson)
- **Features:**
  - ใช้ Jacobian ที่ derive แล้ว
  - รวดเร็วและแม่นยำ (error ~10⁻⁸ mm)
  - ต้องมีค่าเดาเริ่มต้นที่ดี
  - รองรับการปรับมุมเข้าช่วง ±180°

#### Five-Bar-Workspace.py
- **วัตถุประสงค์:** วิเคราะห์ Workspace ของกลไก
- **Features:**
  - แสดง Reachable Workspace
  - ตรวจจับ Singularity Zones (3 ประเภท)
  - Visualization ด้วย matplotlib

#### IK-Five-Bar-Leg-Animation.py
- **วัตถุประสงค์:** สร้าง Animation การเดินของขา
- **Features:**
  - แสดงการเคลื่อนไหวแบบ real-time
  - Walking trajectory simulation

### 2. Vision (ประมวลผลภาพ)
- **AR-Tag-Detection.py** - ตรวจจับ AR Tag
- **Color-Blob-Detection.py** - ตรวจจับสีแบบ Adaptive
- **Hought-Circle.py** - หาวงกลมด้วย Hough Transform

### 3. Control (การควบคุม)
- **Leg-Station-Control.py** - ควบคุมขาหุ่นยนต์
- **IK-Leg-2DOF.py** - IK สำหรับขา 2 DOF

### 4. Analysis (การวิเคราะห์)
- **Actuation-Weight-Scatter.py** - วิเคราะห์น้ำหนัก Actuator
- **TW-Bar-Chart.py** - กราฟแท่งน้ำหนัก
- **Sensor-Usage-Matrix.py** - Matrix การใช้งาน Sensor

## 🔧 การติดตั้ง

### Requirements
```powershell
pip install numpy scipy matplotlib opencv-python pandas pyserial
```

### LaTeX (MiKTeX)
- ดาวน์โหลด MiKTeX จาก https://miktex.org/
- ติดตั้งฟอนต์ TH SarabunPSK (สำหรับภาษาไทย)

### VS Code Extensions แนะนำ
- **LaTeX Workshop** - สำหรับเขียน LaTeX
- **Python** - สำหรับเขียน Python
- **Pylance** - Python Language Server

## 📖 การใช้งาน

### ตัวอย่าง 1: IK Analytical (แนะนำ)
```python
cd scripts/kinematics
python IK-Five-Bar-Leg-Analytical.py
```
**Output:**
```
🎯 เป้าหมาย (Target Pose): [0. -200.] mm

Config 1: Elbow C ลง, D ลง (Down-Down) 🦵
✅ พบคำตอบ!
   Theta A (M1): -119.5309° → -119.5309°
   Theta B (M2):  -37.6760° →  -37.6760°
   |Error|: 6.36e-14 mm ✅ (Valid)
```

### ตัวอย่าง 2: IK Numerical
```python
cd scripts/kinematics
python IK-Five-Bar-Leg-Numerical.py
```

### ตัวอย่าง 3: Workspace Analysis
```python
python scripts/kinematics/Five-Bar-Workspace.py
```
แสดงกราฟ Workspace พร้อม Singularity Zones

### ตัวอย่าง 4: เขียน Code เอง
```python
import sys
sys.path.append('./scripts/kinematics')
from IK_Five_Bar_Leg_Analytical import calculate_ik_analytical
import numpy as np

# กำหนดเป้าหมาย
target = np.array([0.0, -200.0])  # (x, y) mm

# คำนวณ IK - Config 1 (Down-Down)
angles_rad = calculate_ik_analytical(target, elbow_C_down=True, elbow_D_down=True)
angles_deg = np.rad2deg(angles_rad)

print(f"Theta A: {angles_deg[0]:.2f}°")
print(f"Theta B: {angles_deg[1]:.2f}°")
```

## 🎯 ผลการทดสอบ IK

สำหรับเป้าหมาย (0, -200) mm:

| Configuration | θA (°) | θB (°) | Error (mm) | Valid | หมายเหตุ |
|--------------|--------|--------|------------|-------|----------|
| **Down-Down** ⭐ | -119.53| -37.68 | 6.36e-14   | ✅ | แนะนำ - ท่ามาตรฐาน |
| Down-Up      | -139.91| -166.32| 1.68e-13   | ✅ | เสี่ยง singularity |
| Up-Down      | -16.06 | -37.68 | 4.26e-14   | ✅ | ไม่เหมาะพยุงน้ำหนัก |
| Up-Up        | -      | -      | 339        | ❌ | นอก Workspace |

### 🔍 ข้อสังเกต:
- กลไก 5-Bar Linkage มี **3 คำตอบที่ถูกต้อง** สำหรับจุดเดียวกัน
- **Config 1 (Down-Down)** เหมาะสมที่สุดสำหรับการใช้งานจริง
- ความแม่นยำระดับ **machine precision** (~10⁻¹⁴ mm)
- การเลือก Configuration ขึ้นอยู่กับ:
  - ท่าก่อนหน้า (ความต่อเนื่อง)
  - การหลีกเลี่ยง Singularity
  - ประหยัดพลังงาน

## 📝 หมายเหตุ

- ไฟล์ LaTeX ใช้ **XeLaTeX** เพื่อรองรับภาษาไทย (ฟอนต์ TH SarabunPSK)
- Python scripts ใช้ **Python 3.11+**
- การคำนวณ IK มีความแม่นยำระดับ **machine precision** (~10⁻¹⁴ mm)
- สำหรับการควบคุมจริง แนะนำใช้ **Config 1 (Down-Down)**
- ทุก errors ใน `scripts/` ได้รับการแก้ไขแล้ว ✅

## 🔗 เอกสารอ้างอิง

### เอกสารภายใน
- `docs/Phase1_Kinematics/Phase1.1_Forward_Kinematics_5Bar.pdf` - FK และ Jacobian
- `docs/Phase1_Kinematics/Phase1.2_Inverse_Kinematics_Analytical.pdf` - IK Analytical พร้อมผลการทดสอบ
- `docs/Phase2_Dynamics/Phase2.1_Static_Torque_Analysis.pdf` - Static Torque Analysis
- `docs/Phase2_Dynamics/Phase2.2_Dynamic_Torque_Analysis.pdf` - Dynamic Torque Analysis
- `docs/Phase3_Simulation/Phase3.1_Gait_Control_Simulation.pdf` - Gait Control Simulation with PyBullet

### Code Implementation
- `scripts/kinematics/IK-Five-Bar-Leg-Analytical.py` - Implementation ตามเอกสาร
- `scripts/kinematics/IK-Five-Bar-Leg-Numerical.py` - Newton-Raphson solver
- `scripts/simulation/gait_control/gait_control_trot.py` - PyBullet Trot Gait Simulation

## 📊 สถิติโปรเจ็กต์

- **เอกสาร LaTeX:** 5 ไฟล์ (Phase 1-3)
- **Python Scripts:** 20+ ไฟล์
  - Kinematics: 6 ไฟล์
  - Vision: 5 ไฟล์
  - Control: 5 ไฟล์
  - Analysis: 5 ไฟล์
  - Simulation: 2 ไฟล์ (Phase 3 Gait Control)
- **Test Scripts:** 6 ไฟล์
- **URDF Models:** 1 quadruped model (16 joints)
- **Configurations tested:** 4 (3 valid, 1 invalid)
- **Phase Progress:** 1, 2, 3, 4 Complete ✅ | Phase 5 Planning 📋

## 👨‍💻 ผู้พัฒนา

นายธีรโชติ เมืองจำนงค์

## 📅 อัพเดทล่าสุด

8 ธันวาคม 2568 - เพิ่ม Phase 3 Gait Control Simulation พร้อมเอกสาร LaTeX
