# BLEGS Analysis Unit

โปรเจ็กต์วิเคราะห์จลนศาสตร์และควบคุมสำหรับหุ่นยนต์ขาสองขา (Bipedal Legged Robot)

## 📁 โครงสร้างโฟลเดอร์

```
BLEGS_Analysis-Unit/
├── docs/                          # เอกสาร LaTeX และ PDF
│   ├── fk.tex                     # Forward Kinematics
│   ├── figures/                   # รูปภาพสำหรับเอกสาร
│   └── output/                    # ไฟล์ PDF ที่ compile แล้ว
│
├── scripts/                       # Python scripts ทั้งหมด
│   ├── kinematics/               # สคริปต์เกี่ยวกับจลนศาสตร์
│   │   ├── IK-Five-Bar-Leg-Analytical.py
│   │   ├── IK-Five-Bar-Leg-Numerical.py
│   │   └── Five-Bar-Workspace.py
│   │
│   ├── vision/                   # สคริปต์ประมวลผลภาพ
│   │   ├── AR-Tag-Detection.py
│   │   ├── Color-Blob-Detection.py
│   │   ├── Hought-Circle.py
│   │   └── Open-Cam.py
│   │
│   ├── control/                  # สคริปต์ควบคุม
│   │   ├── Leg-Station-Control.py
│   │   └── IK-Leg-2DOF.py
│   │
│   └── analysis/                 # สคริปต์วิเคราะห์ข้อมูล
│       ├── Actuation-Weight-Scatter.py
│       ├── DOF-Weight-Scatter.py
│       ├── TW-Weight-Scatter.py
│       ├── TW-Bar-Chart.py
│       └── Sensor-Usage-Matrix.py
│
├── test/                         # สคริปต์ทดสอบ
│   ├── test_adaptive_blob.py
│   ├── test_camera_info.py
│   ├── check_video.py
│   ├── check_demo_video.py
│   ├── compare_recording_methods.py
│   └── analyze_results.py
│
├── .vscode/                      # การตั้งค่า VS Code
│   └── settings.json
│
├── __pycache__/                  # Python cache (ignored by git)
│
├── .gitignore                    # ไฟล์ที่ไม่ต้อง commit
└── README.md                     # ไฟล์นี้
```

## 📚 เอกสาร LaTeX

### Forward Kinematics (FK)
- **ไฟล์:** `docs/fk.tex`
- **เนื้อหา:** การวิเคราะห์จลนศาสตร์ขาหุ่นยนต์แบบ 5-Bar Linkage
- **คอมไพล์:** XeLaTeX (รองรับภาษาไทย)
- **Output:** `docs/output/fk.pdf`

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

#### IK-Five-Bar-Leg-Analytical.py
- **วัตถุประสงค์:** คำนวณ Inverse Kinematics แบบ Analytical
- **Features:**
  - รองรับ 4 Configurations (Elbow Up/Down)
  - ความแม่นยำสูง (machine precision)
  - ไม่ต้องใช้ค่าเดาเริ่มต้น

#### IK-Five-Bar-Leg-Numerical.py
- **วัตถุประสงค์:** คำนวณ IK แบบ Numerical (Newton-Raphson)
- **Features:**
  - ใช้ Jacobian ที่ derive แล้ว
  - รวดเร็วและแม่นยำ
  - ต้องมีค่าเดาเริ่มต้นที่ดี

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

### ตัวอย่างการใช้ IK Analytical
```python
from scripts.kinematics.IK_Five_Bar_Leg_Analytical import calculate_ik_analytical
import numpy as np

target = np.array([0.0, -200.0])  # เป้าหมาย (x, y) mm
angles = calculate_ik_analytical(target, elbow_C_down=True, elbow_D_down=True)

print(f"Theta A: {np.rad2deg(angles[0]):.2f}°")
print(f"Theta B: {np.rad2deg(angles[1]):.2f}°")
```

## 🎯 ผลการทดสอบ IK

สำหรับเป้าหมาย (0, -200) mm:

| Configuration | θA (°) | θB (°) | Error (mm) | Valid |
|--------------|--------|--------|------------|-------|
| Down-Down    | -119.53| -37.68 | 6.36e-14   | ✅ |
| Down-Up      | -139.91| -166.32| 1.68e-13   | ✅ |
| Up-Down      | -16.06 | -37.68 | 4.26e-14   | ✅ |
| Up-Up        | -      | -      | 339        | ❌ |

## 📝 หมายเหตุ

- ไฟล์ LaTeX ใช้ XeLaTeX เพื่อรองรับภาษาไทย
- Python scripts ใช้ Python 3.11+
- การคำนวณ IK มีความแม่นยำระดับ machine precision (~10⁻¹⁴)

## 👨‍💻 ผู้พัฒนา

นายธีรโชติ เมืองจำนงค์

## 📅 อัพเดทล่าสุด

17 พฤศจิกายน 2568
