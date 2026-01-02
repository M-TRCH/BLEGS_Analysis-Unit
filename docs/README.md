# BLEGS Analysis Unit - Documentation

เอกสารวิชาการสำหรับโปรเจ็กต์หุ่นยนต์สี่ขา BLEGS (Bipedal Legged Robot)

## 📁 โครงสร้างโฟลเดอร์

```
docs/
├── Phase1_Kinematics/          # เอกสาร Phase 1: จลนศาสตร์
│   ├── Phase1.1_Forward_Kinematics_5Bar.tex
│   └── Phase1.2_Inverse_Kinematics_Analytical.tex
│
├── Phase2_Dynamics/            # เอกสาร Phase 2: พลศาสตร์
│   ├── Phase2.1_Static_Torque_Analysis.tex
│   └── Phase2.2_Dynamic_Torque_Analysis.tex
│
├── Phase3_Simulation/          # เอกสาร Phase 3: การจำลอง
│   └── Phase3.1_Gait_Control_Simulation.tex
│
├── Phase4_Control/             # เอกสาร Phase 4: การควบคุม
│   ├── Phase4.1_Controller_Design.tex
│   └── Phase4.2_Hardware_Integration.tex
│
├── Phase5_Quadruped/           # เอกสาร Phase 5: หุ่นยนต์สี่ขา
│   ├── Phase5.1_Quadruped_Scaling.tex
│   └── Phase5.2_Gait_Tuning_Optimization.tex
│
├── output/                     # ไฟล์ PDF ที่คอมไพล์แล้ว
│   ├── Phase1.1_Forward_Kinematics_5Bar.pdf
│   ├── Phase1.2_Inverse_Kinematics_Analytical.pdf
│   ├── Phase2.1_Static_Torque_Analysis.pdf
│   ├── Phase2.2_Dynamic_Torque_Analysis.pdf
│   ├── Phase3.1_Gait_Control_Simulation.pdf
│   ├── Phase4.1_Controller_Design.pdf
│   ├── Phase4.2_Hardware_Integration.pdf
│   ├── Phase5.1_Quadruped_Scaling.pdf
│   ├── Phase5.2_Gait_Tuning_Optimization.pdf
│   └── *.aux, *.log, *.out, *.toc (auxiliary files)
│
├── figures/                    # รูปภาพสำหรับเอกสาร
│
└── README.md                   # ไฟล์นี้
```

## 📚 เอกสารทั้งหมด

### Phase 1: Kinematics Analysis

#### 1.1 Forward Kinematics of 5-Bar Linkage
- **ไฟล์:** `Phase1.1_Forward_Kinematics_5Bar.tex`
- **PDF:** [`output/Phase1.1_Forward_Kinematics_5Bar.pdf`](output/Phase1.1_Forward_Kinematics_5Bar.pdf)
- **เนื้อหา:** การวิเคราะห์จลนศาสตร์เชิงหน้าสำหรับกลไก 5-Bar Linkage พร้อม Jacobian Matrix

#### 1.2 Inverse Kinematics (Analytical)
- **ไฟล์:** `Phase1.2_Inverse_Kinematics_Analytical.tex`
- **PDF:** [`output/Phase1.2_Inverse_Kinematics_Analytical.pdf`](output/Phase1.2_Inverse_Kinematics_Analytical.pdf)
- **เนื้อหา:** การแก้ IK แบบ Analytical พร้อมวิเคราะห์ 4 Configurations

### Phase 2: Dynamics Analysis

#### 2.1 Static Torque Analysis
- **ไฟล์:** `Phase2.1_Static_Torque_Analysis.tex`
- **PDF:** [`output/Phase2.1_Static_Torque_Analysis.pdf`](output/Phase2.1_Static_Torque_Analysis.pdf)
- **เนื้อหา:** การวิเคราะห์แรงบิดแบบสถิตด้วย Virtual Work และ Jacobian Transpose

#### 2.2 Dynamic Torque Analysis
- **ไฟล์:** `Phase2.2_Dynamic_Torque_Analysis.tex`
- **PDF:** [`output/Phase2.2_Dynamic_Torque_Analysis.pdf`](output/Phase2.2_Dynamic_Torque_Analysis.pdf)
- **เนื้อหา:** การวิเคราะห์แรงบิดแบบไดนามิกด้วย Recursive Newton-Euler Algorithm

### Phase 3: Simulation

#### 3.1 Gait Control Simulation
- **ไฟล์:** `Phase3.1_Gait_Control_Simulation.tex`
- **PDF:** [`output/Phase3.1_Gait_Control_Simulation.pdf`](output/Phase3.1_Gait_Control_Simulation.pdf)
- **เนื้อหา:** การจำลองการควบคุมการเดินแบบ Trot Gait ด้วย PyBullet พร้อม Balance Control

### Phase 4: Control & Implementation

#### 4.1 Controller Design
- **ไฟล์:** `Phase4.1_Controller_Design.tex`
- **PDF:** [`output/Phase4.1_Controller_Design.pdf`](output/Phase4.1_Controller_Design.pdf)
- **เนื้อหา:** การออกแบบระบบควบคุมตำแหน่งด้วย Direct Position Control และ S-Curve Motion Profiling

#### 4.2 Hardware Integration
- **ไฟล์:** `Phase4.2_Hardware_Integration.tex`
- **PDF:** [`output/Phase4.2_Hardware_Integration.pdf`](output/Phase4.2_Hardware_Integration.pdf)
- **เนื้อหา:** การบูรณาการฮาร์ดแวร์ Binary Protocol v1.1 และการทดสอบขาเดียว (Single Leg Testing)

### Phase 5: Quadruped Scaling

#### 5.1 Quadruped Scaling
- **ไฟล์:** `Phase5.1_Quadruped_Scaling.tex`
- **PDF:** [`output/Phase5.1_Quadruped_Scaling.pdf`](output/Phase5.1_Quadruped_Scaling.pdf)
- **เนื้อหา:** การขยายระบบเป็นหุ่นยนต์สี่ขา Motor Indexing, Mirror Kinematics และ Multi-Leg Synchronization

#### 5.2 Gait Tuning and Optimization
- **ไฟล์:** `Phase5.2_Gait_Tuning_Optimization.tex`
- **PDF:** [`output/Phase5.2_Gait_Tuning_Optimization.pdf`](output/Phase5.2_Gait_Tuning_Optimization.pdf)
- **เนื้อหา:** การปรับจูนและพัฒนาหลายโหมดการเดิน (6 modes) พร้อม Asymmetric Trajectory Generation

## 🔧 การคอมไพล์เอกสาร

### ข้อกำหนด
- **LaTeX Engine:** XeLaTeX
- **ฟอนต์:** TH SarabunPSK (สำหรับภาษาไทย)
- **Packages:** fontspec, polyglossia, amsmath, amssymb, geometry, graphicx, hyperref, booktabs, tikz

### คำสั่งคอมไพล์

```bash
# คอมไพล์ Phase 1.1
cd Phase1_Kinematics
xelatex Phase1.1_Forward_Kinematics_5Bar.tex

# คอมไพล์ Phase 1.2
xelatex Phase1.2_Inverse_Kinematics_Analytical.tex

# คอมไพล์ Phase 2.1
cd ../Phase2_Dynamics
xelatex Phase2.1_Static_Torque_Analysis.tex

# คอมไพล์ Phase 2.2
xelatex Phase2.2_Dynamic_Torque_Analysis.tex

# คอมไพล์ Phase 3.1
cd ../Phase3_Simulation
xelatex Phase3.1_Gait_Control_Simulation.tex
```

### ย้ายไฟล์ไป output/ (PowerShell)

```powershell
# ย้าย PDF และ auxiliary files
Move-Item -Path "Phase*\*.pdf" -Destination "output\" -Force
Move-Item -Path "Phase*\*.aux,*.log,*.out,*.toc" -Destination "output\" -Force
```

## 📊 สถิติเอกสาร

- **จำนวนเอกสาร:** 9 ไฟล์
- **Phase 1:** 2 เอกสาร (Kinematics)
- **Phase 2:** 2 เอกสาร (Dynamics)
- **Phase 3:** 1 เอกสาร (Simulation)
- **Phase 4:** 2 เอกสาร (Control & Implementation)
- **Phase 5:** 2 เอกสาร (Quadruped Scaling)
- **ภาษา:** ไทย/อังกฤษ (Bilingual)
- **รูปแบบ:** Academic Paper Format

## 🔗 เอกสารที่เกี่ยวข้อง

- **GitHub Repository:** [M-TRCH/BLEGS_Analysis-Unit](https://github.com/M-TRCH/BLEGS_Analysis-Unit)
- **Main README:** [`../README.md`](../README.md)
- **ROADMAP:** [`../ROADMAP.md`](../ROADMAP.md)

## 👨‍💻 ผู้เขียน

นายธีรโชติ เมืองจำนงค์

## 📅 อัพเดทล่าสุด

2 มกราคม 2026 - เพิ่มเอกสาร Phase 4 และ Phase 5 (Control, Hardware Integration และ Quadruped Scaling)
