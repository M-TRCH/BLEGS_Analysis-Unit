# BLEGS Documentation

This folder contains technical documentation for the BLEGS (Bio-inspired Legged Ground System) quadruped robot project.

## Folder Structure

```
docs/
├── Phase1_Kinematics/          # Kinematic Analysis Documents
│   ├── forward-kinematics-5bar.tex/.pdf
│   └── inverse-kinematics-analytical.tex/.pdf
│
├── Phase2_Dynamics/            # Dynamic Analysis Documents
│   ├── static-torque-analysis.tex/.pdf
│   └── Phase2.2_Dynamic_Torque_Analysis.tex
│
├── figures/                    # Shared figures and diagrams
│
└── output/                     # LaTeX compilation artifacts (aux, log, out files)
```

## Documents Overview

### Phase 1: Kinematics
- **Forward Kinematics (5-Bar)** - Mathematical derivation of end-effector position from joint angles
- **Inverse Kinematics (Analytical)** - Closed-form solution for joint angles from desired foot position

### Phase 2: Dynamics
- **Static Torque Analysis** - Gravity-based torque requirements at standing position
- **Dynamic Torque Analysis (Phase 2.2)** - Complete dynamic analysis including:
  - Inertial effects
  - Mass distribution (6.70 kg total robot mass)
  - Elliptical gait pattern (1 Hz)
  - Motor selection validation (5 N·m target)

## Compiling LaTeX Documents

To compile any LaTeX document:

```bash
cd docs/Phase1_Kinematics  # or Phase2_Dynamics
pdflatex forward-kinematics-5bar.tex
```

For documents with references or multiple passes:

```bash
pdflatex document.tex
pdflatex document.tex  # Second pass for references
```

## Notes

- All auxiliary files (.aux, .log, .out) are automatically moved to `output/` folder
- Figures are stored in the shared `figures/` folder and referenced relatively
- Source .tex files and compiled .pdf files are kept together for easy access
