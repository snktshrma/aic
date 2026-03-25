# aic_controller — Changelog

## 2026-03-26

### Documentation
- Created `AIC_CONTROLLER_THEORY.md` — comprehensive mathematical documentation covering:
  - SE(3)/SO(3) Lie group foundations (exp/log maps, quaternion operations)
  - Cartesian impedance control law with PID integral extension
  - Jacobian transpose torque mapping with derivation from virtual work principle
  - Nullspace control with SVD-regularized pseudo-inverse (Chiaverini 1997)
  - Joint-limit avoidance via linear potential fields
  - Joint-space impedance control
  - KDL-based gravity compensation
  - Reference generation (SLERP, linear interpolation, SE(3) velocity integration)
  - Cartesian & joint limit clamping with bisquare soft-margin deceleration
  - Impedance parameter exponential smoothing and rate-limited feedforward wrench
  - Force feedback with tare offset
  - Tracking error watchdog
  - Complete algorithm flow diagram with final torque equations
  - File-to-concept mapping table
- Created `AIC_CONTROLLER_THEORY.tex` — proper LaTeX version of the same document,
  compilable to PDF with `pdflatex`. Includes TikZ diagrams (activation zone,
  bisquare curve, algorithm flowchart), colored theorem-style boxes for key equations,
  example boxes, reasoning boxes, and full cross-referencing.
