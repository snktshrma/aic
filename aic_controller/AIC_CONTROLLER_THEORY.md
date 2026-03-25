# AIC Controller — Mathematical Foundations & Algorithm Reference

> A comprehensive document covering the control theory, mathematical derivations,
> and algorithm flow of the `aic_controller` package.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Mathematical Preliminaries](#2-mathematical-preliminaries)
   - 2.1 [Rigid Body Poses — SE(3) and SO(3)](#21-rigid-body-poses--se3-and-so3)
   - 2.2 [Exponential & Logarithmic Maps](#22-exponential--logarithmic-maps)
   - 2.3 [Robot Jacobian](#23-robot-jacobian)
3. [Cartesian Impedance Control](#3-cartesian-impedance-control)
   - 3.1 [The Spring-Damper Analogy](#31-the-spring-damper-analogy)
   - 3.2 [Control Wrench Law](#32-control-wrench-law)
   - 3.3 [PID-Like Extension — Integral Term](#33-pid-like-extension--integral-term)
   - 3.4 [Mapping Wrench to Joint Torques](#34-mapping-wrench-to-joint-torques)
   - 3.5 [Wrench & Torque Saturation](#35-wrench--torque-saturation)
4. [Nullspace Control](#4-nullspace-control)
   - 4.1 [Redundancy and the Nullspace](#41-redundancy-and-the-nullspace)
   - 4.2 [Nullspace Projector](#42-nullspace-projector)
   - 4.3 [Singularity-Robust Pseudo-Inverse (SVD Regularization)](#43-singularity-robust-pseudo-inverse-svd-regularization)
   - 4.4 [Nullspace Impedance Torque](#44-nullspace-impedance-torque)
5. [Joint-Limit Avoidance](#5-joint-limit-avoidance)
   - 5.1 [Activation Zone](#51-activation-zone)
   - 5.2 [Potential Field Torque](#52-potential-field-torque)
6. [Joint-Space Impedance Control](#6-joint-space-impedance-control)
7. [Gravity Compensation](#7-gravity-compensation)
8. [Reference Generation & Interpolation](#8-reference-generation--interpolation)
   - 8.1 [Position Mode — SLERP and Linear Interpolation](#81-position-mode--slerp-and-linear-interpolation)
   - 8.2 [Velocity Mode — Lie-Group Integration](#82-velocity-mode--lie-group-integration)
9. [Cartesian & Joint Limit Clamping](#9-cartesian--joint-limit-clamping)
   - 9.1 [Translation Clamping](#91-translation-clamping)
   - 9.2 [Rotation Clamping via Quaternion Log-Map](#92-rotation-clamping-via-quaternion-log-map)
   - 9.3 [Velocity Scaling](#93-velocity-scaling)
   - 9.4 [Bisquare Soft-Margin Deceleration](#94-bisquare-soft-margin-deceleration)
10. [Impedance Parameter Smoothing](#10-impedance-parameter-smoothing)
    - 10.1 [Exponential Smoothing of Stiffness & Damping](#101-exponential-smoothing-of-stiffness--damping)
    - 10.2 [Feedforward Wrench Interpolation (Rate-Limited)](#102-feedforward-wrench-interpolation-rate-limited)
    - 10.3 [Force Feedback with Tare Offset](#103-force-feedback-with-tare-offset)
11. [Tracking Error Watchdog](#11-tracking-error-watchdog)
12. [Complete Algorithm Flow (`update()`)](#12-complete-algorithm-flow-update)
13. [File-to-Concept Map](#13-file-to-concept-map)

---

## 1. Architecture Overview

The AIC Controller is a ROS 2 `controller_interface::ControllerInterface` plugin.
It runs inside `ros2_control` at a fixed frequency and supports two **control modes**:

| Mode | Output Interface | Status |
|------|-----------------|--------|
| **Impedance** | Joint effort (torque) | Implemented |
| **Admittance** | Joint position | Not yet implemented |

Within impedance mode, two **target modes** select the coordinate space of
incoming commands:

| Target Mode | Message Type | Impedance Action |
|-------------|-------------|-----------------|
| **Cartesian** | `MotionUpdate` | `CartesianImpedanceAction` |
| **Joint** | `JointMotionUpdate` | `JointImpedanceAction` |

Each control cycle executes the pipeline:

```
Read Hardware → Ingest Commands → Generate Reference → Compute Torques → Write Hardware
```

---

## 2. Mathematical Preliminaries

### 2.1 Rigid Body Poses — SE(3) and SO(3)

A rigid body pose in 3D is an element of the **Special Euclidean group** \( SE(3) \):

\[
T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3), \quad R \in SO(3), \; p \in \mathbb{R}^3
\]

where \( R \) is a rotation matrix (element of \( SO(3) \)) and \( p \) is a
translation vector.

In the code, poses are stored as `Eigen::Isometry3d`, which internally holds a
4×4 homogeneous matrix with `pose.linear()` = \( R \) and
`pose.translation()` = \( p \).

### 2.2 Exponential & Logarithmic Maps

The **Lie algebra** \( \mathfrak{so}(3) \) of \( SO(3) \) consists of 3×3
skew-symmetric matrices. There is a bijection between \( \mathbb{R}^3 \) and
\( \mathfrak{so}(3) \) via the *hat* operator:

\[
\boldsymbol{\omega} = \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix}
\;\mapsto\;
[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}
\]

**Exponential map** (tangent vector → rotation):

\[
\text{Exp}: \mathbb{R}^3 \to SO(3), \quad R = \exp([\boldsymbol{\omega}]_\times)
\]

Computed via the Rodrigues formula. For quaternions (Sophus `SO3::exp`):

\[
q = \exp(\boldsymbol{\delta}) \quad \text{where} \quad
\theta = \|\boldsymbol{\delta}\|, \;
q = \left(\cos\frac{\theta}{2},\; \frac{\boldsymbol{\delta}}{\theta}\sin\frac{\theta}{2}\right)
\]

**Logarithmic map** (rotation → tangent vector):

\[
\text{Log}: SO(3) \to \mathbb{R}^3, \quad \boldsymbol{\omega} = \text{Log}(R)
\]

This is the inverse of Exp. Given a unit quaternion \( q \), Sophus `SO3::log`
extracts the axis-angle vector \( \boldsymbol{\omega} \) such that
\( \|\boldsymbol{\omega}\| = \theta \) (the rotation angle) and
\( \boldsymbol{\omega}/\theta \) is the rotation axis.

**For SE(3)**, the twist (6D velocity) is \( \boldsymbol{\xi} = [v^T, \omega^T]^T \in \mathbb{R}^6 \).
Integration on SE(3) uses:

\[
T_{k+1} = T_k \cdot \text{Exp}(\boldsymbol{\xi} \cdot \Delta t)
\]

This is implemented in `utils::integrate_pose` using Sophus `SE3::exp`.

**Why this matters:** Classical Euler angles suffer from gimbal lock. Working
directly on the Lie group ensures singularity-free pose integration and
consistent orientation error computation.

### 2.3 Robot Jacobian

For an \( n \)-DOF robot, the **geometric Jacobian** \( J(q) \in \mathbb{R}^{6 \times n} \)
maps joint velocities to the end-effector twist:

\[
\boldsymbol{\xi} = J(q)\,\dot{q}
\]

where \( \boldsymbol{\xi} = [v^T, \omega^T]^T \) is the 6D spatial velocity
(linear + angular) of the tool frame.

The Jacobian is computed externally by a `kinematics_interface` plugin
(typically KDL or pinocchio-based) via `calculate_jacobian()`.

---

## 3. Cartesian Impedance Control

### 3.1 The Spring-Damper Analogy

Impedance control makes the robot behave like a **mechanical spring-damper system**
in Cartesian space. Instead of tracking a trajectory rigidly, the controller
generates forces/torques proportional to position and velocity errors, allowing
compliant interaction with the environment.

A 1D mass-spring-damper satisfies:

\[
m\ddot{x} + d\dot{x} + kx = f_{\text{ext}}
\]

In impedance control we invert this: given a desired impedance
\( (K, D) \), we *generate* the wrench:

\[
F = K \cdot e_x + D \cdot e_v
\]

where \( e_x \) is position error and \( e_v \) is velocity error.

### 3.2 Control Wrench Law

The full 6D control wrench in the **base frame** is:

\[
\boxed{
F_{\text{ctrl}} = K \, e_x + D \, e_v + F_{\text{ff}} + K_I \odot \int e_x \, dt + F_{\text{offset}}
}
\]

where:

| Symbol | Dimensions | Description |
|--------|-----------|-------------|
| \( K \) | \( 6 \times 6 \) | Cartesian stiffness matrix (diagonal in practice) |
| \( D \) | \( 6 \times 6 \) | Cartesian damping matrix (diagonal in practice) |
| \( e_x \) | \( 6 \times 1 \) | Pose error \( = x_{\text{des}} - x_{\text{current}} \) |
| \( e_v \) | \( 6 \times 1 \) | Velocity error \( = \dot{x}_{\text{des}} - \dot{x}_{\text{current}} \) |
| \( F_{\text{ff}} \) | \( 6 \times 1 \) | Feedforward wrench (from force feedback loop, see §10) |
| \( K_I \) | \( 6 \times 1 \) | Integrator gain (element-wise) |
| \( F_{\text{offset}} \) | \( 6 \times 1 \) | Constant wrench offset (e.g. payload compensation) |

The pose error \( e_x \) is computed by the kinematics plugin's
`calculate_frame_difference()`, which yields a 6D vector
\( [e_{\text{trans}}^T, e_{\text{rot}}^T]^T \) where the translational part is the
position difference and the rotational part is the orientation error in
axis-angle form.

**Example:** Suppose a 7-DOF arm has its TCP at position \( p = [0.5, 0, 0.3] \) m
and the desired position is \( p_d = [0.5, 0.05, 0.3] \) m. With
\( K = \text{diag}(500, 500, 500, 20, 20, 20) \) N/m (N·m/rad):

\[
e_x = [0, 0.05, 0, 0, 0, 0]^T \;\Rightarrow\; F = K \, e_x = [0, 25, 0, 0, 0, 0]^T \text{ N}
\]

A 25 N restoring force along \( y \).

### 3.3 PID-Like Extension — Integral Term

To eliminate steady-state error (e.g. from unmodeled friction or gravity), an
integral term accumulates pose error over time:

\[
e_I(k+1) = \text{clamp}\!\Big(e_I(k) + e_x(k),\; -B,\; B\Big)
\]

\[
F_I = K_I \odot e_I
\]

where \( B \in \mathbb{R}^6 \) is the integrator bound (anti-windup),
\( \odot \) denotes element-wise multiplication, and the clamp prevents
unbounded growth.

**Reasoning:** The integrator bound is critical in robotics — without it, large
transient errors (e.g. during collisions) would cause the integrator to
*wind up*, producing dangerous persistent forces even after the error is resolved.

### 3.4 Mapping Wrench to Joint Torques

The wrench \( F_{\text{ctrl}} \) in Cartesian space is mapped to joint torques via the
**Jacobian transpose**:

\[
\boxed{
\tau = J^T \, F_{\text{ctrl}}
}
\]

**Why \( J^T \)?** This follows from the principle of virtual work. If a virtual
displacement \( \delta q \) in joint space produces a Cartesian displacement
\( \delta x = J \, \delta q \), then the work done by force \( F \) is:

\[
\delta W = F^T \delta x = F^T J \, \delta q = (J^T F)^T \delta q
\]

So the equivalent joint torque is \( \tau = J^T F \). This does *not* require
inverting the Jacobian and works even at singularities (though the Cartesian
stiffness degenerates in singular directions).

### 3.5 Wrench & Torque Saturation

Before applying the Jacobian transpose, the wrench is clamped element-wise:

\[
F_{\text{ctrl},i} = \text{clamp}(F_{\text{ctrl},i},\; -F_{\max,i},\; F_{\max,i})
\]

After computing joint torques (including nullspace and joint-limit contributions),
each joint torque is clamped to the URDF-specified effort limit:

\[
\tau_i = \text{clamp}(\tau_i,\; -\tau_{\max,i},\; \tau_{\max,i})
\]

---

## 4. Nullspace Control

### 4.1 Redundancy and the Nullspace

A robot with \( n > 6 \) joints (e.g. 7-DOF) has **kinematic redundancy**: multiple
joint configurations can produce the same end-effector pose. The extra degree(s) of
freedom span the **nullspace** of the Jacobian — the set of joint velocities that
produce zero end-effector motion:

\[
\mathcal{N}(J) = \{ \dot{q} \in \mathbb{R}^n \mid J\,\dot{q} = 0 \}
\]

Nullspace control exploits this by adding torques that move the joints without
affecting the TCP, useful for:

- Moving toward a preferred "elbow" configuration
- Avoiding joint limits
- Optimizing manipulability

### 4.2 Nullspace Projector

The nullspace projection matrix is:

\[
N = I_n - J \, J^\dagger
\]

where \( J^\dagger \) is the (right) pseudo-inverse of \( J \). Any torque
\( \tau_0 \in \mathbb{R}^n \) projected through \( N \) will not affect the
end-effector:

\[
\tau_{\text{ns}} = N \, \tau_0
\]

**Proof sketch:** \( J \, J^\dagger \, J = J \), so
\( J \, N = J(I - J J^\dagger) = J - J = 0 \). Therefore the Cartesian
acceleration produced by \( \tau_{\text{ns}} \) is zero (in the linearized
dynamics).

### 4.3 Singularity-Robust Pseudo-Inverse (SVD Regularization)

The naive pseudo-inverse \( J^\dagger = J^T(J J^T)^{-1} \) blows up near
singularities. This implementation uses a **damped pseudo-inverse** following
Chiaverini (1997) with *per-dimension adaptive damping*.

Given the SVD of \( M = J J^T \):

\[
M = J J^T = U \, \Sigma \, V^T
\]

where \( \Sigma = \text{diag}(\sigma_1, \sigma_2, \ldots, \sigma_m) \) with
\( \sigma_1 \ge \sigma_2 \ge \ldots \ge \sigma_m \ge 0 \).

The regularized inverse singular values are:

\[
\Sigma^\dagger_{ii} = \frac{\sigma_i}{\sigma_i^2 + \lambda_i^2}
\]

The damping factor \( \lambda_i \) is **adapted per dimension** based on the
condition number:

\[
\text{cn}_i = \frac{\sigma_1}{\sigma_i + 10^{-10}}
\]

\[
\lambda_i^2 = \begin{cases}
\left(1 - \left(\dfrac{c_{\text{th}}}{\text{cn}_i}\right)^2\right)^2 \cdot \dfrac{1}{\epsilon}
& \text{if } \text{cn}_i > c_{\text{th}} \\[6pt]
0 & \text{otherwise}
\end{cases}
\]

where \( c_{\text{th}} = 1500 \) is the condition number threshold and
\( \epsilon = 10^{-6} \) is a small regularizer.

The **bisquare function** \( f(r) = (1 - r^2)^2 \) where
\( r = c_{\text{th}}/\text{cn}_i \) is used because:

- \( f(0) = 1 \): full damping when the condition number is extremely high
- \( f(1) = 0 \): zero damping at the threshold boundary
- \( f'(0) = 0 \) and \( f'(1) = 0 \): smooth onset/offset — no discontinuous
  jumps in the control signal

The final pseudo-inverse is:

\[
J^\dagger = J^T \, V \, \Sigma^\dagger \, U^T
\]

**Example:** Consider a 7-DOF arm near a shoulder singularity where
\( \sigma_6 \approx 0.001 \) and \( \sigma_1 = 5.0 \):

\[
\text{cn}_6 = \frac{5.0}{0.001} = 5000 \gg 1500 \quad \Rightarrow \quad
\lambda_6^2 = \left(1 - (1500/5000)^2\right)^2 \cdot 10^6 \approx 7.6 \times 10^5
\]

This large \( \lambda_6^2 \) suppresses the near-zero singular value, preventing
torque spikes.

### 4.4 Nullspace Impedance Torque

The raw nullspace torque is a joint-space PD controller toward a preferred configuration:

\[
\tau_0 = K_{\text{ns}} \odot (q_{\text{ns}}^* - q) - D_{\text{ns}} \odot \dot{q}
\]

where:
- \( q_{\text{ns}}^* \) is the desired nullspace joint configuration
- \( K_{\text{ns}}, D_{\text{ns}} \in \mathbb{R}^n \) are per-joint stiffness/damping
- \( \odot \) is element-wise product

This is then projected:

\[
\tau_{\text{ns}} = N \, \tau_0 = (I - J J^\dagger) \, \tau_0
\]

---

## 5. Joint-Limit Avoidance

### 5.1 Activation Zone

For each joint \( k \), an **activation zone** is defined as a percentage of the
joint range:

\[
q_{\text{range}} = q_{\max} - q_{\min}
\]

\[
q_{\text{upper\_thresh}} = q_{\min} + \frac{\alpha \cdot q_{\text{range}}}{2}
\]

\[
q_{\text{lower\_thresh}} = q_{\max} - \frac{\alpha \cdot q_{\text{range}}}{2}
\]

where \( \alpha \in (0, 1] \) is the `activation_percentage`.

When \( \alpha = 1 \), the activation zone spans the entire joint range (the
potential field is always active). When \( \alpha = 0 \), avoidance is disabled.

```
q_min          lower_thresh          upper_thresh          q_max
  |<--- active -->|<--- safe zone --->|<--- active -->|
```

### 5.2 Potential Field Torque

Within the activation zone, a **linear repulsive potential** pushes the joint away
from its limit:

\[
\tau_{\text{avoid},k} = \begin{cases}
g_u \cdot (q_{\text{upper\_thresh}} - q_k) & \text{if } q_k > q_{\text{upper\_thresh}} \\[4pt]
g_l \cdot (q_{\text{lower\_thresh}} - q_k) & \text{if } q_k < q_{\text{lower\_thresh}} \\[4pt]
0 & \text{otherwise}
\end{cases}
\]

The gains are set so the torque reaches the joint's max effort exactly at the limit:

\[
g_u = \frac{\tau_{\max}}{|q_{\max} - q_{\text{upper\_thresh}}|}, \quad
g_l = \frac{\tau_{\max}}{|q_{\min} - q_{\text{lower\_thresh}}|}
\]

If the joint exceeds its limit, the torque is **saturated** at \( \tau_{\max} \).

---

## 6. Joint-Space Impedance Control

For joint-target mode, a simpler diagonal impedance law is used:

\[
\boxed{
\tau = K \odot (q^* - q) + D \odot (\dot{q}^* - \dot{q}) + \tau_{\text{ff}}
}
\]

where:
- \( K, D \in \mathbb{R}^n \) are per-joint stiffness/damping vectors
- \( q^*, \dot{q}^* \) are reference position/velocity
- \( \tau_{\text{ff}} \) is feedforward torque
- All operations are element-wise

Each resulting torque is clamped to the URDF effort limit:

\[
\tau_i = \text{clamp}(\tau_i,\; -\tau_{\max,i},\; \tau_{\max,i})
\]

This mode is simpler than Cartesian impedance because it operates directly in
joint space, requiring no Jacobian, no nullspace projection, and no FK-based
error computation.

---

## 7. Gravity Compensation

The gravity torque vector \( \tau_g(q) \) compensates for gravitational forces on
the robot links:

\[
\tau_g(q) = \sum_{i=1}^{n} J_{v,i}^T(q) \, m_i \, \mathbf{g}
\]

where \( J_{v,i} \) is the linear Jacobian of the center of mass of link \( i \),
\( m_i \) is its mass, and \( \mathbf{g} = [0, 0, -9.80665]^T \) m/s².

This is computed by KDL's `ChainDynParam::JntToGravity` using the URDF-derived
kinematic chain from `base_link` to `gripper/tcp`. The result is **added** to the
impedance torques:

\[
\tau_{\text{total}} = \tau_{\text{impedance}} + \tau_g(q)
\]

**Reasoning:** In simulation, the physics engine already applies gravity. But on real
hardware, the motor controllers typically receive raw torque commands, so the
controller must explicitly cancel gravity to keep the arm from collapsing.

---

## 8. Reference Generation & Interpolation

The controller receives targets asynchronously from user commands (lower frequency)
but must output smooth references at the control rate (higher frequency).

### 8.1 Position Mode — SLERP and Linear Interpolation

When `remaining_time_to_target > 0`:

**Translation** (linear interpolation per cycle):

\[
p_{\text{ref}}(k+1) = p_{\text{ref}}(k) + \frac{p_{\text{target}} - p_{\text{ref}}(k)}{f_c \cdot t_{\text{rem}}}
\]

where \( f_c \) is the control frequency (Hz) and \( t_{\text{rem}} \) is the
remaining time to target (seconds).

**Rotation** (SLERP — Spherical Linear intERPolation):

\[
q_{\text{ref}}(k+1) = \text{SLERP}\!\left(q_{\text{ref}}(k),\; q_{\text{target}},\; \frac{1}{f_c \cdot t_{\text{rem}}}\right)
\]

SLERP interpolates along the great arc on the unit quaternion sphere, ensuring
constant angular velocity:

\[
\text{SLERP}(q_0, q_1, t) = q_0 \, (q_0^{-1} q_1)^t = \frac{\sin((1-t)\Omega)}{\sin\Omega} q_0 + \frac{\sin(t\Omega)}{\sin\Omega} q_1
\]

where \( \Omega = \arccos(q_0 \cdot q_1) \).

When `remaining_time_to_target ≤ 0` (the common case, since this timer is not
currently set from incoming messages): the reference **snaps** directly to the
target pose and the reference velocity is set to zero.

### 8.2 Velocity Mode — Lie-Group Integration

In velocity mode, the target provides a desired twist \( \boldsymbol{\xi}_d \)
in the TCP frame. The reference pose is integrated on \( SE(3) \):

\[
T_{\text{ref}}(k+1) = T_{\text{ref}}(k) \cdot \text{Exp}(\boldsymbol{\xi}_d \, \Delta t)
\]

where \( \Delta t = 1/f_c \) and \( \text{Exp} \) is the SE(3) exponential map
(Sophus `SE3::exp`).

**Why body-frame integration?** The twist \( \boldsymbol{\xi}_d \) is expressed in
the TCP frame (body frame), so we **right-multiply** by the exponential. This
means "move forward along my current heading" regardless of the global
orientation — exactly the intuition for teleop and tool-frame velocity commands.

After integration, the resulting pose is clamped to Cartesian limits (§9).

For joint velocity mode:

\[
q_{\text{ref}}(k+1) = q_{\text{ref}}(k) + \dot{q}_d \cdot \Delta t
\]

---

## 9. Cartesian & Joint Limit Clamping

### 9.1 Translation Clamping

Position targets are clamped element-wise to a bounding box:

\[
p_i = \text{clamp}(p_i,\; p_{\min,i},\; p_{\max,i}) \quad \text{for } i \in \{x, y, z\}
\]

### 9.2 Rotation Clamping via Quaternion Log-Map

Rotation limits are expressed in the tangent space relative to a **reference
quaternion** \( q_{\text{ref}} \):

1. Compute the relative quaternion: \( q_{\text{rel}} = q_{\text{target}} \cdot q_{\text{ref}}^{-1} \)
2. Compute the log-map: \( \boldsymbol{\phi} = \text{Log}(q_{\text{rel}}) \in \mathbb{R}^3 \)
3. Clamp: \( \boldsymbol{\phi}_i = \text{clamp}(\boldsymbol{\phi}_i,\; \phi_{\min,i},\; \phi_{\max,i}) \)
4. Reconstruct: \( q_{\text{clamped}} = \text{Exp}(\boldsymbol{\phi}_{\text{clamped}}) \cdot q_{\text{ref}} \)

**Why use log-map instead of Euler angles?** The log-map provides a
singularity-free, minimal parameterization of the relative rotation. Euler angles
suffer from gimbal lock and ambiguous representations near ±90° pitch.

### 9.3 Velocity Scaling

Velocity targets are scaled (not clamped) to preserve **direction**:

\[
\dot{p}_{\text{scaled}} = s \cdot \dot{p}, \quad s = \min_i\left(\frac{v_{\max,i}}{|\dot{p}_i|}\right) \le 1
\]

A single scaling factor is computed across all three translational axes, so the
velocity direction is preserved. Similarly for rotational velocity, the norm is
checked:

\[
\text{if } \|\boldsymbol{\omega}\| > \omega_{\max}: \quad \boldsymbol{\omega}_{\text{scaled}} = \omega_{\max} \cdot \frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}
\]

### 9.4 Bisquare Soft-Margin Deceleration

When a soft margin \( m > 0 \) is configured, velocity is smoothly scaled to zero
as the position approaches a limit. The normalized penetration into the soft margin
is:

\[
d = \text{clamp}\!\left(\frac{\text{distance from soft margin boundary}}{m},\; 0,\; 1\right)
\]

The scale factor uses the **bisquare** (biweight) kernel:

\[
s(d) = (1 - d^2)^2
\]

Properties:
- \( s(0) = 1 \): full speed at the margin boundary
- \( s(1) = 0 \): zero speed at the hard limit
- \( s'(0) = 0 \): smooth onset
- \( s'(1) = 0 \): smooth offset

This ensures no discontinuous velocity jumps at the margin boundary.

---

## 10. Impedance Parameter Smoothing

### 10.1 Exponential Smoothing of Stiffness & Damping

Stiffness and damping matrices are **not** applied instantaneously from incoming
messages. Instead, an exponential moving average smooths transitions:

\[
S_{n+1} = (1 - c) \, S_n + c \, S_{\text{target}}
\]

where \( c \in [0, 1] \) is the smoothing constant. Separate constants exist for
stiffness and damping.

**Reasoning:** Instantaneous stiffness changes can cause torque discontinuities
that excite structural resonances or jerk the arm. Smoothing acts as a first-order
low-pass filter with time constant \( \tau \approx \Delta t / c \).

**Example:** With \( c = 0.05 \) at 1 kHz, the effective time constant is
\( \tau = 1\text{ms} / 0.05 = 20\text{ms} \). A step change in stiffness from
100 to 500 N/m would reach 95% of the target in \( \approx 60 \) ms (≈ 3τ).

### 10.2 Feedforward Wrench Interpolation (Rate-Limited)

The feedforward wrench at the TCP is ramped toward its target using a
**rate limiter**:

\[
F_{\text{ff},i}(k+1) = F_{\text{ff},i}(k) + \text{clamp}\!\left(
F_{\text{target},i} - F_{\text{ff},i}(k),\;
-\dot{F}_{\max,i} \cdot \Delta t,\;
\dot{F}_{\max,i} \cdot \Delta t
\right)
\]

The target wrench is also clamped to configurable min/max bounds before ramping.

### 10.3 Force Feedback with Tare Offset

For force-controlled behavior, a feedback loop mixes the feedforward wrench with
the sensed wrench:

\[
F_{\text{total}} = F_{\text{ff}} + G_{\text{fb}} \odot (F_{\text{ff}} - F_{\text{sensed}}^{\text{tared}})
\]

where:
- \( G_{\text{fb}} \in \mathbb{R}^6 \) are the per-axis feedback gains
- \( F_{\text{sensed}}^{\text{tared}} = F_{\text{sensed}} - F_{\text{tare}} \) is
  the tared FT reading

The **tare offset** is captured by a service call and stored in the base frame.
Each cycle, it is transformed to the TCP frame using the current rotation:

\[
F_{\text{tare}}^{\text{tip}} = R_{\text{tcp}}^{-1} \, F_{\text{tare}}^{\text{base}}
\]

The total wrench (originally in TCP frame) is then transformed to the base frame
for the impedance law:

\[
F_{\text{ff}}^{\text{base}} = R_{\text{tcp}} \, F_{\text{total}}^{\text{tcp}}
\]

(Applied separately to force and torque 3-vectors.)

---

## 11. Tracking Error Watchdog

A safety mechanism detects when the controller is unable to make progress toward
its target:

1. Each cycle, compute the change in pose error:
   \( \Delta e = |e_x(k) - e_x(k-1)| \)

2. If the absolute error exceeds a minimum threshold **AND** the change
   \( \Delta e \) is below a minimum change threshold for longer than a
   configurable timeout → the target is **reset** to the current pose.

This prevents the controller from indefinitely building up integral error or
feedforward wrench when blocked by obstacles, joint limits, or singularities.

---

## 12. Complete Algorithm Flow (`update()`)

```
┌──────────────────────────────────────────────┐
│                  update()                     │
├──────────────────────────────────────────────┤
│                                              │
│  1. READ HARDWARE                            │
│     ├─ Joint positions q, velocities q̇      │
│     ├─ FK → TCP pose T, velocity ξ           │
│     ├─ FT sensor → sensed wrench             │
│     └─ NaN fallback → last commanded state   │
│                                              │
│  2. INGEST COMMANDS                          │
│     ├─ Cartesian: MotionUpdate message       │
│     │   ├─ Position mode: transform if TCP   │
│     │   │   frame, store as target_state_    │
│     │   └─ Velocity mode: transform twist    │
│     │       if base frame, set current pose  │
│     └─ Joint: JointMotionUpdate message      │
│         └─ Store as joint_target_state_      │
│                                              │
│  3. GENERATE REFERENCE                       │
│     ├─ Clamp target to limits (§9)           │
│     ├─ Linear interpolation / SLERP (§8.1)  │
│     │   or velocity integration (§8.2)       │
│     └─ Decrement remaining_time_to_target    │
│                                              │
│  4. SMOOTH IMPEDANCE PARAMETERS (§10)        │
│     ├─ Exponential smoothing of K, D         │
│     ├─ Rate-limited feedforward wrench       │
│     └─ Force feedback + tare                 │
│                                              │
│  5. COMPUTE CONTROL TORQUES                  │
│     │                                        │
│     ├─ [Cartesian target mode]               │
│     │   ├─ Pose error via FK (§3.2)          │
│     │   ├─ Velocity error (§3.2)             │
│     │   ├─ Jacobian computation              │
│     │   ├─ Tracking watchdog check (§11)     │
│     │   └─ CartesianImpedanceAction:         │
│     │       ├─ Wrench = K*e_x + D*e_v + ... │
│     │       ├─ τ = J^T * F_ctrl (§3.4)      │
│     │       ├─ + Nullspace torque (§4)       │
│     │       ├─ + Joint-limit avoidance (§5)  │
│     │       └─ Clamp to effort limits        │
│     │                                        │
│     ├─ [Joint target mode]                   │
│     │   └─ JointImpedanceAction:             │
│     │       ├─ τ = K⊙e_q + D⊙e_q̇ + τ_ff   │
│     │       └─ Clamp to effort limits        │
│     │                                        │
│     └─ + Gravity compensation τ_g (§7)       │
│                                              │
│  6. WRITE HARDWARE                           │
│     └─ Set effort command interfaces         │
│                                              │
│  7. PUBLISH STATE                            │
│     └─ TCP pose, velocity, error,            │
│        joint reference, tare offset          │
│                                              │
└──────────────────────────────────────────────┘
```

### Putting It All Together — The Final Torque Equation

**Cartesian impedance mode:**

\[
\boxed{
\tau = J^T \Big[\underbrace{K e_x + D e_v + F_{\text{ff}} + K_I \odot e_I + F_{\text{offset}}}_{\text{Cartesian impedance wrench}}\Big]
\;+\; \underbrace{(I - J J^\dagger)(K_{\text{ns}} \odot (q^*_{\text{ns}} - q) - D_{\text{ns}} \odot \dot{q})}_{\text{Nullspace torque}}
\;+\; \underbrace{\tau_{\text{avoid}}}_{\text{Joint-limit}}
\;+\; \underbrace{\tau_g(q)}_{\text{Gravity}}
}
\]

**Joint impedance mode:**

\[
\boxed{
\tau = K \odot (q^* - q) + D \odot (\dot{q}^* - \dot{q}) + \tau_{\text{ff}} + \tau_g(q)
}
\]

---

## 13. File-to-Concept Map

| File | Sections Covered |
|------|-----------------|
| `src/actions/cartesian_impedance_action.cpp` | §3 (Wrench law, J^T mapping), §4 (Nullspace), §5 (Joint limits) |
| `include/.../cartesian_impedance_action.hpp` | §4.3 (Chiaverini pseudo-inverse documentation) |
| `src/actions/joint_impedance_action.cpp` | §6 (Joint impedance law) |
| `src/actions/gravity_compensation_action.cpp` | §7 (KDL gravity) |
| `src/aic_controller.cpp` | §8 (Reference generation), §9 (Clamping), §10 (Smoothing), §11 (Watchdog), §12 (Full flow) |
| `src/utils.cpp` | §2.2 (Exp/Log maps), §8.2 (SE3 integration) |
| `include/.../cartesian_limits.hpp` | §9.2 (Rotation limits structure) |
| `include/.../cartesian_state.hpp` | §2.1 (Pose representation) |

---

### References

1. Chiaverini, S. (1997). *Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators.* IEEE Transactions on Robotics and Automation, 13(3), 398–410.
2. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control.* Springer.
3. Lynch, K. M. & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control.* Cambridge University Press.
