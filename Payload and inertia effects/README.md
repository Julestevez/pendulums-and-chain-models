# 1- 3D Driven Damped Pendulum with Wind: Mathematical Model

This document describes the mathematical model used in the 3D pendulum simulation with a moving pivot point and wind effects.

## System Parameters

The simulation uses the following parameters:

- g = 9.81 m/s² - Gravitational acceleration
- m - Mass of the pendulum bob
- d - Damping coefficient
- L - Length of the pendulum arm
- A - Amplitude of the driving force
- ω - Frequency of the driving force
- Pivot motion amplitudes: [Ax, Ay, Az]
- Pivot motion frequencies: [ωx, ωy, ωz]
- Wind strength and frequency

## Coordinate System and State Variables

The pendulum's position is described using three angles (Euler angles):
- θ (theta): Angle from the vertical axis
- φ (phi): Rotation angle in the horizontal plane
- ψ (psi): Rotation around the pendulum arm

The full state of the system is represented by:
- θ, φ, ψ: The three angles
- θ̇, φ̇, ψ̇: Their respective angular velocities

## Pivot Motion

The pivot position at time t is given by:

```
x(t) = Ax·sin(ωx·t)
y(t) = Ay·cos(ωy·t)
z(t) = Az·sin(ωz·t)
```

The velocity of the pivot is:

```
vx(t) = Ax·ωx·cos(ωx·t)
vy(t) = -Ay·ωy·sin(ωy·t)
vz(t) = Az·ωz·cos(ωz·t)
```

## Wind Force

The wind force in the y-direction is modeled as:

```
Fwind(t) = wind_strength                     (if wind_frequency = 0)
Fwind(t) = wind_strength·sin(wind_frequency·t)  (otherwise)
```

## Pendulum Bob Position

The position of the pendulum bob relative to the pivot is:

```
x = L·sin(θ)·cos(φ)
y = L·sin(θ)·sin(φ)
z = -L·cos(θ)
```

## Equations of Motion

The angular accelerations are given by:

1. For θ (polar angle):
```
θ̈ = -g/L·sin(θ) - d·θ̇ + A·cos(ω·t)/L + (vy·cos(φ) - vx·sin(φ))/L
```

2. For φ (azimuthal angle):
```
φ̈ = -d·φ̇ + (vx·cos(φ) + vy·sin(φ))/(L·sin(θ)) + Fwind/(m·L·sin(θ)·cos(φ))
```

3. For ψ (rotation around pendulum arm):
```
ψ̈ = -d·ψ̇
```

## Simulation Method

The simulation uses `scipy.integrate.solve_ivp` with the following steps:

1. Define the state vector: [θ, φ, ψ, θ̇, φ̇, ψ̇]
2. Set up the differential equations as described above
3. Integrate the equations of motion over the specified time interval
4. Calculate the Cartesian coordinates of the bob position from the angles

## Initial Conditions

The system starts with specified initial angles and zero angular velocities.

## Visualization

The animation shows:
- The pendulum bob (red)
- The pendulum arm (blue)
- The pivot point (green)
- Optionally, a trace of the bob's path (orange)

The simulation captures complex behaviors including:
- Chaotic motion
- Influence of the moving pivot point
- Effects of wind forces
- Combined influence of gravity, damping, and external driving forces

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Payload%20and%20inertia%20effects/pendulum_3d.gif" alt="3D pendulum with inertia and wind" width="500" height="500">




# 2- Dynamic Simulation of Payload with Inelastic Cables: Mathematical Model

This document describes the mathematical model used in the payload dynamics simulation. The simulation models a payload suspended by two inelastic cables, with one anchor point fixed and the other moving in a complex pattern over time.

## 1. System Parameters

The simulation uses the following parameters:

- g = 9.81 m/s² - Gravitational acceleration
- m = 1.5 kg - Mass of the payload
- d = 0.3 kg/s - Damping coefficient
- L₁ = 1.5 m - Length of cable 1 (fixed)
- L₂ = 1.5 m - Length of cable 2 (fixed)
- Fixed anchor position: a₁ = (1.0, 1.0, 2.0)
- Moving anchor position: a₂(t) (time-dependent)

## 2. Moving Anchor Position

The second anchor follows a complex path defined by:

```
a₂(t) = [ -1.0 + r_x·cos(ω_x·t)·sin(0.8·t),
          -1.0 + r_y·sin(ω_y·t)·cos(0.6·t),
           2.0 + r_z·sin(ω_z·t) ]
```

Where:
- r_x = 1.5, r_y = 1.5, r_z = 0.7 (radii of motion)
- ω_x = 2.5, ω_y = 2.0, ω_z = 3.0 (angular velocities in rad/s)

However, to maintain system stability, the distance between anchors is constrained to be less than the maximum safe distance:

```
max_safe_distance = (L₁ + L₂) × 0.9
```

If the unconstrained position would exceed this distance, the position is scaled to maintain the maximum safe distance:

```
a₂(t) = a₁ + direction × max_safe_distance
```

where `direction` is the unit vector from a₁ to the unconstrained position.

## 3. Position Constraints

The payload position must satisfy two cable length constraints:

```
|p - a₁| = L₁
|p - a₂| = L₂
```

Where p is the payload position. These constraints are enforced using constrained optimization at each time step, solving:

```
minimize |p - (p_prev + v_prev·Δt)|²
```

Subject to:
```
|p - a₁| - L₁ = 0
|p - a₂| - L₂ = 0
```

This minimizes the distance between the constrained position and the position predicted by the previous state, ensuring the cables maintain their fixed lengths while following plausible dynamics.

## 4. Force Model

For the unconstrained dynamics, the forces acting on the payload are:

- **Gravity Force**: 
  ```
  F_gravity = (0, 0, -m·g)
  ```

- **Damping Force**: 
  ```
  F_damping = -d·v
  ```

- **Tension Forces**: 
  For estimating the initial acceleration (before constraints are applied), temporary tension forces are modeled as:
  
  ```
  F_tension = T₁·u₁ + T₂·u₂
  ```
  
  Where:
  - T₁ = k·(l₁ - L₁) and T₂ = k·(l₂ - L₂) are the magnitudes of tensions
  - k = 1000 is a temporary high spring constant
  - l₁ = |a₁ - p| and l₂ = |a₂ - p| are the current cable lengths
  - u₁ = (a₁ - p)/l₁ and u₂ = (a₂ - p)/l₂ are unit vectors along the cables

- **Total Force**: 
  ```
  F_total = F_gravity + F_damping + F_tension
  ```

- **Acceleration**: 
  ```
  a = F_total/m
  ```

## 5. Integration Scheme

The simulation uses a custom integrator with the following steps at each time t:

1. Calculate unconstrained acceleration: 
   ```
   a(t) = compute_acceleration(p(t-Δt), v(t-Δt), t)
   ```

2. Predict velocity and position using semi-implicit Euler:
   ```
   v_pred(t) = v(t-Δt) + a(t)·Δt
   p_pred(t) = p(t-Δt) + v_pred(t)·Δt
   ```

3. Find constrained position p(t) by solving the optimization problem described in section 3.

4. Update velocity based on the position correction:
   ```
   v(t) = (p(t) - p(t-Δt))/Δt
   ```

This approach ensures that the cable length constraints are satisfied at each time step while maintaining physically plausible dynamics.

## 6. Initial Conditions

The initial payload position is found by minimizing:

```
minimize (l₁ - L₁)² + (l₂ - L₂)²
```

Where l₁ = |p - a₁| and l₂ = |p - a₂(0)|.

The initial velocity is set to zero: v(0) = (0, 0, 0).

This provides a stable starting point for the simulation with the cables at their target lengths and the system at rest.

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Payload%20and%20inertia%20effects/payload_dynamics.gif" alt="Mass particle hold by two cables" width="500" height="500">



# 3- Dual-Cable Payload Dynamics with Loose Cables

## Overview
This simulation models a payload mass suspended by two cables attached to independently moving anchor points, with realistic slack cable dynamics. It includes visualization of cable tension, slack behavior, and inertial effects of the payload.

## Physical Model

### Parameters
| Parameter | Symbol | Value | Units | Description |
|-----------|--------|-------|-------|-------------|
| Gravitational acceleration | g | 9.81 | m/s² | Standard Earth gravity |
| Payload mass | m | 2.0 | kg | Mass of the suspended object |
| Damping coefficient | d | 0.2 | kg/s | Environmental damping |
| Cable 1 length | L₁ | 1.5 | m | Natural length of first cable |
| Cable 2 length | L₂ | 1.5 | m | Natural length of second cable |
| Spring constant | k | 1000.0 | N/m | Stiffness of cables when stretched |

### Key Formulas

#### Anchor Motion Equations

**Anchor 1 Position:**
```python
r_A1(t) = [
    1.0 + r_x * sin(ω_x * t + φ_x),
    1.0 + r_y * cos(ω_y * t + φ_y),
    2.0 + r_z * sin(ω_z * t + φ_z)
]
```
Where:
- r_x = 1.0, r_y = 1.2, r_z = 0.5 (oscillation amplitudes)
- ω_x = 1.8, ω_y = 1.3, ω_z = 2.2 (angular frequencies)
- φ_x = 0.7, φ_y = 1.1, φ_z = 0.9 (phase shifts)

**Anchor 2 Position:**
```python
r_A2(t) = [
    -1.0 + r_x * cos(ω_x * t) * sin(0.8 * t),
    -1.0 + r_y * sin(ω_y * t) * cos(0.6 * t),
    2.0 + r_z * sin(ω_z * t)
]
```
Where:
- r_x = 1.5, r_y = 1.5, r_z = 0.7 (oscillation amplitudes)
- ω_x = 2.5, ω_y = 2.0, ω_z = 3.0 (angular frequencies)

#### Anchor Distance Constraint

The maximum allowed distance between anchors:
```
d_max = α(L₁ + L₂)
```
Where α = 0.9 is a safety factor.

If the actual distance exceeds this maximum, both anchor positions are adjusted:
```
r_A1_new = r_A1 + unit_vector * Δd/2
r_A2_new = r_A2 - unit_vector * Δd/2
```
Where:
- unit_vector = (r_A2 - r_A1)/|r_A2 - r_A1|
- Δd = |r_A2 - r_A1| - d_max (excess distance)

#### Forces on Payload

**Gravitational Force:**
```
F_gravity = (0, 0, -m*g)
```

**Damping Force:**
```
F_damping = -d * velocity
```

**Cable Tension Forces** (for each cable i):
```
F_tension_i = k(l_i - L_i) * unit_vector_i   if l_i > L_i (stretched)
F_tension_i = 0                              if l_i ≤ L_i (slack)
```
Where:
- l_i is current cable length
- L_i is natural cable length
- unit_vector_i is the direction from payload to anchor

**Total Force:**
```
F_total = F_gravity + F_damping + F_tension_1 + F_tension_2
```

#### Motion Integration

**Acceleration:**
```
acceleration = F_total / m
```

**Semi-implicit Euler Method:**
```
velocity_new = velocity + acceleration * Δt
position_new = position + velocity_new * Δt
```

### Slack Cable Modeling

When cables are slack, they follow catenary curves with parametric equation:
```
r(t) = r_payload + x_hat*t - z_hat*s*sin(π*t/d)
```
Where:
- t varies from 0 to d (direct distance between points)
- s = 1.5*(L_i - d) is the sag parameter
- x_hat is direction vector between endpoints
- z_hat is the vertical sag direction

### Initial Condition Finding

The initial position of the payload is determined by finding the minimum of the potential energy function:
```
U(r) = m*g*height + 0.5*k*sum(max(0, l_i - L_i)²)
```
Where:
- The first term represents gravitational potential energy
- The second term represents elastic potential energy in the cables

## Features

- Realistic physics simulation of loose cables (tension only when stretched)
- Visualization of cable state (taut vs. slack) with proper catenary curves
- Dynamic anchor points with complex 3D movement patterns
- Real-time display of cable tensions, lengths, and payload velocity
- Automatic constraint enforcement to ensure physically feasible configurations
- Animation export to GIF format

## Implementation Details

### Simulation Algorithm

1. Calculate all anchor positions for all time steps
2. Find initial equilibrium position
3. For each time step:
   - Calculate current cable lengths
   - Determine if cables are taut or slack
   - Calculate tension forces (zero if slack)
   - Compute total force and acceleration
   - Update velocity and position using semi-implicit Euler
   - Store state variables for visualization

### Visualization

- Taut cables: Straight lines
- Slack cables: Catenary curves
- Color coding: Red (Cable 1), Blue (Cable 2), Green (Payload)
- Status display: Cable status (TAUT/SLACK), tension values, payload speed

## Requirements

- Python 3.x
- NumPy
- Matplotlib
- SciPy

## Usage

Run the script to generate and display the animation:
```
python cable_dynamics.py
```
The simulation will automatically save an animation file named `loose_cable_dynamics.gif` in the current directory.

## Limitations

- Cable mass is neglected (massless cable approximation)
- Simplified air resistance model (linear damping)
- No collision detection between cables
- No elastic deformation of cables beyond simple spring model when taut

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Payload%20and%20inertia%20effects/loose_cable_dynamics.gif" alt="Payload hanged from loose cables" width="500" height="500">
