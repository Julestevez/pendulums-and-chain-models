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

