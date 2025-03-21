# Lumped-Mass Cable Simulation

This document describes a Python implementation of a lumped-mass cable model that simulates the dynamics of a cable suspended between two moving endpoints.

## Overview

The implementation uses a mass-spring system to model a cable as a series of point masses connected by rigid links. The simulation accounts for:
- Gravity
- Position constraints to maintain link lengths
- Dynamic behavior as endpoints move

## Mathematical Model

### Lumped-Mass Discretization

The continuous cable is discretized into `N` particles, each with an equal portion of the total mass:

![equation](https://latex.codecogs.com/png.latex?m_i%20=%20\frac{M_{total}}{N})

Where:
- m<sub>i</sub> is the mass of each particle
- M<sub>total</sub> is the total mass of the cable
- N is the number of particles

### Forces and Motion

Each particle (except the endpoints) is subjected to:

1. **Gravitational Force**:
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{F}_{gravity}%20=%20m_i%20\cdot%20\vec{g})
   
   Where ![equation](https://latex.codecogs.com/png.latex?\vec{g}%20=%20[0,%20-9.81]) m/s² is the gravitational acceleration vector.

2. **Constraint Forces** (implicit):
   These forces maintain the constant distance between adjacent particles.

### Time Integration

The simulation uses a basic explicit Euler integration method:

1. **Velocity Update**:
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{v}_{i}^{t+1}%20=%20\vec{v}_{i}^{t}%20+%20\vec{a}_{i}^{t}%20\cdot%20\Delta%20t)
   
   Where ![equation](https://latex.codecogs.com/png.latex?\vec{a}_{i}^{t}%20=%20\vec{g}) (acceleration due to gravity)

2. **Position Update**:
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{p}_{i}^{t+1}%20=%20\vec{p}_{i}^{t}%20+%20\vec{v}_{i}^{t+1}%20\cdot%20\Delta%20t)

### Constraint Satisfaction

After the integration step, a constraint satisfaction procedure enforces constant link lengths:

For each link between particles i and i+1:

1. Calculate the current distance: 
   
   ![equation](https://latex.codecogs.com/png.latex?d%20=%20\|\vec{p}_{i+1}%20-%20\vec{p}_{i}\|)

2. Calculate the correction vector: 
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{c}%20=%20(d%20-%20l_{target})%20\cdot%20\frac{\vec{p}_{i+1}%20-%20\vec{p}_{i}}{d})

3. Apply the correction to maintain the target length l<sub>target</sub>

The constraint satisfaction is performed iteratively (50 iterations per time step) to improve stability.

### Visual Representation of Constraint Satisfaction

Below is a visualization of how the constraint satisfaction process works:

![Constraint Satisfaction Process](constraint-satisfaction.png)

In this illustration:
1. Two particles initially have a distance that doesn't match the target length
2. The algorithm calculates the correction needed
3. Both particles are moved (interior points are moved equally in opposite directions)
4. The final positions ensure the distance constraint is satisfied

The constraint satisfaction is performed iteratively (50 iterations per time step) to improve stability.

## Code Implementation

### Cable Class

The `Cable` class encapsulates the cable simulation:

```python
class Cable:
    def __init__(self, num_particles, cable_length, mass):
        # Initialize parameters
        # Create evenly spaced particles
        # Set up masses and physical properties
        
    def update(self, dt, start_pos, end_pos):
        # Update positions based on velocities
        # Apply gravitational acceleration
        # Fix endpoint positions
        # Apply constraints
    
    def _apply_constraints(self):
        # Iterate multiple times for stability
        # Maintain constant segment lengths
        
    def plot(self):
        # Visualization code
```

### Key Features

1. **Particle Representation**:
   - Position and velocity stored as NumPy arrays
   - Evenly spaced along the cable initially

2. **Constraint Solver**:
   - Uses a position-based dynamics approach
   - Multiple iterations improve stability
   - Different handling for endpoints vs. interior points

3. **Simulation Loop**:
   - Moves endpoints in a circular pattern
   - Updates cable physics
   - Visualizes results

## Simulation Parameters

- Cable length: 10 units
- Number of particles: 10
- Total cable mass: 1 unit
- Time step (dt): 0.1 seconds
- Simulation time: 10 seconds
- Endpoint motion: Circular paths with phase difference (π)

## Visualization

The simulation plots the cable configuration at each time step, showing:
- Particles as blue dots
- Links as blue lines
- Dynamic movement as endpoints follow circular paths

## Limitations

This implementation uses:
- Simple Euler integration (less accurate than higher-order methods)
- Position-based dynamics instead of force-based physics
- No damping or elasticity in the links
- Fixed iteration count rather than convergence criteria

## Example Usage

```python
# Create cable with 10 particles
cable = Cable(10, 10.0, 1.0)

# Simulation loop
for t in np.arange(0, 10, 0.1):
    # Update endpoint positions
    start_pos = [np.cos(t), np.sin(t) + 1]
    end_pos = [6 + np.cos(t + np.pi), np.sin(t + np.pi) + 1]
    
    # Update physics
    cable.update(0.1, start_pos, end_pos)
    
    # Visualize
    cable.plot()
```

## Notes

The constraint solver is the core of this simulation. It ensures each segment maintains a constant length, effectively making the links inextensible while allowing the system to respond to gravity and endpoint movement.
