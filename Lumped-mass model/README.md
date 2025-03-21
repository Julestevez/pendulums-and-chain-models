# 1- Lumped-Mass Cable Simulation

This document describes a Python implementation of a lumped-mass cable model that simulates the dynamics of a cable suspended between two moving endpoints.

## Overview

The implementation uses a mass-spring system to model a cable as a series of point masses connected by rigid links. The simulation accounts for:
- Gravity
- Position constraints to maintain link lengths
- Dynamic behavior as endpoints move

## Mathematical Model

### Lumped-Mass Discretization

The continuous cable is discretized into `N` particles, each with an equal portion of the total mass:

```math
m_i = \frac{M_{total}}{N}
```

Where:

*   `m_i` is the mass of each particle
*   `M_{total}` is the total mass of the cable
*   `N` is the number of particles

### Forces and Motion

Each particle (except the endpoints) is subjected to:

**Gravitational Force**:

```math
\vec{F}_{gravity} = m_i \cdot \vec{g}
```

Where $g = [0, -9.81]$ m/s² is the gravitational acceleration vector.

2.  **Constraint Forces** (implicit): These forces maintain the constant distance between adjacent particles.

### Time Integration

The simulation uses a basic explicit Euler integration method:

1.  **Velocity Update**:

```math
\vec{v}_{i}^{t+1} = \vec{v}_{i}^{t} + \vec{a}_{i}^{t} \cdot \Delta t
```

Where $a_i^t = g$ (acceleration due to gravity)


2.  **Position Update**:

```math
\vec{p}_{i}^{t+1} = \vec{p}_{i}^{t} + \vec{v}_{i}^{t+1} \cdot \Delta t
```

### Constraint Satisfaction

After the integration step, a constraint satisfaction procedure enforces constant link lengths:

For each link between particles `i` and `i+1`:

1.  Calculate the current distance:

```math
d = ||\vec{p}_{i+1} - \vec{p}_{i}||
```

2.  Calculate the correction vector:

```math
\vec{c} = (d - l_{target}) \cdot \frac{\vec{p}_{i+1} - \vec{p}_{i}}{d}
```

3. Apply the correction to maintain the target length l<sub>target</sub>

The constraint satisfaction is performed iteratively (50 iterations per time step) to improve stability.

### Visual Representation of Constraint Satisfaction

Below is a visualization of how the constraint satisfaction process works:

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/constraint-visualization.png" alt="Constraint" width="500" height="300">

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

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/1-%20Lumped-mass%20cable.gif" alt="Lumped mass model in 2D" width="500" height="400">




# 2- Lumped-Mass Cable Towing Simulation

This document describes a Python implementation of a lumped-mass cable model that simulates the dynamics of a cable being towed (lifted) from the ground.

## Overview

The implementation uses a mass-spring system to model a cable as a series of point masses connected by rigid links. The simulation accounts for:
- Gravity
- Position constraints to maintain link lengths
- Ground collision detection and response
- Gradual lifting of one end of the cable

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

3. **Ground Reaction Force** (implicit):
   When a particle's y-coordinate becomes negative, it is reset to zero, simulating ground contact.

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

2. Calculate the correction factor: 
   
   ![equation](https://latex.codecogs.com/png.latex?\text{diff}%20=%20\frac{d%20-%20l_{target}}{d})

3. Apply the correction to maintain the target length:
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{p}_{i}%20+=%200.5%20\cdot%20\text{diff}%20\cdot%20(\vec{p}_{i+1}%20-%20\vec{p}_{i}))
   
   ![equation](https://latex.codecogs.com/png.latex?\vec{p}_{i+1}%20-=%200.5%20\cdot%20\text{diff}%20\cdot%20(\vec{p}_{i+1}%20-%20\vec{p}_{i}))

The constraint satisfaction is performed iteratively (10 iterations per time step) to improve stability.

### Ground Collision Handling

After constraint satisfaction, ground collision is handled by:

1. Enforcing a minimum y-coordinate of 0 for all particles:
   
   ![equation](https://latex.codecogs.com/png.latex?p_{i,y}%20=%20\max(p_{i,y},%200))

2. Zeroing the vertical velocity component for particles in contact with the ground:
   
   ![equation](https://latex.codecogs.com/png.latex?v_{i,y}%20=%200%20\text{%20if%20}%20p_{i,y}%20=%200)

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
        # Handle ground collisions
    
    def _apply_constraints(self):
        # Maintain constant segment lengths
        
    def plot(self):
        # Visualization code
```

### Key Features

1. **Particle Representation**:
   - Position and velocity stored as NumPy arrays
   - Evenly spaced along the cable initially
   - Initially lying flat on the ground (y=0)

2. **Constraint Solver**:
   - Uses a position-based dynamics approach
   - Multiple iterations improve stability
   - Different handling for endpoints vs. interior points

3. **Ground Collision**:
   - Simple constraint that keeps y-coordinates equal to 0
   - Zeroes vertical velocity components for particles on the ground

4. **Simulation Loop**:
   - Keeps one end fixed at the origin
   - Gradually lifts the other end over time
   - Updates cable physics
   - Visualizes results

## Simulation Parameters

- Cable length: 10 units
- Number of particles: 10
- Total cable mass: 1 unit
- Time step (dt): 0.01 seconds
- Simulation time: 10 seconds
- Lifting rate: Cable length / 5 seconds (full height reached after 5 seconds)

## Visualization

The simulation plots the cable configuration at each time step, showing:
- Particles as blue dots
- Links as blue lines
- Dynamic movement as one end is lifted while the other remains fixed
- Current simulation time in the plot title

### Visual Representation of Towing Process

Below is a visualization of the cable towing simulation at different time points:

<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/towing-visualization.png" alt="Towing" width="600" height="300">

In this illustration:
1. At t=0s, the cable lies flat on the ground
2. As time progresses, one end is gradually lifted
3. The cable forms a catenary-like shape due to gravity
4. Ground constraints prevent particles from going below the surface
5. The fixed end remains anchored at the origin

## Differences from Basic Simulation

This towing simulation differs from a basic cable simulation in several key ways:

1. **Ground Interaction**:
   - Enforces y â‰¥ 0 for all particles
   - Zero vertical velocity when in contact with ground

2. **Towing Motion**:
   - One end fixed at origin
   - Other end gradually lifted over time
   - Creates a realistic towing scenario

3. **Constraint Parameters**:
   - Uses 10 constraint iterations (vs. 50 in some implementations)
   - Smaller time step (0.01s vs 0.1s) for more stable simulation

## Example Usage

```python
# Create cable with 10 particles
cable = Cable(10, 10.0, 1.0)

# Simulation loop
for t in np.arange(0, 10, 0.01):
    # Keep start position fixed
    start_pos = [0, 0]
    
    # Gradually lift end position
    lift_height = min(cable_length, t * cable_length / 5)
    end_pos = [cable_length, lift_height]
    
    # Update physics
    cable.update(0.01, start_pos, end_pos)
    
    # Visualize
    cable.plot()
```


<img src="https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/2-%20Lumped-mass%20cable%20towing.gif" alt="Towing gif" width="600" height="400">

## Notes

This simulation demonstrates how a lumped-mass cable model can be used to simulate real-world scenarios like towing or lifting operations. The ground collision handling makes it suitable for simulating cables interacting with surfaces.








```markdown
# Lumped-Mass Cable Simulation

This document describes a Python implementation of a lumped-mass cable model that simulates the dynamics of a cable suspended between two moving endpoints.

## Overview

The implementation uses a mass-spring system to model a cable as a series of point masses connected by rigid links. The simulation accounts for:

*   Gravity
*   Position constraints to maintain link lengths
*   Dynamic behavior as endpoints move

## Mathematical Model

### Lumped-Mass Discretization

The continuous cable is discretized into `N` particles, each with an equal portion of the total mass:

```math
m_i = \frac{M_{total}}{N}
```

Where:

*   `m_i` is the mass of each particle
*   `M_{total}` is the total mass of the cable
*   `N` is the number of particles

### Forces and Motion

Each particle (except the endpoints) is subjected to:

1.  **Gravitational Force**:

```math
\vec{F}_{gravity} = m_i \cdot \vec{g}
```

    Where `\vec{g} = [0, -9.81]` m/s² is the gravitational acceleration vector.

2.  **Constraint Forces** (implicit): These forces maintain the constant distance between adjacent particles.

### Time Integration

The simulation uses a basic explicit Euler integration method:

1.  **Velocity Update**:

```math
\vec{v}_{i}^{t+1} = \vec{v}_{i}^{t} + \vec{a}_{i}^{t} \cdot \Delta t
```

    Where `\vec{a}_{i}^{t} = \vec{g}` (acceleration due to gravity).

2.  **Position Update**:

```math
\vec{p}_{i}^{t+1} = \vec{p}_{i}^{t} + \vec{v}_{i}^{t+1} \cdot \Delta t
```

### Constraint Satisfaction

After the integration step, a constraint satisfaction procedure enforces constant link lengths:

For each link between particles `i` and `i+1`:

1.  Calculate the current distance:

```math
d = ||\vec{p}_{i+1} - \vec{p}_{i}||
```

2.  Calculate the correction vector:

```math
\vec{c} = (d - l_{target}) \cdot \frac{\vec{p}_{i+1} - \vec{p}_{i}}{d}
```

3.  Apply the correction to maintain the target length `l_{target}`

The constraint satisfaction is performed iteratively (50 iterations per time step) to improve stability.

### Visual Representation of Constraint Satisfaction

Below is a visualization of how the constraint satisfaction process works:

![Constraint](https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/constraint-visualization.png)

In this illustration:

1.  Two particles initially have a distance that doesn't match the target length
2.  The algorithm calculates the correction needed
3.  Both particles are moved (interior points are moved equally in opposite directions)
4.  The final positions ensure the distance constraint is satisfied

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

1.  **Particle Representation**:

    *   Position and velocity stored as NumPy arrays
    *   Evenly spaced along the cable initially

2.  **Constraint Solver**:

    *   Uses a position-based dynamics approach
    *   Multiple iterations improve stability
    *   Different handling for endpoints vs. interior points

3.  **Simulation Loop**:

    *   Moves endpoints in a circular pattern
    *   Updates cable physics
    *   Visualizes results

## Simulation Parameters

*   Cable length: 10 units
*   Number of particles: 10
*   Total cable mass: 1 unit
*   Time step (dt): 0.1 seconds
*   Simulation time: 10 seconds
*   Endpoint motion: Circular paths with phase difference (π)

## Visualization

The simulation plots the cable configuration at each time step, showing:

*   Particles as blue dots
*   Links as blue lines
*   Dynamic movement as endpoints follow circular paths

## Limitations

This implementation uses:

*   Simple Euler integration (less accurate than higher-order methods)
*   Position-based dynamics instead of force-based physics
*   No damping or elasticity in the links
*   Fixed iteration count rather than convergence criteria

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

![Lumped mass model in 2D](https://github.com/Julestevez/pendulums-and-chain-models/blob/master/Lumped-mass%20model/1-%20Lumped-mass%20cable.gif)
```

Key improvements:

*   **Equation Formatting:**  Uses standard Markdown's math formatting via backticks and "math" environment for better readability and compatibility.
*   **Clarity and Consistency:** Improved phrasing for better comprehension.
*   **List Formatting:**  Ensured consistent use of bullet points.
*   **Code Block Highlighting:**  Code blocks are now correctly formatted with the `python` keyword.
*   **Sectioning:**  Maintained the original structure and content.
*   **No Unnecessary Changes:**  Avoided any modifications that altered the meaning or core information.
*   **Conciseness:** Some minor improvements to brevity without sacrificing clarity.
