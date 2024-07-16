import numpy as np
import matplotlib.pyplot as plt

class Cable:
    def __init__(self, num_particles, cable_length, mass):
        self.num_particles = num_particles
        self.cable_length = cable_length
        self.mass = mass
        
        # Initialize particle positions (lying flat on the ground)
        self.particles = np.linspace([0, 0], [cable_length, 0], num_particles)
        
        # Calculate segment length
        self.segment_length = cable_length / (num_particles - 1)
        
        # Initialize particle masses (equal distribution)
        self.particle_mass = mass / num_particles
        
        # Initialize velocities
        self.velocities = np.zeros_like(self.particles)
        
        # Gravity
        self.gravity = np.array([0, -9.81])
        
    def update(self, dt, start_pos, end_pos):
        # Update positions based on current velocities
        self.particles += self.velocities * dt
        
        # Apply gravity
        self.velocities += self.gravity * dt
        
        # Set end positions
        self.particles[0] = start_pos
        self.particles[-1] = end_pos
        self.velocities[0] = [0, 0]
        self.velocities[-1] = [0, 0]
        
        # Apply constraints to maintain link lengths
        for _ in range(10):  # Iterate multiple times for better stability
            self._apply_constraints()
        
        # Ground collision
        self.particles[:, 1] = np.maximum(self.particles[:, 1], 0)
        self.velocities[self.particles[:, 1] == 0, 1] = 0
    
    def _apply_constraints(self):
        for i in range(self.num_particles - 1):
            p1, p2 = self.particles[i], self.particles[i+1]
            delta = p2 - p1
            dist = np.linalg.norm(delta)
            diff = (dist - self.segment_length) / dist
            
            if i > 0:
                p1 += 0.5 * diff * delta
                self.particles[i] = p1
            if i < self.num_particles - 2:
                p2 -= 0.5 * diff * delta
                self.particles[i+1] = p2
    
    def plot(self):
        plt.clf()
        plt.plot(self.particles[:, 0], self.particles[:, 1], 'bo-')
        plt.xlim(0, self.cable_length * 1.2)
        plt.ylim(0, self.cable_length)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title(f"Time: {t:.2f}s")
        plt.pause(0.01)

# Simulation parameters
cable_length = 10
num_particles = 10  # Changed to 10 particles
total_mass = 1
simulation_time = 10
dt = 0.01

# Create cable
cable = Cable(num_particles, cable_length, total_mass)

# Simulation loop
for t in np.arange(0, simulation_time, dt):
    # Keep the start position fixed on the ground
    start_pos = [0, 0]
    
    # Gradually lift the end position
    lift_height = min(cable_length, t * cable_length / 5)  # Lift over 5 seconds
    end_pos = [cable_length, lift_height]
    
    # Update cable
    cable.update(dt, start_pos, end_pos)
    
    # Plot current state
    cable.plot()

plt.show()