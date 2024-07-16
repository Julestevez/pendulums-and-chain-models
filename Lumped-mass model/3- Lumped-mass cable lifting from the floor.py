"""I would like to lift completely the cable from the floor, 
as if a drone was lifting it. Could you also reduce the number of mass particles to 6?"""

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
        
    def update(self, dt, drone_pos):
        # Update positions based on current velocities
        self.particles += self.velocities * dt
        
        # Apply gravity
        self.velocities += self.gravity * dt
        
        # Set drone-lifted end position
        self.particles[-1] = drone_pos
        self.velocities[-1] = [0, 0]
        
        # Apply constraints to maintain link lengths
        for _ in range(10):  # Iterate multiple times for better stability
            self._apply_constraints()
        
        # Ground collision (only if any part of the cable is still touching)
        if np.any(self.particles[:, 1] <= 0):
            self.particles[:, 1] = np.maximum(self.particles[:, 1], 0)
            self.velocities[self.particles[:, 1] == 0, 1] = 0
    
    def _apply_constraints(self):
        for i in range(self.num_particles - 1):
            p1, p2 = self.particles[i], self.particles[i+1]
            delta = p2 - p1
            dist = np.linalg.norm(delta)
            diff = (dist - self.segment_length) / dist
            
            if i < self.num_particles - 2:
                p1 += 0.5 * diff * delta
                p2 -= 0.5 * diff * delta
                self.particles[i] = p1
                self.particles[i+1] = p2
            else:
                # For the last segment, only move the non-drone end
                p1 += diff * delta
                self.particles[i] = p1
    
    def plot(self):
        plt.clf()
        plt.plot(self.particles[:, 0], self.particles[:, 1], 'bo-')
        plt.plot(self.particles[-1, 0], self.particles[-1, 1], 'ro', markersize=10)  # Drone position
        plt.xlim(-1, self.cable_length + 1)
        plt.ylim(-1, self.cable_length + 1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title(f"Time: {t:.2f}s")
        plt.pause(0.01)

# Simulation parameters
cable_length = 10
num_particles = 6  # Changed to 6 particles
total_mass = 1
simulation_time = 10
dt = 0.01

# Create cable
cable = Cable(num_particles, cable_length, total_mass)

# Simulation loop
for t in np.arange(0, simulation_time, dt):
    # Calculate drone position (lifting up and then moving to the side)
    lift_height = min(cable_length, t * cable_length / 3)  # Lift over 3 seconds
    side_movement = max(0, (t - 3) * cable_length / 4)  # Move sideways after 3 seconds
    drone_pos = [cable_length - side_movement, lift_height]
    
    # Update cable
    cable.update(dt, drone_pos)
    
    # Plot current state
    cable.plot()

plt.show()