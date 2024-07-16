"""el archivi claude_funciona_bien.py tiene segmentos de cable que se alargan y acortan
le he pedido a Claude que corrija eso"""
import numpy as np
import matplotlib.pyplot as plt

class Cable:
    def __init__(self, num_particles, cable_length, mass):
        self.num_particles = num_particles
        self.cable_length = cable_length
        self.mass = mass
        
        # Initialize particle positions (evenly spaced)
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
        self._apply_constraints()
    
    def _apply_constraints(self): # este código es el que parece que hace que los segmentos tengan longitud constante
        for _ in range(50):  # Increase iterations for better stability
            for i in range(self.num_particles - 1):
                p1, p2 = self.particles[i], self.particles[i+1]
                delta = p2 - p1
                dist = np.linalg.norm(delta)
                if dist == 0:
                    continue  # Avoid division by zero
                correction = (dist - self.segment_length) * delta / dist
                
                if i > 0 and i < self.num_particles - 2:
                    self.particles[i] += 0.5 * correction
                    self.particles[i+1] -= 0.5 * correction
                elif i == 0:
                    self.particles[i+1] -= correction
                else:  # i == self.num_particles - 2
                    self.particles[i] += correction
    
    def plot(self):
        plt.clf()
        plt.plot(self.particles[:, 0], self.particles[:, 1], 'bo-')
        plt.xlim(-self.cable_length/2, self.cable_length * 1.5)
        plt.ylim(-self.cable_length, self.cable_length)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.pause(0.01)

# Simulation parameters
cable_length = 10
num_particles = 10
total_mass = 1
simulation_time = 10
dt = 0.1

# Create cable
cable = Cable(num_particles, cable_length, total_mass)

# Simulation loop
for t in np.arange(0, simulation_time, dt):
    # Update robot positions (example: circular motion)
    start_pos = [1*np.cos(t), 1 * np.sin(t) + 1]
    end_pos = [0.6*cable_length + 1 * np.cos(t + np.pi), 1 * np.sin(t + np.pi) + 1]
    
    # Update cable
    cable.update(dt, start_pos, end_pos)
    
    # Plot current state
    cable.plot()

plt.show()