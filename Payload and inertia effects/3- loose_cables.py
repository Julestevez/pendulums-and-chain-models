#In this code, I try to introduce loose cables dynamics. Cables don't push, they just pull.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from scipy.integrate import solve_ivp

# Constants
g = 9.81  # gravitational acceleration (m/s²)
m = 2.0   # mass of the payload (kg)
d = 0.2   # damping coefficient (kg/s)
L1 = 1.5  # length of cable 1 (m) - FIXED
L2 = 1.5  # length of cable 2 (m) - FIXED

# Function to get the moving anchor1 position at time t
def get_anchor1(t):
    # Motion parameters for anchor1 (different pattern than anchor2)
    radius_x = 1.0
    radius_y = 1.2
    radius_z = 0.5
    omega_x = 1.8
    omega_y = 1.3
    omega_z = 2.2
    phase_x = 0.7  # Phase shift to create interesting patterns with anchor2
    phase_y = 1.1
    phase_z = 0.9
    
    # Calculate position
    pos = np.array([
        1.0 + radius_x * np.sin(omega_x * t + phase_x),
        1.0 + radius_y * np.cos(omega_y * t + phase_y),
        2.0 + radius_z * np.sin(omega_z * t + phase_z)
    ])
    
    return pos

# Function to get the moving anchor2 position at time t
def get_anchor2(t):
    # Even faster motion parameters
    radius_x = 1.5
    radius_y = 1.5
    radius_z = 0.7
    omega_x = 2.5  # much faster angular velocity (rad/s)
    omega_y = 2.0
    omega_z = 3.0
    
    # Calculate unconstrained position with more aggressive, non-circular movement
    unconstrained_pos = np.array([
        -1.0 + radius_x * np.cos(omega_x * t) * np.sin(0.8 * t),
        -1.0 + radius_y * np.sin(omega_y * t) * np.cos(0.6 * t),
        2.0 + radius_z * np.sin(omega_z * t)
    ])
    
    return unconstrained_pos

# Calculate maximum allowed distance between anchors
# For two cables of length L1 and L2, the maximum distance between anchors is L1 + L2
max_anchor_distance = L1 + L2
# For stability, we'll keep it a bit less than the theoretical maximum
safety_factor = 0.9
max_safe_distance = max_anchor_distance * safety_factor

# Function to ensure anchors don't exceed maximum distance
def constrain_anchors(anchor1_pos, anchor2_pos):
    # Calculate current distance
    vec_between = anchor2_pos - anchor1_pos
    distance = np.linalg.norm(vec_between)
    
    # If too far, adjust both positions to maintain their midpoint
    if distance > max_safe_distance:
        # Direction vector
        direction = vec_between / distance
        
        # Calculate how much to move each anchor
        excess_distance = distance - max_safe_distance
        move_distance = excess_distance / 2
        
        # Adjust both anchors equally
        adjusted_anchor1 = anchor1_pos + direction * move_distance
        adjusted_anchor2 = anchor2_pos - direction * move_distance
        
        return adjusted_anchor1, adjusted_anchor2
    else:
        # No adjustment needed
        return anchor1_pos, anchor2_pos

# Direct physics integration with loose cable dynamics
def simulate_loose_cables(initial_pos, initial_vel, anchor1_positions, anchor2_positions, times):
    num_steps = len(times)
    dt = times[1] - times[0]
    
    # Arrays to store results
    positions = np.zeros((num_steps, 3))
    velocities = np.zeros((num_steps, 3))
    cable1_tension = np.zeros(num_steps)
    cable2_tension = np.zeros(num_steps)
    
    # Initial conditions
    positions[0] = initial_pos
    velocities[0] = initial_vel
    
    # Spring constant for when cables become taut
    k = 1000.0  # N/m - high value to approximate inelastic behavior
    
    # Simulation loop
    for i in range(1, num_steps):
        pos = positions[i-1]
        vel = velocities[i-1]
        
        # Get anchor positions
        anchor1_pos = anchor1_positions[i-1]
        anchor2_pos = anchor2_positions[i-1]
        
        # Vector from payload to anchors
        vec_to_anchor1 = anchor1_pos - pos
        vec_to_anchor2 = anchor2_pos - pos
        
        # Current lengths of the cables
        current_length1 = np.linalg.norm(vec_to_anchor1)
        current_length2 = np.linalg.norm(vec_to_anchor2)
        
        # Unit vectors pointing from payload to anchors
        if current_length1 > 0:
            unit_vec1 = vec_to_anchor1 / current_length1
        else:
            unit_vec1 = np.array([0, 0, 0])
            
        if current_length2 > 0:
            unit_vec2 = vec_to_anchor2 / current_length2
        else:
            unit_vec2 = np.array([0, 0, 0])
        
        # Calculate tension forces (only when cable is extended beyond rest length)
        # Loose cables: no tension when shorter than rest length
        tension1 = max(0, k * (current_length1 - L1)) if current_length1 > L1 else 0
        tension2 = max(0, k * (current_length2 - L2)) if current_length2 > L2 else 0
        
        # Store tension values for visualization
        cable1_tension[i-1] = tension1
        cable2_tension[i-1] = tension2
        
        # Forces
        F_tension1 = tension1 * unit_vec1
        F_tension2 = tension2 * unit_vec2
        F_gravity = np.array([0, 0, -m * g])
        F_damping = -d * vel
        
        # Total force
        F_total = F_tension1 + F_tension2 + F_gravity + F_damping
        
        # Acceleration
        acc = F_total / m
        
        # Semi-implicit Euler integration
        vel_new = vel + acc * dt
        pos_new = pos + vel_new * dt
        
        # Store results
        velocities[i] = vel_new
        positions[i] = pos_new
    
    cable1_tension[-1] = cable1_tension[-2]  # Copy last value for consistent array lengths
    cable2_tension[-1] = cable2_tension[-2]
    
    return positions, velocities, cable1_tension, cable2_tension

# Time settings
t_max = 10.0  # seconds
dt = 0.02     # smaller timestep for smoother animation
num_steps = int(t_max / dt)
times = np.linspace(0, t_max, num_steps)

# Initialize arrays to store anchor positions and distances
anchor1_positions = np.zeros((num_steps, 3))
anchor2_positions = np.zeros((num_steps, 3))
anchor_distances = np.zeros(num_steps)

# Calculate all anchor positions in advance
for i in range(num_steps):
    t = times[i]
    anchor1_pos = get_anchor1(t)
    anchor2_pos = get_anchor2(t)
    
    # Ensure anchors aren't too far apart
    anchor1_pos, anchor2_pos = constrain_anchors(anchor1_pos, anchor2_pos)
    
    anchor1_positions[i] = anchor1_pos
    anchor2_positions[i] = anchor2_pos
    anchor_distances[i] = np.linalg.norm(anchor1_pos - anchor2_pos)

# Find initial equilibrium position
def find_initial_position(anchor1_pos, anchor2_pos):
    # Initial guess halfway between anchors but lower
    midpoint = (anchor1_pos + anchor2_pos) / 2
    initial_guess = np.array([midpoint[0], midpoint[1], midpoint[2] - 0.5])
    
    # Function to minimize: potential energy
    def objective(pos):
        # Distance to anchors
        d1 = np.linalg.norm(pos - anchor1_pos)
        d2 = np.linalg.norm(pos - anchor2_pos)
        
        # Potential energy from gravity and stretched cables
        # Only count stretching beyond rest length
        cable1_stretch = max(0, d1 - L1)
        cable2_stretch = max(0, d2 - L2)
        
        # Using a high spring constant to approximate inelastic behavior
        k = 1000.0
        
        # Potential energy: gravity + spring energy
        return m * g * pos[2] + 0.5 * k * (cable1_stretch**2 + cable2_stretch**2)
    
    # Solve for minimum energy position
    result = minimize(objective, initial_guess, method='L-BFGS-B', tol=1e-10)
    
    return result.x

# Get initial position and zero initial velocity
initial_pos = find_initial_position(anchor1_positions[0], anchor2_positions[0])
initial_vel = np.zeros(3)

# Run the simulation
positions, velocities, cable1_tension, cable2_tension = simulate_loose_cables(
    initial_pos, initial_vel, anchor1_positions, anchor2_positions, times)

# Create figure for animation
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Set up static elements
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Dynamic Simulation with Loose Cables')

# Initialize plot elements
anchor1_point, = ax.plot([], [], [], 'ro', markersize=10, label='Anchor 1')
anchor2_point, = ax.plot([], [], [], 'bo', markersize=10, label='Anchor 2')
payload_point, = ax.plot([], [], [], 'go', markersize=12, label='Payload')
cable1_line, = ax.plot([], [], [], 'r-', linewidth=2, label='Cable 1')
cable2_line, = ax.plot([], [], [], 'b-', linewidth=2, label='Cable 2')
trajectory, = ax.plot([], [], [], 'g--', linewidth=1, alpha=0.5, label='Trajectory')

# Add loose cable visualization with catenary curve shape
def catenary_curve(start, end, length, num_points=20):
    """Generate points for a catenary curve between two points when cable is slack"""
    # Vector from start to end
    direct_vec = end - start
    direct_dist = np.linalg.norm(direct_vec)
    
    # If the cable is nearly taut, just return a straight line
    if direct_dist > length * 0.99:
        t = np.linspace(0, 1, num_points)
        return np.array([start + direct_vec * ti for ti in t])
    
    # Calculate slack amount
    slack = length - direct_dist
    
    # Unit vector in the direction from start to end
    if direct_dist > 0:
        unit_vec = direct_vec / direct_dist
    else:
        # If points are coincident, default to vertical
        unit_vec = np.array([0, 0, 1])
    
    # Create a coordinate system
    # x-axis along the direct vector
    x_axis = unit_vec
    
    # z-axis has a component pointing upward (gravity direction)
    # but needs to be perpendicular to x_axis
    z_component = np.array([0, 0, 1])
    z_axis = z_component - np.dot(z_component, x_axis) * x_axis
    if np.linalg.norm(z_axis) > 0:
        z_axis = z_axis / np.linalg.norm(z_axis)
    else:
        # If x_axis is vertical, use y-axis as the perpendicular direction
        z_axis = np.array([0, 1, 0])
    
    # y-axis is perpendicular to both x and z
    y_axis = np.cross(z_axis, x_axis)
    if np.linalg.norm(y_axis) > 0:
        y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Parameter for catenary sag (higher value = more sag)
    sag_parameter = slack * 1.5
    
    # Generate catenary curve points
    t = np.linspace(0, direct_dist, num_points)
    catenary_points = []
    
    for ti in t:
        # Position along the direct vector
        x_pos = ti
        
        # Catenary curve shape in the sag direction
        # Maximum sag at the midpoint
        relative_pos = ti / direct_dist
        sag = sag_parameter * np.sin(np.pi * relative_pos)
        
        # Calculate 3D point
        point = start + x_axis * ti - z_axis * sag
        catenary_points.append(point)
    
    return np.array(catenary_points)

# Text annotations for information
time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, color='black')
cable1_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes, color='red')
cable2_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes, color='blue')
speed_text = ax.text2D(0.02, 0.80, '', transform=ax.transAxes, color='green')
anchor_dist_text = ax.text2D(0.02, 0.75, '', transform=ax.transAxes, color='purple')

# Set limits for better visualization
ax.set_xlim([-3.0, 3.0])
ax.set_ylim([-3.0, 3.0])
ax.set_zlim([0, 3.5])

# Animation update function
def update(frame):
    t = times[frame]
    payload_pos = positions[frame]
    payload_vel = velocities[frame]
    
    # Get the current anchor positions
    anchor1_pos = anchor1_positions[frame]
    anchor2_pos = anchor2_positions[frame]
    
    # Get the current tensions
    tension1 = cable1_tension[frame]
    tension2 = cable2_tension[frame]
    
    # Calculate current cable lengths
    current_length1 = np.linalg.norm(anchor1_pos - payload_pos)
    current_length2 = np.linalg.norm(anchor2_pos - payload_pos)
    
    # Calculate payload speed
    speed = np.linalg.norm(payload_vel)
    
    # Calculate distance between anchors
    anchor_distance = np.linalg.norm(anchor1_pos - anchor2_pos)
    
    # Update anchor positions
    anchor1_point.set_data([anchor1_pos[0]], [anchor1_pos[1]])
    anchor1_point.set_3d_properties([anchor1_pos[2]])
    
    anchor2_point.set_data([anchor2_pos[0]], [anchor2_pos[1]])
    anchor2_point.set_3d_properties([anchor2_pos[2]])
    
    # Update payload position
    payload_point.set_data([payload_pos[0]], [payload_pos[1]])
    payload_point.set_3d_properties([payload_pos[2]])
    
    # Update cable visualizations with catenary curves if slack
    # Cable 1
    if current_length1 < L1 * 0.99:  # Cable is slack
        catenary_points1 = catenary_curve(payload_pos, anchor1_pos, L1, num_points=20)
        cable1_line.set_data(catenary_points1[:, 0], catenary_points1[:, 1])
        cable1_line.set_3d_properties(catenary_points1[:, 2])
        cable1_status = "SLACK"
    else:  # Cable is taut
        cable1_line.set_data([anchor1_pos[0], payload_pos[0]], [anchor1_pos[1], payload_pos[1]])
        cable1_line.set_3d_properties([anchor1_pos[2], payload_pos[2]])
        cable1_status = "TAUT"
    
    # Cable 2
    if current_length2 < L2 * 0.99:  # Cable is slack
        catenary_points2 = catenary_curve(payload_pos, anchor2_pos, L2, num_points=20)
        cable2_line.set_data(catenary_points2[:, 0], catenary_points2[:, 1])
        cable2_line.set_3d_properties(catenary_points2[:, 2])
        cable2_status = "SLACK"
    else:  # Cable is taut
        cable2_line.set_data([anchor2_pos[0], payload_pos[0]], [anchor2_pos[1], payload_pos[1]])
        cable2_line.set_3d_properties([anchor2_pos[2], payload_pos[2]])
        cable2_status = "TAUT"
    
    # Update text information
    time_text.set_text(f'Time: {t:.2f} s')
    cable1_text.set_text(f'Cable 1: Length={current_length1:.3f}m ({cable1_status}) Tension={tension1:.1f}N')
    cable2_text.set_text(f'Cable 2: Length={current_length2:.3f}m ({cable2_status}) Tension={tension2:.1f}N')
    speed_text.set_text(f'Payload Speed: {speed:.3f} m/s')
    anchor_dist_text.set_text(f'Anchor Distance: {anchor_distance:.3f} m (Max: {max_safe_distance:.3f} m)')
    
    # Update trajectory
    trajectory.set_data(positions[:frame+1, 0], positions[:frame+1, 1])
    trajectory.set_3d_properties(positions[:frame+1, 2])
    
    return (anchor1_point, anchor2_point, payload_point, cable1_line, cable2_line, 
            trajectory, time_text, cable1_text, cable2_text, speed_text, anchor_dist_text)

# Create animation
anim = FuncAnimation(fig, update, frames=len(times), interval=25, blit=True)

# Save as GIF
writer = 'pillow'  # Use pillow writer for GIF
gif_filename = 'loose_cable_dynamics.gif'
anim.save(gif_filename, writer=writer, fps=30, dpi=100)
print(f"Animation saved as {gif_filename}")

plt.legend()
plt.tight_layout()
plt.show()
