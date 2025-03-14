import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from scipy.integrate import solve_ivp

# Constants
g = 9.81  # gravitational acceleration (m/s²)
m = 1.5   # mass of the payload (kg) - increased for more pronounced inertia
d = 0.3   # damping coefficient (kg/s) - reduced to allow more oscillation
L1 = 1.5  # length of cable 1 (m) - FIXED
L2 = 1.5  # length of cable 2 (m) - FIXED

# Fixed anchor position (x, y, z)
anchor1 = np.array([1.0, 1.0, 2.0])

# Calculate maximum allowed distance between anchors
# For two cables of length L1 and L2, the maximum distance between anchors is L1 + L2
max_anchor_distance = L1 + L2
# For stability, we'll keep it a bit less than the theoretical maximum
safety_factor = 0.9
max_safe_distance = max_anchor_distance * safety_factor

# Function to get the moving anchor position at time t with distance constraint
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
    
    # Check distance from anchor1
    vec_to_anchor1 = unconstrained_pos - anchor1
    distance = np.linalg.norm(vec_to_anchor1)
    
    # If too far, scale the position vector to maintain maximum safe distance
    if distance > max_safe_distance:
        # Direction vector from anchor1 to anchor2
        direction = vec_to_anchor1 / distance
        # Scale to max safe distance
        constrained_pos = anchor1 + direction * max_safe_distance
        return constrained_pos
    else:
        return unconstrained_pos

# Calculate position that satisfies both cable length constraints
def find_constrained_position(anchor1, anchor2, prev_pos, prev_vel, dt):
    # Function to minimize: distance from predicted position
    def objective(pos):
        return np.sum((pos - (prev_pos + prev_vel * dt))**2)
    
    # Constraint: cable 1 length must be L1
    def constraint1(pos):
        return np.linalg.norm(pos - anchor1) - L1
    
    # Constraint: cable 2 length must be L2
    def constraint2(pos):
        return np.linalg.norm(pos - anchor2) - L2
    
    # Set up the constraints
    constraints = [
        {'type': 'eq', 'fun': constraint1},
        {'type': 'eq', 'fun': constraint2}
    ]
    
    # Initial guess is the previous position
    initial_guess = prev_pos
    
    # Solve the constrained optimization problem
    result = minimize(objective, initial_guess, method='SLSQP', 
                      constraints=constraints, tol=1e-10)
    
    # Check if optimization was successful
    if not result.success:
        # Fallback: use a different initial guess
        midpoint = (anchor1 + anchor2) / 2
        lower_point = np.array([midpoint[0], midpoint[1], midpoint[2] - min(L1, L2) * 0.8])
        result = minimize(objective, lower_point, method='SLSQP', 
                          constraints=constraints, tol=1e-10)
    
    return result.x

# Function to compute acceleration for unconstrained motion
def compute_acceleration(pos, vel, t):
    # Get current anchor2 position
    anchor2 = get_anchor2(t)
    
    # Calculate cable vectors (from payload to anchors)
    r1 = anchor1 - pos
    r2 = anchor2 - pos
    
    # Calculate current cable lengths
    l1 = np.linalg.norm(r1)
    l2 = np.linalg.norm(r2)
    
    # Unit vectors along cables
    u1 = r1 / l1
    u2 = r2 / l2
    
    # Gravity force
    F_gravity = np.array([0, 0, -m * g])
    
    # Damping force
    F_damping = -d * vel
    
    # For initial acceleration calculation, use strong spring forces to approximate constraints
    k_temp = 1000.0  # Temporary high spring constant
    T1 = k_temp * (l1 - L1)
    T2 = k_temp * (l2 - L2)
    F_tension = T1 * u1 + T2 * u2
    
    # Total force
    F_total = F_gravity + F_damping + F_tension
    
    # Return acceleration
    return F_total / m

# Find initial equilibrium position that satisfies cable constraints
def find_initial_position():
    # Initial guess halfway between anchors but lower
    initial_anchor2 = get_anchor2(0)
    midpoint = (anchor1 + initial_anchor2) / 2
    initial_guess = np.array([midpoint[0], midpoint[1], midpoint[2] - 0.5])
    
    # Function to minimize: sum of the differences from the target cable lengths
    def objective(pos):
        l1 = np.linalg.norm(pos - anchor1)
        l2 = np.linalg.norm(pos - initial_anchor2)
        return (l1 - L1)**2 + (l2 - L2)**2
    
    # Solve for position
    result = minimize(objective, initial_guess, method='L-BFGS-B', tol=1e-10)
    
    return result.x

# Time settings
t_max = 15.0  # seconds - shortened for more concentrated action
dt = 0.03     # smaller timestep for smoother animation with faster motion
num_steps = int(t_max / dt)
times = np.linspace(0, t_max, num_steps)

# Initialize arrays to store simulation results
positions = np.zeros((num_steps, 3))
velocities = np.zeros((num_steps, 3))
anchor2_positions = np.zeros((num_steps, 3))
anchor_distances = np.zeros(num_steps)

# Find initial position and set zero initial velocity
positions[0] = find_initial_position()
velocities[0] = np.zeros(3)
anchor2_positions[0] = get_anchor2(times[0])
anchor_distances[0] = np.linalg.norm(anchor1 - anchor2_positions[0])

# Custom integrator that enforces constraints at each step
for i in range(1, num_steps):
    t = times[i]
    t_prev = times[i-1]
    
    # Current state
    pos = positions[i-1]
    vel = velocities[i-1]
    
    # Get current anchor2 position
    anchor2 = get_anchor2(t)
    anchor2_positions[i] = anchor2
    anchor_distances[i] = np.linalg.norm(anchor1 - anchor2)
    
    # Calculate acceleration (without constraints)
    acc = compute_acceleration(pos, vel, t)
    
    # Predict new position and velocity using semi-implicit Euler
    vel_pred = vel + acc * dt
    pos_pred = pos + vel_pred * dt
    
    # Apply constraints to find actual position
    new_pos = find_constrained_position(anchor1, anchor2, pos, vel_pred, dt)
    
    # Calculate new velocity considering the constraint correction
    new_vel = (new_pos - pos) / dt
    
    # Store the results
    positions[i] = new_pos
    velocities[i] = new_vel

# Create figure for animation
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Set up static elements
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Dynamic Simulation of Payload with Inelastic Cables')

# Initialize plot elements
anchor1_point, = ax.plot([], [], [], 'ro', markersize=10, label='Anchor 1')
anchor2_point, = ax.plot([], [], [], 'bo', markersize=10, label='Anchor 2')
payload_point, = ax.plot([], [], [], 'go', markersize=12, label='Payload')
cable1_line, = ax.plot([], [], [], 'r-', linewidth=2, label='Cable 1')
cable2_line, = ax.plot([], [], [], 'b-', linewidth=2, label='Cable 2')
trajectory, = ax.plot([], [], [], 'g--', linewidth=1, alpha=0.5, label='Trajectory')

# Text annotations for information
time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)
cable1_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes)
cable2_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes)
speed_text = ax.text2D(0.02, 0.80, '', transform=ax.transAxes)
anchor_dist_text = ax.text2D(0.02, 0.75, '', transform=ax.transAxes)

# Set limits for better visualization
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-2.5, 2.5])
ax.set_zlim([0, 3])

# Animation update function
def update(frame):
    t = times[frame]
    payload_pos = positions[frame]
    payload_vel = velocities[frame]
    
    # Get the current anchor2 position
    anchor2_pos = anchor2_positions[frame]
    
    # Calculate cable lengths (for verification)
    cable1_length = np.linalg.norm(anchor1 - payload_pos)
    cable2_length = np.linalg.norm(anchor2_pos - payload_pos)
    
    # Calculate payload speed
    speed = np.linalg.norm(payload_vel)
    
    # Calculate distance between anchors
    anchor_distance = np.linalg.norm(anchor1 - anchor2_pos)
    
    # Update anchor positions
    anchor1_point.set_data([anchor1[0]], [anchor1[1]])
    anchor1_point.set_3d_properties([anchor1[2]])
    
    anchor2_point.set_data([anchor2_pos[0]], [anchor2_pos[1]])
    anchor2_point.set_3d_properties([anchor2_pos[2]])
    
    # Update payload position
    payload_point.set_data([payload_pos[0]], [payload_pos[1]])
    payload_point.set_3d_properties([payload_pos[2]])
    
    # Update cable positions
    cable1_line.set_data([anchor1[0], payload_pos[0]], [anchor1[1], payload_pos[1]])
    cable1_line.set_3d_properties([anchor1[2], payload_pos[2]])
    
    cable2_line.set_data([anchor2_pos[0], payload_pos[0]], [anchor2_pos[1], payload_pos[1]])
    cable2_line.set_3d_properties([anchor2_pos[2], payload_pos[2]])
    
    # Update text information
    time_text.set_text(f'Time: {t:.2f} s')
    cable1_text.set_text(f'Cable 1 Length: {cable1_length:.6f} m (Target: {L1} m)')
    cable2_text.set_text(f'Cable 2 Length: {cable2_length:.6f} m (Target: {L2} m)')
    speed_text.set_text(f'Payload Speed: {speed:.3f} m/s')
    anchor_dist_text.set_text(f'Anchor Distance: {anchor_distance:.3f} m (Max: {max_safe_distance:.3f} m)')
    
    # Update trajectory
    trajectory.set_data(positions[:frame+1, 0], positions[:frame+1, 1])
    trajectory.set_3d_properties(positions[:frame+1, 2])
    
    return (anchor1_point, anchor2_point, payload_point, cable1_line, cable2_line, 
            trajectory, time_text, cable1_text, cable2_text, speed_text, anchor_dist_text)

# Create animation
anim = FuncAnimation(fig, update, frames=len(times), interval=40, blit=True)

plt.legend()
plt.tight_layout()

# Save the animation as a GIF file
# Note: PillowWriter might be slower but more widely available than imagemagick
from matplotlib.animation import PillowWriter
writer = PillowWriter(fps=20)
anim.save('payload_dynamics.gif', writer=writer)

print("Animation saved as 'payload_dynamics.gif'")

# Display the animation (optional)
plt.show()
