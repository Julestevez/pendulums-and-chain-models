import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import solve_ivp
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter

class DrivenDampedPendulum3D:
    """
    Simulates a 3D driven damped pendulum with a moving pivot and wind effects.
    """

    def __init__(self, length=1.0, mass=1.0, damping=0.1, gravity=9.81,
                 driving_amplitude=1.0, driving_frequency=1.0,
                 pivot_motion_amplitude=np.array([0.5, 0.5, 0.5]),  # Motion amplitude in x, y, z
                 pivot_motion_frequency=np.array([0.5, 0.7, 0.9]),  # Motion frequency in x, y, z
                 initial_angles=np.array([np.pi/2, 0.0, 0.0]),  # initial angles (theta, phi, psi)
                 initial_angular_velocities=np.array([0.0, 0.0, 0.0]), # Initial angular velocities
                 wind_strength=0.1, # Wind strength in the y-axis
                 wind_frequency=0.5):  # Wind frequency (optional - makes wind more interesting)

        """
        Initializes the pendulum parameters.

        Args:
            length (float): Length of the pendulum arm.
            mass (float): Mass of the pendulum bob.
            damping (float): Damping coefficient.
            gravity (float): Acceleration due to gravity.
            driving_amplitude (float): Amplitude of the driving force.
            driving_frequency (float): Frequency of the driving force.
            pivot_motion_amplitude (np.ndarray): Amplitude of the pivot's motion in x, y, and z.
            pivot_motion_frequency (np.ndarray): Frequency of the pivot's motion in x, y, and z.
            initial_angles (np.ndarray): Initial angles (theta, phi, psi).
            initial_angular_velocities (np.ndarray): Initial angular velocities.
            wind_strength (float): Strength of the wind force in the y-axis.
            wind_frequency (float): Frequency of the oscillating wind force (set to 0 for constant wind).
        """
        self.length = length
        self.mass = mass
        self.damping = damping
        self.gravity = gravity
        self.driving_amplitude = driving_amplitude
        self.driving_frequency = driving_frequency
        self.pivot_motion_amplitude = pivot_motion_amplitude
        self.pivot_motion_frequency = pivot_motion_frequency
        self.initial_angles = initial_angles
        self.initial_angular_velocities = initial_angular_velocities
        self.wind_strength = wind_strength
        self.wind_frequency = wind_frequency


    def pivot_position(self, t):
        """
        Calculates the pivot position at a given time.

        Args:
            t (float): Time.

        Returns:
            np.ndarray: Pivot position (x, y, z).
        """
        x = self.pivot_motion_amplitude[0] * np.sin(self.pivot_motion_frequency[0] * t)
        y = self.pivot_motion_amplitude[1] * np.cos(self.pivot_motion_frequency[1] * t)
        z = self.pivot_motion_amplitude[2] * np.sin(self.pivot_motion_frequency[2] * t)
        return np.array([x, y, z])

    def wind_force(self, t):
        """
        Calculates the wind force at a given time.  The wind force acts in the y-axis.

        Args:
            t (float): Time.

        Returns:
            float: Wind force in the y-axis.
        """
        if self.wind_frequency == 0:
            return self.wind_strength # Constant wind
        else:
            return self.wind_strength * np.sin(self.wind_frequency * t) # Oscillating wind

    def equations_of_motion(self, t, state):
        """
        Defines the equations of motion for the 3D pendulum with wind.

        Args:
            t (float): Time.
            state (np.ndarray): Current state [theta, phi, psi, dtheta/dt, dphi/dt, dpsi/dt].

        Returns:
            np.ndarray: Derivatives of the state variables [dtheta/dt, dphi/dt, dpsi/dt, d2theta/dt2, d2phi/dt2, d2psi/dt2].
        """
        theta, phi, psi, theta_dot, phi_dot, psi_dot = state

        # Extract pivot position and velocity
        pivot_pos = self.pivot_position(t)
        pivot_vel = np.array([
            self.pivot_motion_amplitude[0] * self.pivot_motion_frequency[0] * np.cos(self.pivot_motion_frequency[0] * t),
            -self.pivot_motion_amplitude[1] * self.pivot_motion_frequency[1] * np.sin(self.pivot_motion_frequency[1] * t),
            self.pivot_motion_amplitude[2] * self.pivot_motion_frequency[2] * np.cos(self.pivot_motion_frequency[2] * t)
        ])

        # Calculate the position of the pendulum bob
        bob_pos = pivot_pos + self.length * np.array([
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            -np.cos(theta)
        ])

        # Calculate the wind force
        wind_force_y = self.wind_force(t)

        # Calculate angular accelerations (more complex in 3D, using Euler angles)
        # This is a simplified example and may need adjustment depending on the desired behavior
        # in a full 3D rotational dynamics simulation.  This approximation attempts to
        # treat each angle somewhat independently while including some coupling through trigonometric terms.
        theta_ddot = (-self.gravity / self.length * np.sin(theta)
                      - self.damping * theta_dot + self.driving_amplitude * np.cos(self.driving_frequency * t) / self.length
                      + (pivot_vel[1] * np.cos(phi) - pivot_vel[0] * np.sin(phi)) / self.length)  # Projection of pivot acceleration

        phi_ddot = (-self.damping * phi_dot + (pivot_vel[0] * np.cos(phi) + pivot_vel[1] * np.sin(phi)) / (self.length * np.sin(theta))
                    + wind_force_y / (self.mass * self.length * np.sin(theta) * np.cos(phi))) # Wind influence
        psi_ddot = -self.damping * psi_dot


        return [theta_dot, phi_dot, psi_dot, theta_ddot, phi_ddot, psi_ddot]

    def simulate(self, t_span=(0, 20), t_eval=None, max_step=0.01):
        """
        Simulates the pendulum motion using scipy.integrate.solve_ivp.

        Args:
            t_span (tuple): Time span for the simulation (start, end).
            t_eval (np.ndarray, optional): Times at which to store the computed solution.
            max_step (float):  Maximum allowed step size.

        Returns:
            scipy.integrate._ivp.ivp_solution: Solution object containing the simulation results.
        """

        initial_state = np.concatenate((self.initial_angles, self.initial_angular_velocities))

        sol = solve_ivp(self.equations_of_motion, t_span, initial_state,
                        dense_output=True, t_eval=t_eval, max_step=max_step)
        return sol

    def get_bob_positions(self, sol):
         """
         Calculates the bob positions from the simulation results.

         Args:
             sol (scipy.integrate._ivp.ivp_solution): Solution object from the simulation.

         Returns:
             np.ndarray: Array of bob positions (x, y, z) at each time step.
         """
         bob_positions = []
         for i in range(len(sol.t)):
             theta, phi, psi = sol.y[0, i], sol.y[1, i], sol.y[2, i]
             pivot_pos = self.pivot_position(sol.t[i])
             bob_pos = pivot_pos + self.length * np.array([
                 np.sin(theta) * np.cos(phi),
                 np.sin(theta) * np.sin(phi),
                 -np.cos(theta)
             ])
             bob_positions.append(bob_pos)
         return np.array(bob_positions)


    def animate(self, sol, interval=20, show_trace=True, trace_length=50, trace_color='gray', save_gif=False, gif_filename='pendulum_animation.gif', fps=15):
        """
        Animates the pendulum motion in 3D.

        Args:
            sol (scipy.integrate._ivp.ivp_solution): Solution object from the simulation.
            interval (int): Time between frames in milliseconds.
            show_trace (bool): Whether to show the pendulum's trace.
            trace_length (int): Length of the trace (number of points to keep).
            trace_color (str): Color of the trace.
            save_gif (bool): Whether to save the animation as a GIF.
            gif_filename (str): Name of the GIF file to save.
            fps (int): Frames per second for the GIF.
        """

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Calculate the bob positions
        bob_positions = self.get_bob_positions(sol)
        pivot_positions = np.array([self.pivot_position(t) for t in sol.t])

        # Set up the plot limits (based on the maximum extent of the pivot and bob positions)
        max_extent = np.max(np.abs(np.concatenate([pivot_positions.flatten(), bob_positions.flatten()])))
        ax_range = [-max_extent - self.length * 0.1, max_extent + self.length * 0.1]  # Add a bit of padding
        ax.set_xlim(ax_range)
        ax.set_ylim(ax_range)
        ax.set_zlim(ax_range)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("3D Driven Damped Pendulum with Wind")

        # Initialize the pendulum arm and bob
        line, = ax.plot([pivot_positions[0, 0], bob_positions[0, 0]],
                       [pivot_positions[0, 1], bob_positions[0, 1]],
                       [pivot_positions[0, 2], bob_positions[0, 2]], lw=2, color='blue')
        bob, = ax.plot([bob_positions[0, 0]], [bob_positions[0, 1]], [bob_positions[0, 2]],
                        marker='o', markersize=8, color='red')

        # Initialize the pivot point
        pivot, = ax.plot([pivot_positions[0, 0]], [pivot_positions[0, 1]], [pivot_positions[0, 2]],
                            marker='x', markersize=8, color='green')

        # Initialize the trace
        if show_trace:
            trace, = ax.plot([], [], [], lw=1, color=trace_color, alpha=0.5)
            trace_x, trace_y, trace_z = [], [], []  # Keep track of trace data


        def update(frame):
            # Update pendulum arm and bob positions
            line.set_data([pivot_positions[frame, 0], bob_positions[frame, 0]],
                          [pivot_positions[frame, 1], bob_positions[frame, 1]])
            line.set_3d_properties([pivot_positions[frame, 2], bob_positions[frame, 2]])
            bob.set_data([bob_positions[frame, 0]], [bob_positions[frame, 1]])
            bob.set_3d_properties([bob_positions[frame, 2]])

            #Update pivot position
            pivot.set_data([pivot_positions[frame, 0]], [pivot_positions[frame, 1]])
            pivot.set_3d_properties([pivot_positions[frame, 2]])

            # Update the trace
            if show_trace:
                trace_x.append(bob_positions[frame, 0])
                trace_y.append(bob_positions[frame, 1])
                trace_z.append(bob_positions[frame, 2])

                # Keep the trace length limited
                trace_x_trimmed = trace_x[-trace_length:]
                trace_y_trimmed = trace_y[-trace_length:]
                trace_z_trimmed = trace_z[-trace_length:]


                trace.set_data(trace_x_trimmed, trace_y_trimmed)
                trace.set_3d_properties(trace_z_trimmed)

            return line, bob, pivot, trace if show_trace else line, bob, pivot

        ani = animation.FuncAnimation(fig, update, frames=len(sol.t), interval=interval, blit=False)  #blit=True can cause problems in 3d, especially with trace.
        
        # Save animation as GIF if requested
        if save_gif:
            print(f"Saving animation to {gif_filename}...")
            writer = PillowWriter(fps=fps)
            ani.save(gif_filename, writer=writer)
            print(f"Animation saved as {gif_filename}")
        
        plt.show()

# Example usage:
if __name__ == "__main__":
    # Define pendulum parameters
    length = 1.0
    mass = 1.0
    damping = 0.2
    gravity = 9.81
    driving_amplitude = 0.5
    driving_frequency = 2.0
    pivot_motion_amplitude = np.array([0.2, 0.3, 0.1])  # Amplitude of pivot motion in x, y, z
    pivot_motion_frequency = np.array([1.0, 1.2, 0.8])  # Frequency of pivot motion in x, y, z
    initial_angles = np.array([np.pi/2, 0.1, 0.0])  # Initial angles (theta, phi, psi)
    initial_angular_velocities = np.array([0.0, 0.0, 0.0]) # Initial angular velocities
    wind_strength = 0.1  # Strength of the wind force
    wind_frequency = 0.5 # Frequency of the wind force

    # Create a pendulum object
    pendulum = DrivenDampedPendulum3D(length=length, mass=mass, damping=damping,
                                        gravity=gravity, driving_amplitude=driving_amplitude,
                                        driving_frequency=driving_frequency,
                                        pivot_motion_amplitude=pivot_motion_amplitude,
                                        pivot_motion_frequency=pivot_motion_frequency,
                                        initial_angles=initial_angles,
                                        initial_angular_velocities=initial_angular_velocities,
                                        wind_strength=wind_strength,
                                        wind_frequency=wind_frequency)

    # Simulate the pendulum motion
    t_eval = np.linspace(0, 20, 500) # Evaluate the solution at specific time points for smoother animation
    solution = pendulum.simulate(t_span=(0, 20), t_eval=t_eval, max_step=0.005) #Reduce max_step for better accuracy


    # Animate the pendulum and save as GIF
    pendulum.animate(solution, interval=20, show_trace=True, trace_length=100, trace_color='orange',
                    save_gif=True, gif_filename='pendulum_3d.gif', fps=20)
