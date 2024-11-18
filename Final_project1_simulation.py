from vpython import *
import numpy as np
from scipy.integrate import odeint
from scipy.signal import place_poles

# System parameters
USE_NOISE = False
USE_FRICTION = True
USE_LINEAR_MODEL = True
USE_FILTER = False

# Noise parameters
NOISE_STD_POSITION = 0.01  # Standard deviation for position noise
NOISE_STD_ANGLE = 0.05     # Standard deviation for angle noise
FILTER_WINDOW_SIZE = 5     # Moving average filter window size

# Pendulum Model Class
class PendulumModel:
    def __init__(self, l=0.4, m=0.05, M=1, g=9.81, b=0.0):
        self.l = l
        self.m = m
        self.M = M
        self.g = g
        self.b = b

    def equations(self, state, t, F):
        x, x_dot, theta, theta_dot = state
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        temp = (F + self.m * self.l * theta_dot**2 * sin_theta) / (self.M + self.m)
        theta_ddot = (self.g * sin_theta - cos_theta * temp) / (self.l * (4/3 - (self.m * cos_theta**2) / (self.M + self.m)))
        x_ddot = temp - (self.m * self.l * theta_ddot * cos_theta) / (self.M + self.m)
        return [x_dot, x_ddot, theta_dot, theta_ddot]

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def compute_control(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# Moving Average Filter
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def apply(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return np.mean(self.values)

# State Feedback Controller (Pole Placement)
class StateFeedbackController:
    def __init__(self, A, B, desired_poles, force_limit=100):
        self.A = A
        self.B = B
        self.force_limit = force_limit
        self.update_poles(desired_poles)

    def update_poles(self, poles):
        self.K = place_poles(self.A, self.B, poles).gain_matrix

    def compute_control(self, state):
        control_force = -self.K @ state
        return np.clip(control_force[0], -self.force_limit, self.force_limit)

# Inverted Pendulum System with Controller Selection
class InvertedPendulumSystem:
    def __init__(self, controller_type="PID"):
        self.model = PendulumModel()
        
        # Define State Space Matrices
        A = np.array([[0, 1, 0, 0],
                      [0, -0.1, -self.model.g, 0],
                      [0, 0, 0, 1],
                      [0, 0.1, self.model.g, 0]])
        B = np.array([[0],
                      [1],
                      [0],
                      [-1]])
        initial_poles = np.array([-3, -3.5, -4, -4.5])
        
        # Initialize controllers
        self.pid_controller = PIDController(Kp=-700, Ki=-2500, Kd=-80, dt=0.01)
        self.pole_controller = StateFeedbackController(A, B, initial_poles, force_limit=125)
        
        # Set initial controller type
        self.controller_type = controller_type
        self.controller = self.pid_controller if controller_type == "PID" else self.pole_controller

        # VPython setup with centered, large title
        self.scene = canvas(width=800, height=500, background=color.white, title="<h1 style='text-align:center;'>Hamza's Project 1</h1>")
        self.cart = box(pos=vector(0, -0.2, 0), size=vector(0.4, 0.1, 0.01), color=color.black)
        self.pendulum = cylinder(pos=self.cart.pos, axis=vector(0, -self.model.l, 0), radius=0.02, color=color.green)
        self.pivot = sphere(pos=self.cart.pos + self.pendulum.axis, radius=0.03, color=color.black)
        self.force_arrow = arrow(pos=vector(0, -0.4, 0), axis=vector(0, 0, 0), color=color.red, shaftwidth=0.03)

        # Toggle buttons positioned at the top of the screen
        self.noise_button = button(text="Noise: Off", bind=self.toggle_noise, pos=self.scene.title_anchor)
        self.friction_button = button(text="Friction: On", bind=self.toggle_friction, pos=self.scene.title_anchor)
        self.linear_button = button(text="Linear Model: On", bind=self.toggle_linear_model, pos=self.scene.title_anchor)
        self.filter_button = button(text="Filter: Off", bind=self.toggle_filter, pos=self.scene.title_anchor)
        
        # Labels and text boxes for PID and pole values
        wtext(text="Kp:")
        self.kp_box = winput(bind=self.update_kp, text=str(self.pid_controller.Kp), width=50)
        
        wtext(text=" Ki:")
        self.ki_box = winput(bind=self.update_ki, text=str(self.pid_controller.Ki), width=50)
        
        wtext(text=" Kd:")
        self.kd_box = winput(bind=self.update_kd, text=str(self.pid_controller.Kd), width=50)
        
        wtext(text=" Pole 1:")
        self.pole1_box = winput(bind=self.update_pole1, text=str(initial_poles[0]), width=50)
        
        wtext(text=" Pole 2:")
        self.pole2_box = winput(bind=self.update_pole2, text=str(initial_poles[1]), width=50)
        
        wtext(text=" Pole 3:")
        self.pole3_box = winput(bind=self.update_pole3, text=str(initial_poles[2]), width=50)
        
        wtext(text=" Pole 4:")
        self.pole4_box = winput(bind=self.update_pole4, text=str(initial_poles[3]), width=50)

        # Controller label and dropdown menu for selection
        wtext(text="Controller Type: ")
        self.controller_menu = menu(choices=['PID', 'Pole'], index=0, bind=self.select_controller, text='Controller Type')

        # Real-time data plots below the simulation interface
        self.angle_plot = graph(width=250, height=200, title="Angle vs Time", xtitle="Time", ytitle="Angle (rad)", align="left")
        self.position_plot = graph(width=250, height=200, title="Position vs Time", xtitle="Time", ytitle="Position (m)", align="left")
        self.force_plot = graph(width=250, height=200, title="Control Force vs Time", xtitle="Time", ytitle="Force (N)", align="left")
        self.angle_curve = gcurve(color=color.blue, graph=self.angle_plot)
        self.position_curve = gcurve(color=color.green, graph=self.position_plot)
        self.force_curve = gcurve(color=color.red, graph=self.force_plot)

        # Filters
        self.angle_filter = MovingAverageFilter(FILTER_WINDOW_SIZE)
        self.position_filter = MovingAverageFilter(FILTER_WINDOW_SIZE)

    def add_noise(self, value, std_dev):
        """Add Gaussian noise to a value if noise is enabled."""
        return value + np.random.normal(0, std_dev) if USE_NOISE else value

    def toggle_noise(self):
        global USE_NOISE
        USE_NOISE = not USE_NOISE
        self.noise_button.text = f"Noise: {'On' if USE_NOISE else 'Off'}"

    def toggle_friction(self):
        global USE_FRICTION
        USE_FRICTION = not USE_FRICTION
        self.friction_button.text = f"Friction: {'On' if USE_FRICTION else 'Off'}"

    def toggle_linear_model(self):
        global USE_LINEAR_MODEL
        USE_LINEAR_MODEL = not USE_LINEAR_MODEL
        self.linear_button.text = f"Linear Model: {'On' if USE_LINEAR_MODEL else 'Off'}"

    def toggle_filter(self):
        global USE_FILTER
        USE_FILTER = not USE_FILTER
        self.filter_button.text = f"Filter: {'On' if USE_FILTER else 'Off'}"

    def update_kp(self, box):
        self.pid_controller.Kp = float(box.text)

    def update_ki(self, box):
        self.pid_controller.Ki = float(box.text)

    def update_kd(self, box):
        self.pid_controller.Kd = float(box.text)

    def update_pole1(self, box):
        poles = [float(self.pole1_box.text), float(self.pole2_box.text), float(self.pole3_box.text), float(self.pole4_box.text)]
        self.pole_controller.update_poles(poles)

    def update_pole2(self, box):
        self.update_pole1(box)

    def update_pole3(self, box):
        self.update_pole1(box)

    def update_pole4(self, box):
        self.update_pole1(box)

    def select_controller(self, m):
        self.controller = self.pid_controller if m.selected == 'PID' else self.pole_controller

    def run_simulation(self, initial_state, t):
        state = initial_state
        dt = t[1] - t[0]
        disturbance_force = 0
        
        for time_step in t:
            rate(100)
            if 'left' in keysdown():
                disturbance_force = -20
            elif 'right' in keysdown():
                disturbance_force = 20
            else:
                disturbance_force = 0

            control_force = self.controller.compute_control(state) if isinstance(self.controller, StateFeedbackController) else self.controller.compute_control(0, state[2])
            total_force = control_force + disturbance_force
            state = odeint(self.model.equations, state, [time_step, time_step + dt], args=(total_force,))[-1]

            # Add noise to position and angle
            noisy_position = self.add_noise(state[0], NOISE_STD_POSITION)
            noisy_angle = self.add_noise(state[2], NOISE_STD_ANGLE)

            # Apply filter if enabled
            filtered_position = self.position_filter.apply(noisy_position) if USE_FILTER else noisy_position
            filtered_angle = self.angle_filter.apply(noisy_angle) if USE_FILTER else noisy_angle

            # Update cart and pendulum positions
            self.cart.pos.x = filtered_position
            self.pendulum.axis = vector(self.model.l * np.sin(filtered_angle), self.model.l * np.cos(filtered_angle), 0)
            self.pendulum.pos = self.cart.pos
            self.pivot.pos = self.cart.pos + self.pendulum.axis

            # Display force
            self.force_arrow.axis = vector(disturbance_force * 0.02, 0, 0)

            # Update plots
            self.angle_curve.plot(time_step, filtered_angle)
            self.position_curve.plot(time_step, filtered_position)
            self.force_curve.plot(time_step, control_force)

# Run the Simulation
t = np.linspace(0, 60, 12000)
initial_state = [0, 0, 0.1, 0]
system = InvertedPendulumSystem(controller_type="PID")
system.run_simulation(initial_state, t)
