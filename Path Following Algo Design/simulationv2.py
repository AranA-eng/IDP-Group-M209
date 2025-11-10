import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# System constants
# -----------------------------
r = 0.025       # wheel radius [m]
L = 0.15        # wheelbase [m]
v = 0.1         # forward speed [m/s]
T = 30          # simulation time [s]
thickness = 0.02
line_x = 0.0

sensor_offsets = [-0.02, -0.005, 0.005, 0.02]
d = 0.05        # forward offset of sensors

Kp, Ki, Kd = 6.0, 0.0, 0.1
dt_pid = 0.01
alpha = 0.9

# -----------------------------
# Sensor model
# -----------------------------
def sensor_readings(x, y, theta, sensor_offsets, d=d, line_x=0, t_line=thickness):
    readings = []
    for s in sensor_offsets:
        x_i = x + d * np.sin(theta) + s * np.cos(theta)
        y_i = y + d * np.cos(theta) + s * np.sin(theta)
        readings.append(1 if abs(x_i - line_x) <= t_line/2 else 0)
    return np.array(readings)

def weighted_sum(readings, sensor_offsets, prev_error=0.0):
    active = sum(readings)
    if active == 0:
        return prev_error
    error = sum(s*r_i for s, r_i in zip(sensor_offsets, readings))
    return error / active

# -----------------------------
# PID controller
# -----------------------------
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.prev_error = 0.0
        self.I = 0.0
        self.P = 0.0
        self.D = 0.0
        self.u = 0.0

    def update(self, error):
        self.P = self.Kp * error
        self.I += self.Ki * error * self.dt
        self.D = self.Kd * (error - self.prev_error) / self.dt
        self.u = self.P + self.I + self.D
        self.prev_error = error
        return self.u

pid = PIDController(Kp, Ki, Kd, dt_pid)

# -----------------------------
# Simulation setup
# -----------------------------
dt = 0.001
x0, y0, theta0 = 0.016, 0.0, 0.0
state = np.array([x0, y0, theta0])
omega1 = omega2 = v/r

time_hist, x_hist, y_hist, theta_hist = [], [], [], []
U_hist, P_hist, I_hist, D_hist = [], [], [], []
error_hist, pred_hist = [], []

predicted_error = 0.0
t = 0.0

# -----------------------------
# Simulation loop
# -----------------------------
while t < T:
    readings = sensor_readings(*state, sensor_offsets)
    meas = weighted_sum(readings, sensor_offsets, predicted_error)

    v_forward = r * (omega1 + omega2) / 2.0
    predicted_error += v_forward * np.sin(state[2]) * dt
    predicted_error = alpha * meas + (1-alpha) * predicted_error

    # Update PID every dt_pid
    if (len(time_hist) == 0) or (t - time_hist[-1] >= dt_pid):
        u = pid.update(predicted_error)
        max_wheel = 30.0
        omega1 = np.clip((v + (L/2)*u)/r, -max_wheel, max_wheel)
        omega2 = np.clip((v - (L/2)*u)/r, -max_wheel, max_wheel)

    theta_dot = r * (omega1 - omega2) / L
    dx = v_forward * np.sin(state[2])
    dy = v_forward * np.cos(state[2])
    dtheta = theta_dot

    state += dt * np.array([dx, dy, dtheta])

    # Store histories
    time_hist.append(t)
    x_hist.append(state[0])
    y_hist.append(state[1])
    theta_hist.append(state[2])

    error_hist.append(predicted_error)
    pred_hist.append(predicted_error)
    U_hist.append(pid.u)
    P_hist.append(pid.P)
    I_hist.append(pid.I)
    D_hist.append(pid.D)

    t += dt

# -----------------------------
# Plot results
# -----------------------------
plt.figure(figsize=(12,4))
plt.subplot(1,2,1)
plt.plot(x_hist, y_hist, label='Robot path')
plt.axvline(x=0, color='k', linestyle='--', label='Line')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)

plt.subplot(1,2,2)
plt.plot(time_hist, error_hist, label='Weighted error')
plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.title('PID Error Signal')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10,4))
plt.plot(time_hist, U_hist, label='u')
plt.plot(time_hist, P_hist, label='P')
plt.plot(time_hist, I_hist, label='I')
plt.plot(time_hist, D_hist, label='D')
plt.xlabel('Time [s]')
plt.ylabel('PID terms')
plt.title('PID Components')
plt.legend()
plt.grid(True)
plt.show()
