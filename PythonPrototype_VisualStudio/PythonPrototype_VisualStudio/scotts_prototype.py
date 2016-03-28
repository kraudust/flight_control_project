from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

class DoublePendulum:
    """Double Pendulum Class

    init_state is [theta1, omega1, theta2, omega2] in degrees,
    where theta1, omega1 is the angular position and velocity of the first
    pendulum arm, and theta2, omega2 is that of the second pendulum arm
    """
    def __init__(self,
                 init_state = [120, 0, -20, 0],
                 L1=1.0,  # length of pendulum 1 in m
                 L2=1.0,  # length of pendulum 2 in m
                 M1=1.0,  # mass of pendulum 1 in kg
                 M2=1.0,  # mass of pendulum 2 in kg
                 G=9.8,  # acceleration due to gravity, in m/s^2
                 origin=(0, 0)): 
        self.init_state = np.asarray(init_state, dtype='float')
        self.params = (L1, L2, M1, M2, G)
        self.origin = origin
        self.time_elapsed = 0

        self.state = self.init_state * np.pi / 180.
    
    def position(self):
        """compute the current x,y positions of the pendulum arms"""
        (L1, L2, M1, M2, G) = self.params

        x = np.cumsum([self.origin[0],
                       L1 * sin(self.state[0]),
                       L2 * sin(self.state[2])])
        y = np.cumsum([self.origin[1],
                       -L1 * cos(self.state[0]),
                       -L2 * cos(self.state[2])])
        return (x, y)

    def energy(self):
        """compute the energy of the current state"""
        (L1, L2, M1, M2, G) = self.params

        x = np.cumsum([L1 * sin(self.state[0]),
                       L2 * sin(self.state[2])])
        y = np.cumsum([-L1 * cos(self.state[0]),
                       -L2 * cos(self.state[2])])
        vx = np.cumsum([L1 * self.state[1] * cos(self.state[0]),
                        L2 * self.state[3] * cos(self.state[2])])
        vy = np.cumsum([L1 * self.state[1] * sin(self.state[0]),
                        L2 * self.state[3] * sin(self.state[2])])

        U = G * (M1 * y[0] + M2 * y[1])
        K = 0.5 * (M1 * np.dot(vx, vx) + M2 * np.dot(vy, vy))

        return U + K

    def dstate_dt(self, state, t):
        """compute the derivative of the given state"""
        (M1, M2, L1, L2, G) = self.params

        dydx = np.zeros_like(state)
        dydx[0] = state[1]
        dydx[2] = state[3]

        cos_delta = cos(state[2] - state[0])
        sin_delta = sin(state[2] - state[0])
        dry_friction = 0.5;

        friction = 0.2

        den1 = (M1 + M2) * L1 - M2 * L1 * cos_delta * cos_delta
        dydx[1] = (M2 * L1 * state[1] * state[1] * sin_delta * cos_delta
                   + M2 * G * sin(state[2]) * cos_delta
                   + M2 * L2 * state[3] * state[3] * sin_delta
                   - ((M1 + M2) * G * sin(state[0])) / den1) - (dry_friction * state[1])

        den2 = (L2 / L1) * den1
        dydx[3] = (-M2 * L2 * state[3] * state[3] * sin_delta * cos_delta
                   + (M1 + M2) * G * sin(state[0]) * cos_delta
                   - (M1 + M2) * L1 * state[1] * state[1] * sin_delta
                   - ((M1 + M2) * G * sin(state[2])) / den2) - (dry_friction * state[3])
        
        return dydx

    def step(self, dt):
        """execute one time step of length dt and update state"""
        self.state = integrate.odeint(self.dstate_dt, self.state, [0, dt])[1]
        self.time_elapsed += dt

#------------------------------------------------------------
# set up initial state and global variables
pendulum = DoublePendulum([180., 0.0, -20., 0.0])
dt = 1./30 # 30 fps

#------------------------------------------------------------
# set up figure and animation for pendulum
fig_pendulum = plt.figure()
ax_pendulum = fig_pendulum.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-2, 2), ylim=(-2, 2))
ax_pendulum.grid()

line, = ax_pendulum.plot([], [], 'o-', lw=2)
time_text = ax_pendulum.text(0.02, 0.95, '', transform=ax_pendulum.transAxes)
energy_text = ax_pendulum.text(0.02, 0.90, '', transform=ax_pendulum.transAxes)

def init_pendulum():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')
    energy_text.set_text('')
    return line, time_text, energy_text

def animate_pendulum(i):
    """perform animation step"""
    global pendulum, dt
    pendulum.step(dt)
    
    line.set_data(*pendulum.position())
    time_text.set_text('time = %.1f' % pendulum.time_elapsed)
    energy_text.set_text('energy = %.3f J' % pendulum.energy())
    return line, time_text, energy_text

#------------------------------------------------------------
# set up figure and animation for plotting of pendulum states
fig_plots = plt.figure()

ax_theta1 = fig_plots.add_subplot(411, xlim=(0, 4), ylim=(-7, 7))
ax_omega1 = fig_plots.add_subplot(412, sharex=ax_theta1, ylim=(-7, 7))
ax_theta2 = fig_plots.add_subplot(413, sharex=ax_theta1, ylim=(-7, 7))
ax_omega2 = fig_plots.add_subplot(414, sharex=ax_theta1, ylim=(-7, 7))
ax_theta1.set_xlim(0, 4)

ax_theta1.grid()
ax_omega1.grid()
ax_theta2.grid()
ax_omega2.grid()

line_theta1, = ax_theta1.plot([], [])
line_omega1, = ax_omega1.plot([], [])
line_theta2, = ax_theta2.plot([], [])
line_omega2, = ax_omega2.plot([], [])

ax_theta1.set_xlabel('time (s)')
ax_theta1.set_ylabel('theta1')

ax_omega1.set_xlabel('time (s)')
ax_omega1.set_ylabel('omega1')

ax_theta2.set_xlabel('time (s)')
ax_theta2.set_ylabel('theta2')

ax_omega2.set_xlabel('time (s)')
ax_omega2.set_ylabel('omega2')

theta1_data = np.array([])
omega1_data = np.array([])
theta2_data = np.array([])
omega2_data = np.array([])
plot_time   = np.array([])
# the following variables keep track of our axis limits so we can scale them when needed
axis_xlim   = 4.0
theta1_max  = 7.0
theta2_max  = 7.0
omega1_max  = 7.0
omega2_max  = 7.0

def init_plot():
    """initialize animation"""
    line_theta1.set_data([], [])
    line_omega1.set_data([], [])
    line_theta2.set_data([], [])
    line_omega2.set_data([], [])

    return line_theta1, line_omega1, line_theta2, line_omega2

def animate_plot(i):
    """perform animation step"""
    global pendulum, dt, theta1_data, theta2_data, omega1_data, omega2_data, plot_time, ax_theta1, axis_xlim, fig_plots
    
    # the append function doesn't append to the array given by reference, so we have to pass it by value and simultaneously assign it to the original
    theta1_data = np.append(theta1_data, pendulum.state[0])
    omega1_data = np.append(omega1_data, pendulum.state[1])
    theta2_data = np.append(theta2_data, pendulum.state[2])
    omega2_data = np.append(omega2_data, pendulum.state[3])
    plot_time   = np.append(plot_time, pendulum.time_elapsed)
    
    # update the time axis when necessary... they are all linked to the same pointer so you only need to update theta1
    if(pendulum.time_elapsed > axis_xlim - 1):
        axis_xlim += 2.0 # this number moves the x axis of all plots by a given amount
        ax_theta1.set_xlim(0, axis_xlim)
        fig_plots.show()

    # update the y-axis of each plot by a certain amount if the max or min is going off the plot

    line_theta1.set_data(plot_time, theta1_data)
    line_omega1.set_data(plot_time, omega1_data)
    line_theta2.set_data(plot_time, theta2_data)
    line_omega2.set_data(plot_time, omega2_data)
    return line_theta1, line_omega1, line_theta2, line_omega2#, ax_theta1

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate_pendulum(0)
animate_plot(0)
t1 = time()
interval = 1000 * dt - (t1 - t0)

ani_pendulum = animation.FuncAnimation(fig_pendulum, animate_pendulum, frames=300,
                              interval=interval, blit=True, init_func=init_pendulum)
ani_plot = animation.FuncAnimation(fig_plots, animate_plot, frames=300,
                              interval=interval, blit=True, init_func=init_plot)

plt.show()