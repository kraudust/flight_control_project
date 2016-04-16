from numpy import sin, cos
from time import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
import control

class PendulumOnCart:
    """Pendulum On Cart Class
        
        init_state is [z, theta, zdot, thetadot] in radians and meters
        where z is the position of the cart and theta is the angle of the pendulum from vertical
    """
    def __init__(   self, 
                    init_state = [0, 0, 0, 0],
                    M1 = 0.05, # mass of pendulum in kg
                    M2 = 0.25, # mass of cart in kg
                    L  = 0.50, # length of pendulum in meters
                    b  = 0.05, # friction of connection N m
                    g  = 9.8,  # accel of gravity in m/s^2
                    origin = ( 0, 0)):
        self.init_state = np.asarray(init_state, dtype='float')
        self.params = (M1, M2, L, b, g)
        self.origin = origin
        self.time_elapsed = 0
        self.state = np.append(self.init_state, 0) # we'll add the force onto the end
    
    def position(self):
        """compute the current position of the pendulum and cart"""
        (M1, M2, L, b, g) = self.params

        x = np.array([ self.state[0],
                        (L/4)+self.state[0],
                        (L/4)+self.state[0],
                        (-L/4)+self.state[0],
                        (-L/4)+self.state[0],
                        self.state[0],
                        L*sin(self.state[1]) + self.state[0]])

        y = np.array([  0,
                        0,
                       -L/4,
                       -L/4,
                        0,
                        0,
                        L * cos(self.state[1])])
        
        return(x, y)

    def dstate_dt(self, state, t):
        """compute the derivatives of the given state"""
        (M1, M2, L, b, g) = self.params
        [z, theta, zdot, thetadot, F] = state

        M = np.matrix([[M1+M2, M1*L*cos(theta)], [ M1*L*cos(theta), M1*(L**2)]])
        
        c = np.matrix([[M1*L*(thetadot**2)*sin(theta) + F - b*zdot],
                          [M1*g*L*sin(theta)]])

        tmp = np.linalg.inv(M)*c
        zddot     = tmp[0];
        thetaddot = tmp[1]; 
        
        dydx = np.zeros_like(state)

        dydx[0] = zdot
        dydx[1] = thetadot
        dydx[2] = zddot
        dydx[3] = thetaddot
        dydx[4] = F
        
        return dydx

    def step(self, dt):
        """execute one time step of length dt and update state"""
        self.state = integrate.odeint(self.dstate_dt, self.state, [0, dt])[1]
        self.time_elapsed += dt

#------------------------------------------------------------
# set up initial state and global variables
pendulumCart = PendulumOnCart([0.0, 0.0, 0.0, 0.0])
dt = 1./30 # 30 fps

#------------------------------------------------------------
# Calculate the control gains from the system parameters
# design for the case where you miscalculated the system parameters 
(M1, M2, L, b, g) = pendulumCart.params
M1_design = M1 
M2_design = M2
L_design  = L
b_design  = b
g_design  = g

# input constraint
F_max     = 5.0

# Select gains for inner loop
A_theta     = 1.25*np.pi/180
zeta_theta  = 0.9
kp_theta    = -F_max/A_theta
wn_theta    = np.sqrt(-(M1_design+M2_design)*g_design/M2_design/L_design-kp_theta/M2_design/L);
kd_theta    = -2*zeta_theta*wn_theta*M2_design*L_design

# DC gain of the inner loop using final value theorem
k_DC_theta  = kp_theta/((M1_design+M2_design)*g_design+kp_theta)

# Now use that to pick the gains for the outer loop
A_z = 1;
zeta_z = 0.707;
kp_z = A_theta/A_z;
wn_z = np.sqrt(M1_design*g_design/M2_design*kp_z);
kd_z = M2_design/M1_design/g_design*(-b_design/M2_design + 2*zeta_z*wn_z);
ki_z = 0.001

#------------------------------------------------------------
# Control variables and functions
position_integrator = 0.0
previous_error_z    = 0.0
previous_error_th   = 0.0
theta_max = 15*np.pi/180

def controlCart(state, z_d, dt):
    [z, theta, zdot, thetadot, F] = state
    
    theta_desired = PID_z(state, z_d, dt)    # outer control loop
    Force = PID_th(state, theta_desired, dt) # inner control loop
    return Force

def PID_z(state, z_d, dt):
    global position_integrator, previous_error_z, theta_max
    [z, theta, zdot, thetadot, F] = state
    
    error = z_d - z
    # anti_windup
    if error < 0.25:
        position_integrator = position_integrator + (dt/2)*(error + previous_error_z)
    
    theta_d_unsaturated = kp_z*error + ki_z*position_integrator - kd_z*zdot
    theta_d = sat(theta_d_unsaturated, theta_max)

    return theta_d

def PID_th(state, theta_d, dt):
    global previous_error_th, F_max
    [z, theta, zdot, thetadot, F] = state

    error = theta_d - theta

    F_unsaturated = kp_theta*error - kd_theta*thetadot
    F = sat(F_unsaturated, F_max)
    
    return F

def sat(unsaturated, limit):
    saturated = unsaturated
    if unsaturated > limit:
        saturated = limit
    elif unsaturated < -limit:
        saturated = -limit
    
    return saturated  

#------------------------------------------------------------
# set up figure and animation for pendulum
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-1, 1), ylim=(-1, 1))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init_pendulum():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')

    return line, time_text

def animate_pendulum(i):
    """perform animation step"""
    global pendulumCart, dt
    # this function takes in the pendulum cart state and acts on it to control the force
    # Every 5 seconds we're changing the desired location from -0.5 to 0.5 as a square wave
    if((pendulumCart.time_elapsed % 10) < 5):
        pendulumCart.state[4] = controlCart(pendulumCart.state, 0.5, dt)
    else:
        pendulumCart.state[4] = controlCart(pendulumCart.state, -0.5, dt)

    # step the differential equations forward one time step with the control input
    pendulumCart.step(dt) 
    
    line.set_data(*pendulumCart.position())
    time_text.set_text('time = %.1f' % pendulumCart.time_elapsed)

    return line, time_text

# set up figure for displaying the desired and current states of the system
plots = plt.figure()

ax_z        = plots.add_subplot(411, xlim=(0, 25), ylim=(-1.0, 1.0))
ax_theta    = plots.add_subplot(412, sharex=ax_z, sharey=ax_z)
ax_zdot     = plots.add_subplot(413, sharex=ax_z, sharey=ax_z)
ax_thetadot = plots.add_subplot(414, sharex=ax_z, sharey=ax_z)
ax_z.set_xlim(0, 25)
ax_z.set_ylim(-1, 1)

ax_z.grid()       
ax_theta.grid()   
ax_zdot.grid()    
ax_thetadot.grid()

line_z,         = ax_z.plot([], [])
line_theta,     = ax_theta.plot([], [])
line_zdot,      = ax_zdot.plot([], [])
line_thetadot,  = ax_thetadot.plot([], [])

ax_thetadot.set_xlabel('time (s)')

ax_z.set_ylabel('z')       
ax_theta.set_ylabel('theta')   
ax_zdot.set_ylabel('zdot')    
ax_thetadot.set_ylabel('thetadot')

z_data          = np.array([])
theta_data      = np.array([])
zdot_data       = np.array([])
thetadot_data   = np.array([])
plot_time       = np.array([])

axis_xlim   = 25.0
z_max  = 1.0
z_min  =-1.0
theta_max  = 1.0
theta_min  =-1.0
zdot_max  = 1.0
zdot_min  =-1.0
thetadot_max  = 1.0
thetadot_min  =-1.0

def init_plots():
    """initialize animation"""
    line_z.set_data([], [])
    line_theta.set_data([], [])  
    line_zdot.set_data([], [])   
    line_thetadot.set_data([], [])

    return line_z, line_theta, line_zdot, line_thetadot

def animate_plots(i):
    """perform animation step"""
    global pendulumCart, dt, z_data, theta_data, zdot_data, thetadot_data, plot_time, ax_z, axis_xlim, plots
    global theta_max, theta_min, z_max, z_min, zdot_max, zdot_min, thetadot_max, thetadot_min
    # the append function doesn't append to the array given by reference, so we have to pass it by value and simultaneously assign it to the original
    z_data        = np.append(z_data,        pendulumCart.state[0])
    theta_data    = np.append(theta_data,    pendulumCart.state[1])
    zdot_data     = np.append(zdot_data,     pendulumCart.state[2])
    thetadot_data = np.append(thetadot_data, pendulumCart.state[3])
    plot_time     = np.append(plot_time, pendulumCart.time_elapsed)
    
    # update the time axis when necessary... they are all linked to the same pointer so you only need to update theta1
    need_to_plot = False
    #if(pendulumCart.time_elapsed > axis_xlim - 1):
    #    axis_xlim += 2.0 # this number moves the x axis of all plots by a given amount
    #    ax_z.set_xlim(0, axis_xlim)
    #    need_to_plot = True

    # update the y-axis of each plot by a certain amount if the max or min is going off the plot
    # z check
    #if(z_min > z_data.min()):
    #    z_min = z_data.min() - 0.1
    #    ax_z.set_ylim(z_min, z_max)
    #    need_to_plot = True

    #if(z_max < z_data.max()):
    #    z_max = z_data.max() + 0.1
    #    ax_z.set_ylim(z_min, z_max)
    #    need_to_plot = True

    ## theta check
    #if(theta_min > theta_data.min()):
    #    theta_min = theta_data.min() - 0.1
    #    ax_theta.set_ylim(theta_min, theta_max)
    #    need_to_plot = True

    #if(theta_max < theta_data.max()):
    #    theta_max = theta_data.max() + 0.1
    #    ax_theta.set_ylim(theta_min, theta_max)
    #    need_to_plot = True

    ## zdot check
    #if(zdot_min > zdot_data.min()):
    #    zdot_min = zdot_data.min() - 0.1
    #    ax_zdot.set_ylim(zdot_min, zdot_max)
    #    need_to_plot = True

    #if(zdot_max < zdot_data.max()):
    #    zdot_max = zdot_data.max() + 0.1
    #    ax_zdot.set_ylim(zdot_min, zdot_max)
    #    need_to_plot = True

    ## thetadot check
    #if(thetadot_min > thetadot_data.min()):
    #    thetadot_min = thetadot_data.min() - 0.1
    #    ax_thetadot.set_ylim(thetadot_min, thetadot_max)
    #    need_to_plot = True

    #if(thetadot_max < thetadot_data.max()):
    #    thetadot_max = thetadot_data.max() + 0.1
    #    ax_thetadot.set_ylim(thetadot_min, thetadot_max)
    #    need_to_plot = True
        
    # update the plot if any of the axis limits have changed
    if need_to_plot:
        plots.show()

    line_z.set_data(plot_time, z_data)
    line_theta.set_data(plot_time, theta_data)
    line_zdot.set_data(plot_time, zdot_data)
    line_thetadot.set_data(plot_time, thetadot_data)
    return line_z, line_theta, line_zdot, line_thetadot

# choose the interval based on dt and the time to animate one step
t0 = time()
animate_pendulum(0)
animate_plots(0)
t1 = time()
interval = 1000 * dt - (t1 - t0)

simulation = animation.FuncAnimation(fig, animate_pendulum, frames=300,
                              interval=interval, blit=True, init_func=init_pendulum)
anim_plots = animation.FuncAnimation(plots, animate_plots, frames=300,
                              interval=interval, blit=True, init_func=init_plots)
plt.show()