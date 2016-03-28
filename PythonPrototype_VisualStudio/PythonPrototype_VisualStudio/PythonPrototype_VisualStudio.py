#nabbed this off of some robotics website and pared it down
#their full code is here http://www.roboticslab.ca/matplotlib-animation/

import numpy
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
from matplotlib import style

style.use("ggplot") 

# Setup figure and subplots
f0 = figure()
f0.suptitle("Oscillation decay", fontsize=12)
ax01 = subplot2grid((2, 2), (0, 0))
ax02 = subplot2grid((2, 2), (0, 1))
ax03 = subplot2grid((2, 2), (1, 0), colspan=2, rowspan=1)
ax04 = ax03.twinx()
#tight_layout()

# Set titles of subplots
ax01.set_title('Position vs Time')
ax02.set_title('Velocity vs Time')
ax03.set_title('Position and Velocity vs Time')

# set y-limits
ax01.set_ylim(0,2)
ax02.set_ylim(-6,6)
ax03.set_ylim(-0,5)
ax04.set_ylim(-10,10)

# sex x-limits
ax01.set_xlim(0,5.0)
ax02.set_xlim(0,5.0)
ax03.set_xlim(0,5.0)
ax04.set_xlim(0,5.0)

# Turn on grids
ax01.grid(True)
ax02.grid(True)
ax03.grid(True)

# set label names
ax01.set_xlabel("x")
ax01.set_ylabel("py")
ax02.set_xlabel("t")
ax02.set_ylabel("vy")
ax03.set_xlabel("t")
ax03.set_ylabel("py")
ax04.set_ylabel("vy")

# Data Placeholders
yp1=zeros(0)
yv1=zeros(0)
yp2=zeros(0)
yv2=zeros(0)
t=zeros(0)

# set plots
p011, = ax01.plot(t,yp1, label="yp1")
p012, = ax01.plot(t,yp2, label="yp2")

p021, = ax02.plot(t,yv1, label="yv1")
p022, = ax02.plot(t,yv2, label="yv2")

p031, = ax03.plot(t,yp1, label="yp1")
p032, = ax04.plot(t,yv1, label="yv1")

# set lagends
ax01.legend([p011,p012], [p011.get_label(),p012.get_label()])
ax02.legend([p021,p022], [p021.get_label(),p022.get_label()])
ax03.legend([p031,p032], [p031.get_label(),p032.get_label()])

# Data Update
xmin = 0.0
xmax = 5.0
x = 0.0

def updateData(self):
	global x
	global yp1
	global yv1
	global yp2
	global yv2
	global t

    # we won't actually do computation in the animation function in our final example, this is just to make the lines move
    # instead what we'll do is bring in the message from our subscriber and append it onto our data variables
	tmpp1 = 1 + exp(-x) *sin(2 * pi * x)
	tmpv1 = - exp(-x) * sin(2 * pi * x) + exp(-x) * cos(2 * pi * x) * 2 * pi
	yp1=append(yp1,tmpp1)
	yv1=append(yv1,tmpv1)
	yp2=append(yp2,0.5*tmpp1)
	yv2=append(yv2,0.5*tmpv1)
	t=append(t,x)

	x += 0.05

	p011.set_data(t,yp1)
	p012.set_data(t,yp2)

	p021.set_data(t,yv1)
	p022.set_data(t,yv2)

	p031.set_data(t,yp1)
	p032.set_data(t,yv1)

	if x >= xmax-1.00:
		p011.axes.set_xlim(x-xmax+1.0,x+1.0)
		p021.axes.set_xlim(x-xmax+1.0,x+1.0)
		p031.axes.set_xlim(x-xmax+1.0,x+1.0)
		p032.axes.set_xlim(x-xmax+1.0,x+1.0)

# interval: draw new frame every 'interval' ms
simulation = animation.FuncAnimation(f0, updateData, interval=250)

plt.show()
