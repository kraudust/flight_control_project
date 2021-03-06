#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from fcu_common.msg import FW_State
from rosgraph_msgs.msg import Clock


class state_subscriber():

	def __init__(self):
		#----------------------States-------------------------------
		self.pn = None	
		self.pe = None
		self.pd = None
		self.Va = None
		self.alpha = None
		self.beta = None
		self.phi = None
		self.theta = None
		self.psi = None
		self.chi = None
		self.p = None
		self.q = None
		self.r = None
		self.Vg = None
		self.wn = None
		self.we = None
		self.time = None
		#------------------------------------------------------------
		rospy.Subscriber("/junker/truth", FW_State, self.callback)
		rospy.Subscriber("/clock", Clock, self.callback_time)
		self.rate = 100 # 10 hz

	def callback(self, FW_State):
		self.pn = FW_State.position[0]
		self.pe = FW_State.position[1]
		self.pd = FW_State.position[2]
		self.Va = FW_State.Va
		self.alpha = FW_State.alpha
		self.beta = FW_State.beta
		self.phi = FW_State.phi
		self.theta = FW_State.theta
		self.psi = FW_State.psi
		self.chi = FW_State.chi
		self.p = FW_State.p
		self.q = FW_State.q
		self.r = FW_State.r
		self.Vg = FW_State.Vg
		self.wn = FW_State.wn
		self.we = FW_State.we
		
	def callback_time(self, Clock):
		self.time = Clock.clock
		
	def print_states(self):
		print "pn: ", self.pn
		print "pe: ", self.pe
		print "pd: ", self.pd
		print "Va: ", self.Va
		print "alpha: ", self.alpha
		print "beta: ", self.beta
		print "phi: ", self.phi
		print "theta: ", self.theta
		print "psi: ", self.psi
		print "chi: ", self.chi
		print "p: ", self.p
		print "q:", self.q
		print "r:", self.r
		print "Vg:", self.Vg
		print "wn:", self.wn
		print "we:", self.we
		if self.time == None:
		    #do nothing
		    eric = 7
		else:
		    print "time: ", self.time.to_sec(), '\n'

		
if __name__ == '__main__':	
	rospy.init_node('plotter', anonymous=True)
	#rospy.init_node('time', anonymous = True)
	states = state_subscriber()
	rate = rospy.Rate(states.rate)

	try:
		while not rospy.is_shutdown():
			states.print_states()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass	
