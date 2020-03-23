from __future__ import division
import numpy as np
import math
import rospy


####### Function Compute Derivative ##########
def derivative(Ts,x0,x1):
	return ((x1-x0)/Ts)


##### Orientation Control Class ########
class Orientation_control:
	def __init__(self, k, c1, c2, Tz, etol):
		self.K = k
		self.C1 = c1
		self.C2 = c2
		self.Ts = Tz
		self.etol = etol

		self.setpoint = 0.0
		self.last_setpoint = 0.0
		self.d_setpoint = 0.0
		self.int_erro = 0.0
		self.last_error = 0.0
		self.last_alpha2 = 0.0


        #### Parameters
		self.m = rospy.get_param('ackermann_control/dynamics/mass')						# mass (kg)
		self.Cr = rospy.get_param('ackermann_control/dynamics/cr')
		self.Cf = rospy.get_param('ackermann_control/dynamics/cf')
		self.lf = rospy.get_param('ackermann_control/dynamics/lf')
		self.lr = rospy.get_param('ackermann_control/dynamics/lr')
		self.Iz = rospy.get_param('ackermann_control/dynamics/inertia_z')

###### Update Control Law ##############
	def update(self, ref, velocity, orientation, vel_ang):
		error = ref - orientation
		if (abs(error) < self.etol):
			error = 0.0

		self.int_erro = self.int_erro + ((self.last_error + error)/2)*self.Ts
		self.last_error = error

		alpha1 = ref - self.K*self.int_erro
		z1 = orientation - alpha1
		self.d_setpoint = derivative(self.Ts, self.last_setpoint, ref)
		self.last_setpoint = ref
		alpha2 = self.d_setpoint - self.K*z1 - self.int_erro*self.K*self.K + self.int_erro - self.C1*z1
		z2 = vel_ang - alpha2
		d_alpha2 = derivative(self.Ts, self.last_alpha2, alpha2)
		self.last_alpha2 = alpha2

		if (abs(velocity) < 0.4):
			un = 0.0
		else:
			un = d_alpha2 + (2.0/(self.Iz*velocity))*(self.Cf*self.lf*self.lf + self.Cr*self.lr*self.lr)*vel_ang - z1 - self.C2*z2

		u = (self.Iz/(2.0*self.lf*self.Cf))*un

		if (u > 0.5):
			u = 0.5
		elif (u < -0.5):
			u = -0.5

		return u



##### Velocity Control Class ######
class Vel_control:
	def __init__(self, k, c1, c2, Tz, etol):
		self.K = k
		self.C1 = c1
		self.C2 = c2
		self.Ts = Tz
		self.etol = etol
		
		self.setpoint = 0.0		      # setpoint
		self.d_setpoint = 0.0         # first derivative of setpoint		
		self.last_setpoint = 0.0      # last setpoint
		self.last_alpha2 = 0.0		  # last auxiliar control law
		self.torque = 0.0			  # motor torque
		self.int_erro= 0.0			  # error integral
		self.last_error = 0.0		  # last error

		self.u_filtrado_ant = 0.0	  # filter control law
		
		#### Parameters
		self.m = rospy.get_param('ackermann_control/dynamics/mass')						# mass (kg)
		self.g = rospy.get_param('ackermann_control/dynamics/gravity')								# gravity (m/s^2)
		self.inclination = 0.0																	# road slope							#
		self.Fa = rospy.get_param('ackermann_control/dynamics/drag')		#
		self.r = rospy.get_param('ackermann_control/dynamics/wheel_radio')								# wheel radius
		self.n = rospy.get_param('ackermann_control/dynamics/motor_efficiency')								# motor efficiency
		self.tal = rospy.get_param('ackermann_control/dynamics/inertial_coef')							#
		self.H = rospy.get_param('ackermann_control/dynamics/friction_coef')							# friction coeff
	
####### Update control law ########
	def update(self, ref, velocity):
		error = ref - velocity
		if (abs(error) < self.etol):
			error = 0.0

		self.int_erro = self.int_erro + ((self.last_error + error)/2)*self.Ts
		self.last_error = error

		alpha1 = ref + self.K*self.int_erro
		z1 = velocity - alpha1
		signal = (1.0-math.exp(-velocity))/(1+math.exp(-velocity))
		self.d_setpoint = derivative(self.Ts, self.last_setpoint, ref)
		self.last_setpoint = ref
		alpha2 = (self.m*self.r/self.n)*(self.Fa*abs(velocity)*velocity + self.g*math.sin(self.inclination) + self.H*self.g*math.cos(self.inclination)*signal + self.d_setpoint - self.K*z1 - self.int_erro*self.K*self.K + self.int_erro - self.C1*z1)
		z2 = self.torque - alpha2
		d_alpha2 = derivative(self.Ts, self.last_alpha2, alpha2)
		self.last_alpha2 = alpha2
		un = d_alpha2 - (self.n/(self.m*self.r))*z1 - self.C2*z2
		control = self.tal*un + self.torque

		self.torque = control

		
		#beta = 0.65
		#u_filtrado = beta*self.u_filtrado_ant + (1-beta)*u_control
		#self.u_filtrado_ant = u_filtrado
		
		#return u_filtrado
		return control