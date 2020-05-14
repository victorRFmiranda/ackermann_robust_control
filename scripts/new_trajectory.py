#!/usr/bin/env python
# This code compute a trajectory online
# and publish orientation and velocity for the controller


import rospy
import rospkg

# messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import yaml



class trajectory:
	def __init__(self):
		rospy.init_node('trajectory', anonymous=True)
		self.t = []
		self.poly = []
		self.time = 0.0
		self.time_aux = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.v = 0.0
		self.theta = 0.0
		self.x = 0.0
		self.y = 0.0
		self.x_ant = 0.0
		self.y_ant = 0.0
		self.vel = Twist()
		self.angle = Float32()
		self.start_time = rospy.get_rostime().to_sec()
		self.pub_orientation = rospy.Publisher("yaw_angle", Float32, queue_size = 1)
		self.pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)



	def read(self, file_name):
		rospack = rospkg.RosPack()
		fl = rospack.get_path('ackermann_robust_control')+"/config/"+file_name
		a_yaml_file = open(fl)
		parsed_yaml_file = yaml.load(a_yaml_file, Loader=yaml.FullLoader)
		yaml_data = parsed_yaml_file.get("Trajectory")
		self.t = yaml_data["tf"]
		curves = yaml_data["number_of_curves"]
		for k in range(0,curves):
			self.poly.append(yaml_data["c_pos"+str(k)])



	def run(self):
		rate = rospy.Rate(5)
		while not rospy.is_shutdown() and self.time<self.t[0]+self.t[1]+self.t[2]+self.t[3]+self.t[4]+self.t[5]:
			self.time = rospy.get_rostime().to_sec() - self.start_time
			if(self.time <= self.t[0]):
				self.time_aux = self.time
				self.vx = 3*self.poly[0][3]*self.time_aux*self.time_aux + 2*self.poly[0][2]*self.time_aux + self.poly[0][1]
				self.vy = 3*self.poly[0][7]*self.time_aux*self.time_aux + 2*self.poly[0][6]*self.time_aux + self.poly[0][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[0][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[0][2]*self.time_aux*self.time_aux + self.poly[0][1]*self.time_aux + self.poly[0][0]
				self.y = self.poly[0][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[0][6]*self.time_aux*self.time_aux + self.poly[0][5]*self.time_aux + self.poly[0][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x
			elif(self.time > self.t[0] and self.time <= self.t[0]+self.t[1]):
				self.time_aux = self.time - self.t[0]
				self.vx = 3*self.poly[1][3]*self.time_aux*self.time_aux + 2*self.poly[1][2]*self.time_aux + self.poly[1][1]
				self.vy = 3*self.poly[1][7]*self.time_aux*self.time_aux + 2*self.poly[1][6]*self.time_aux + self.poly[1][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[1][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[1][2]*self.time_aux*self.time_aux + self.poly[1][1]*self.time_aux + self.poly[1][0]
				self.y = self.poly[1][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[1][6]*self.time_aux*self.time_aux + self.poly[1][5]*self.time_aux + self.poly[1][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x
			elif(self.time > self.t[0]+self.t[1] and self.time <= self.t[0]+self.t[1]+self.t[2]):
				self.time_aux = self.time - (self.t[0]+self.t[1])
				self.vx = 3*self.poly[2][3]*self.time_aux*self.time_aux + 2*self.poly[2][2]*self.time_aux + self.poly[2][1]
				self.vy = 3*self.poly[2][7]*self.time_aux*self.time_aux + 2*self.poly[2][6]*self.time_aux + self.poly[2][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[2][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[2][2]*self.time_aux*self.time_aux + self.poly[2][1]*self.time_aux + self.poly[2][0]
				self.y = self.poly[2][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[2][6]*self.time_aux*self.time_aux + self.poly[2][5]*self.time_aux + self.poly[2][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x
			elif(self.time > self.t[0]+self.t[1]+self.t[2] and self.time <= self.t[0]+self.t[1]+self.t[2]+self.t[3]):
				self.time_aux = self.time - (self.t[0]+self.t[1]+self.t[2])
				self.vx = 3*self.poly[3][3]*self.time_aux*self.time_aux + 2*self.poly[3][2]*self.time_aux + self.poly[3][1]
				self.vy = 3*self.poly[3][7]*self.time_aux*self.time_aux + 2*self.poly[3][6]*self.time_aux + self.poly[3][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[3][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[3][2]*self.time_aux*self.time_aux + self.poly[3][1]*self.time_aux + self.poly[3][0]
				self.y = self.poly[3][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[3][6]*self.time_aux*self.time_aux + self.poly[3][5]*self.time_aux + self.poly[3][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x
			elif(self.time > self.t[0]+self.t[1]+self.t[2]+self.t[3] and self.time <= self.t[0]+self.t[1]+self.t[2]+self.t[3]+self.t[4]):
				self.time_aux = self.time - (self.t[0]+self.t[1]+self.t[2]+self.t[3])
				self.vx = 3*self.poly[4][3]*self.time_aux*self.time_aux + 2*self.poly[4][2]*self.time_aux + self.poly[4][1]
				self.vy = 3*self.poly[4][7]*self.time_aux*self.time_aux + 2*self.poly[4][6]*self.time_aux + self.poly[4][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[4][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[4][2]*self.time_aux*self.time_aux + self.poly[4][1]*self.time_aux + self.poly[4][0]
				self.y = self.poly[4][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[4][6]*self.time_aux*self.time_aux + self.poly[4][5]*self.time_aux + self.poly[4][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x
			elif(self.time > self.t[0]+self.t[1]+self.t[2]+self.t[3]+self.t[4] and self.time <= self.t[0]+self.t[1]+self.t[2]+self.t[3]+self.t[4]+self.t[5]):				
				self.time_aux = self.time - (self.t[0]+self.t[1]+self.t[2]+self.t[3]+self.t[4])
				self.vx = 3*self.poly[5][3]*self.time_aux*self.time_aux + 2*self.poly[5][2]*self.time_aux + self.poly[5][1]
				self.vy = 3*self.poly[5][7]*self.time_aux*self.time_aux + 2*self.poly[5][6]*self.time_aux + self.poly[5][5]
				self.v = math.sqrt(self.vx*self.vx + self.vy*self.vy)
				self.x = self.poly[5][3]*self.time_aux*self.time_aux*self.time_aux + self.poly[5][2]*self.time_aux*self.time_aux + self.poly[5][1]*self.time_aux + self.poly[5][0]
				self.y = self.poly[5][7]*self.time_aux*self.time_aux*self.time_aux + self.poly[5][6]*self.time_aux*self.time_aux + self.poly[5][5]*self.time_aux + self.poly[5][4]
				self.theta = math.atan2((self.y-self.y_ant),(self.x-self.x_ant))
				self.y_ant = self.y
				self.x_ant = self.x

			self.angle.data = self.theta
			self.vel.linear.x = self.v
			self.pub_orientation.publish(self.angle)
			self.pub_vel.publish(self.vel)
			print("Velocity: %f \t Orientation: %f \n" % (self.v,self.theta))
			rate.sleep()

		print("acabou")
		self.vel.linear.x = 0.0
		self.pub_vel.publish(self.vel)





if __name__ == '__main__':
	try:

		r = trajectory()
		file_name = 'XQuad_Trajectory.yaml'
		r.read(file_name)
		r.run()

	except rospy.ROSInterruptException:
		pass
		