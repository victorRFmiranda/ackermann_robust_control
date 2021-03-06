#!/usr/bin/env python
# This code read a .txt file with data of velocity and orientation
# This data is a trajectory


import rospy
import rospkg
# library
from lib.backstepping_class import Orientation_control

# messages
from geometry_msgs.msg import Twist, Wrench, QuaternionStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32




class trajectory:
	def __init__(self, file_name):
		rospack = rospkg.RosPack()
		self.vehicle_number = 'master'
		self.orientation_pos = []
		self.velocity = []
		self.angle = QuaternionStamped()
		self.file = open(rospack.get_path('ackermann_robust_control')+"/config/"+file_name, "r")
		rospy.init_node('trajectory', anonymous=True)
		self.pub_orientation = rospy.Publisher("/yaw_angle", QuaternionStamped, queue_size = 1)
		self.pub_vel = rospy.Publisher("/cmd_vel", TwistStamped, queue_size = 1)



	def read(self):
		aux = []
		for x in self.file.read().split():
			aux.append(x)

		for i in range(1,len(aux),2):
			self.velocity.append(aux[i])

		for i in range(2,len(aux),2):
			self.orientation_pos.append(aux[i])

	def run(self):
		rate = rospy.Rate(20)
		count = 0
		cont = 0
		vel = TwistStamped()
		# vel.linear.x = 1.25
		#print(self.orientation_pos)
		while not rospy.is_shutdown() and cont < 1*len(self.orientation_pos):
			self.angle.header.stamp = rospy.get_rostime()
			self.angle.header.frame_id = ("vehicle_")+str(self.vehicle_number)
			self.angle.quaternion.z = float(self.orientation_pos[count])
			self.pub_orientation.publish(self.angle)

			vel.header.stamp = rospy.get_rostime()
			vel.header.frame_id = ("vehicle_")+str(self.vehicle_number)
			vel.twist.linear.x = float(self.velocity[count])
			self.pub_vel.publish(vel)
			count = count + 1
			cont = cont + 1
			if (count >= len(self.orientation_pos)):
				count = 0
			#print("Count: %d \t Cont: %d" % (count, cont))
			print("Velocity: %f \t Orientation: %f \n" % (float(self.velocity[count]),float(self.orientation_pos[count])))
			rate.sleep()

		vel.twist.linear.x = 0.0
		self.pub_vel.publish(vel)





if __name__ == '__main__':
	try:
		f_name = rospy.get_param('ackermann_control/file')
		r = trajectory(f_name)
		r.read()
		r.file.close()
		r.run()

	except rospy.ROSInterruptException:
		pass
		