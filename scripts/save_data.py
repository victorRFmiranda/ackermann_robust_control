#!/usr/bin/env python

import rospy

# messages
from geometry_msgs.msg import Twist, Wrench, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import tf2_ros




class trajectory:
	def __init__(self, file_name):
		self.file = open(file_name, "w+")
		rospy.init_node('save_imu', anonymous=True)
		#self.file.write("o.w\to.x\to.y\to.z\ta.x\ta.y\ta.z\tl.x\tl.y\tl.z\n")
		# rospy.Subscriber("/imu_data", Imu, self.imu)
		#rospy.Subscriber("/gps", Odometry, self.gps)
		# rospy.Subscriber("/odom", Odometry, self.odom)
		rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf)
		rospy.Subscriber("/tf", TFMessage, self.TF)
		# tfBuffer = tf2_ros.Buffer()
		# listener = tf2_ros.TransformListener(tfBuffer)
		# TF = tfBuffer.lookup_transform('world', 'world', rospy.Time())
		# print(TF)
		self.tf = TransformStamped()
		self.ekf = Odometry()
		print("saving!")
		#rospy.spin()
		

	def TF(self, msg):
		if (msg.transforms[0].child_frame_id != "base_footprint"):
			return
		else:
			pos = msg.transforms[0].transform.translation
			ori = msg.transforms[0].transform.rotation
			self.tf = msg.transforms[0]
			#print(pos)
			#self.file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w))

	def ekf(self, msg):
		self.ekf = msg
		#self.file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

	def imu(self, msg):
		self.file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))


	def gps(self, msg):
		self.file.write("%f\t%f\t%f\n" % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))


	def odom(self, msg):
		self.file.write("%f\t%f\t%f\t%f\t%f\n" % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.twist.twist.linear.x, msg.twist.twist.angular.z))

	def run(self):
		rate = rospy.Rate(11)
		while not rospy.is_shutdown():
			pos = self.tf.transform.translation
			ori = self.tf.transform.rotation
			ekf_pos = self.ekf.pose.pose.position
			ekf_ori = self.ekf.pose.pose.orientation
			self.file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, ekf_pos.x, ekf_pos.y, ekf_pos.z, ekf_ori.x, ekf_ori.y, ekf_ori.z, ekf_ori.w))
			rate.sleep()

if __name__ == '__main__':
	try:
		f_name = '/home/victor/results.txt'
		r = trajectory(f_name)
		r.run()
		r.file.close()
		#r.run()

	except rospy.ROSInterruptException:
		pass
		