#!/usr/bin/python2

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import *
from ekf import EKF
import numpy as np
import math
import tf

from scipy import stats


# Global Variables
initilization = True

class GSF:
	def __init__(self,M,w):		
		self.M = M

		# Dictionary to hold EKFs
		self.ekf_dict = {}

		# Dictionary to hold weights
		self.w = w

	def gsf_fill_dict(self, key, x_0, P_0):
		self.ekf_dict[key] = EKF(x_0,P_0)

	def gsf_predict(self,u,delta_t):
		for i in range(self.M):
			self.ekf_dict[i].prediction(u,delta_t)

	def gsf_correct(self,y,delta_t):
		for i in range(self.M):
			self.ekf_dict[i].correction(y, delta_t)

	def weight_update(self, y):
		w_times_N = np.array(self.M,1)
		for i in range(self.M):
			w_times_N[i] = w[i]*stats.multi_variate_normal(y,mean=self.ekf_dict[i].get_y_hat_plus_one_minus(), cov=self.ekf_dict[i].get_S_k_plus_one())
		sum_w_times_n = np.sum(w_times_N)
		for i in range(self.M):
			self.w[i] = w_times_N[i]/sum_w_times_n

	def get_mu(self):
		mu = np.zeros((self.M,1))
		for i in range(self.M):
			mu += self.w[i]*self.ekf_dict[i].get_x_hat_k_plus_one_plus()
		return mu

	def get_sigma(self):
		sigma = np.zeros((self.M,self.M))
		for i in range(self.M):
			x = self.ekf_dict[i].get_x_hat_k_plus_one_plus()
			mu = self.get_mu()
			P_tilde = self.ekf_dict[i].get_P_k_plus_one_plus() + np.dot(x,x.T)
			sigma += self.w[i]*P_tilde - np.dot(mu,mu.T)
		return sigma


class ros_odom_sub_pub:
	def __init__(self):
		self.initialization = 1
		self.slow_vel_count = 0
		self.frame_id = "/map"
		# Subscribers and Publishers
		self.odom_sub = rospy.Subscriber('cart_odom', Odometry, self.odom_callback)
		self.filter_output_pub = rospy.Publisher('filter_out', Odometry, queue_size = 100)
		self.P_publish = rospy.Publisher('covariance_matrix', Float64MultiArray, queue_size = 100)
		self.x_store = np.zeros([5,1])
		self.P_store = np.zeros([25,1])
		self.M = 5

	def odom_callback(self,odom_msg):
		self.slow_vel_count += 1
		if(self.initialization):
			self.initialization = 0
			w_0 = 1./self.M*np.ones((5,1))
			self.gsf_obj = GSF(self.M,w_0)
			for i in range(self.M):
				# Establish x_0
				if i == 0:
					x_0 = np.empty([5,1])
					x_0[0] = odom_msg.pose.pose.position.x + 0.1
					x_0[1] = odom_msg.pose.pose.position.y
					orientation_quat = odom_msg.pose.pose.orientation
					orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
					x_0[2] = orientation_euler[2]	
					x_0[3] = 0
					x_0[4] = 0
				if i == 1:
					x_0 = np.empty([5,1])
					x_0[0] = odom_msg.pose.pose.position.x - 0.1
					x_0[1] = odom_msg.pose.pose.position.y
					orientation_quat = odom_msg.pose.pose.orientation
					orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
					x_0[2] = orientation_euler[2]	
					x_0[3] = 0
					x_0[4] = 0
				if i == 2:
					x_0 = np.empty([5,1])
					x_0[0] = odom_msg.pose.pose.position.x 
					x_0[1] = odom_msg.pose.pose.position.y + 0.1
					orientation_quat = odom_msg.pose.pose.orientation
					orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
					x_0[2] = orientation_euler[2]	
					x_0[3] = 0
					x_0[4] = 0
				if i == 3:
					x_0 = np.empty([5,1])
					x_0[0] = odom_msg.pose.pose.position.x
					x_0[1] = odom_msg.pose.pose.position.y - 0.1
					orientation_quat = odom_msg.pose.pose.orientation
					orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
					x_0[2] = orientation_euler[2]	
					x_0[3] = 0
					x_0[4] = 0
				if i == 4:
					x_0 = np.empty([5,1])
					x_0[0] = odom_msg.pose.pose.position.x
					x_0[1] = odom_msg.pose.pose.position.y
					self.x_prev = x_0[0]
					self.y_prev = x_0[1]
					orientation_quat = odom_msg.pose.pose.orientation
					orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
					x_0[2] = orientation_euler[2]	
					x_0[3] = 0
					x_0[4] = 0

				# Establish P_0
				P_0 = 0.1*np.identity(5)

				# Initialize EKFs                
				self.gsf_obj.gsf_fill_dict(i,x_0,P_0)

			self.time_prev = odom_msg.header.stamp
			self.slow_time_prev = self.time_prev	
		
		time_curr = odom_msg.header.stamp
		d = time_curr - self.time_prev
		delta_t = d.to_sec()

		# Establish u_k from imu_msg 
		imu_msg = rospy.wait_for_message("/L01/imu_raw", Imu)
		u = np.empty([3,1])
		u[0][0] = imu_msg.angular_velocity.z
		u[1][0] = imu_msg.linear_acceleration.x
		u[2][0] = imu_msg.linear_acceleration.y
		self.gsf_obj.gsf_predict(u,delta_t)

		# Establish y_k+1 from odom msg
		y = np.empty([3,1])
		y[0][0] = odom_msg.pose.pose.position.x
		y[1][0] = odom_msg.pose.pose.position.y
		orientation_quat = odom_msg.pose.pose.orientation
		orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
		y[2][0] = orientation_euler[2]

		# Can possibly create fake velocities here?
		if(self.slow_vel_count % 100==0):
			# Find Current values
			slow_time_curr = odom_msg.header.stamp
			x_curr = y[0]
			y_curr = y[1]
			slow_time_delta = slow_time_curr - self.slow_time_prev
			delta_t_slow = slow_time_delta.to_sec()

			# Find velocitie estimates
			x_vel = (x_curr - self.x_prev)/delta_t_slow
			y_vel = (y_curr - self.y_prev)/delta_t_slow
			y_new = np.empty([5,1])
			y_new[0] = y[0]
			y_new[1] = y[1]
			y_new[2] = y[2]
			y_new[3] = x_vel
			y_new[4] = y_vel
			# Set previous values to current values
			self.x_prev = x_curr
			self.y_prev = y_curr
			self.slow_time_prev = slow_time_curr
			y = y_new

		# Given measurements, correct x_hat estimate
		self.gsf_obj.gsf_correct(y,delta_t)

		# Get and publish MMSE
		odom_output = Odometry()
		odom_output.header.stamp = rospy.Time.now()
		odom_output.header.frame_id = self.frame_id

		mu_k_plus_one_plus = self.gsf_obj.get_mu()
		self.x_store = np.append(self.x_store,mu_k_plus_one_plus,axis=1)
		rotation_quat = tf.transformations.quaternion_from_euler(0,0,mu_k_plus_one_plus[2])
		odom_output.pose.pose = Pose(Point(mu_k_plus_one_plus[0], mu_k_plus_one_plus[1], 0), Quaternion(*rotation_quat))
		odom_output.twist.twist = Twist(Vector3(mu_k_plus_one_plus[3], mu_k_plus_one_plus[4], 0), Vector3(0,0,u[0]))

		self.filter_output_pub.publish(odom_output)
		Sigma_k_plus_one_plus = self.gsf_obj.get_sigma()

		self.P_store = np.append(self.P_store,Sigma_k_plus_one_plus.reshape(25,1),axis=1)
		P_msg = Float64MultiArray()
		P_msg.layout.dim.append(MultiArrayDimension())
		P_msg.layout.dim[0].size = 5
		P_msg.layout.dim[0].stride = 25
		P_msg.layout.dim.append(MultiArrayDimension())
		P_msg.layout.dim[1].size = 5
		P_msg.layout.dim[1].stride = 5
		P_msg.data = Sigma_k_plus_one_plus.reshape(25)
		
		
		# rospy.loginfo("Publishing y")
		self.P_publish.publish(P_msg)

		self.time_prev = time_curr
		if(self.slow_vel_count==22000):
			#print "Saving to ekf.mat"
			#scipy.io.savemat('ekf',{'x': self.x_store, 'P': self.P_store})
			self.odom_sub.unregister()
			print "Save to csv"
			np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/gsf_x.csv",self.x_store,delimiter=",")
			np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/gsf_P.csv",self.P_store,delimiter=",")
			print "Save complete"
			rospy.signal_shtudown("Ended GSF")


def main():
	rospy.init_node('ekf', anonymous=True)
	my_ros = ros_odom_sub_pub()
	rospy.spin()

if __name__ == '__main__':	
	try:
		main()
	except rospy.ROSInterruptException:
		pass
