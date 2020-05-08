#!/usr/bin/python2

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import *
import numpy as np
import math
import tf
import scipy.io

# Global Variables
initilization = True

# Global Variables
initilization = True

class EKF:
	def __init__(self,x_0,P_0):
		# Define x_0
		self.x_hat_k_plus_one_plus = x_0

		# Tunables: Q_t and P_0
		sigma_x = 0.05 # m^2
		sigma_y = 0.05 # m^2
		sigma_theta = 0.01 # rad
		sigma_x_dot = 0.5 # m^2/s
		sigma_y_dot = 0.5 # m^2/s
		self.Q_t = np.array([[sigma_x**2, 0., 0., 0., 0.],
							[0., sigma_y**2, 0., 0., 0.],
							[0., 0., sigma_theta**2, 0., 0.],
							[0., 0., 0., sigma_x_dot, 0.],
							[0., 0., 0., 0., sigma_y_dot]])
		
		# Define P_0
		self.P_k_plus_one_plus = P_0

		# Estimated by sensor outputs
		sigma_r_x = 0.005 # m^2
		sigma_r_y = 0.005 # m^2
		sigma_r_theta = 0.005 # rad
		sigma_r_x_dot = 0.1 # m^2/s
		sigma_r_y_dot = 0.1 # m^2/s
		self.R_t = np.array([[sigma_r_x**2, 0., 0.],
							[0., sigma_r_y**2, 0.],
							[0., 0., sigma_r_theta**2]])
		self.R_t_vel = np.array([[sigma_r_x**2, 0., 0., 0., 0.],
								[0., sigma_r_y**2, 0., 0., 0.],
								[0., 0., sigma_r_theta**2, 0., 0.],
								[0., 0., 0., sigma_r_x_dot, 0.],
								[0., 0., 0., 0., sigma_r_y_dot]])

	def jacobian(self,x,u,delta_t):
		s = np.sqrt([x[3]**2+x[4]**2])
		F_tilde = np.identity(5)
		A_tilde = np.zeros((5,5))
		A_tilde[0,2] = -s*math.sin(x[2])
		A_tilde[1,2] = s*math.cos(x[2])
		if(s > 0):
			A_tilde[0,3] = x[3]*math.cos(x[2])/s
			A_tilde[0,4] = x[4]*math.cos(x[2])/s
			A_tilde[1,3] = x[3]*math.sin(x[2])/s
			A_tilde[1,4] = x[4]*math.sin(x[2])/s
		
		F_tilde = np.identity(5)+A_tilde*delta_t
		return F_tilde

	def f(self,x,u,delta_t):
		s = np.sqrt([x[3]**2+x[4]**2])
		x_hat_k_plus_one_minus = np.zeros(np.shape(x))
		x_hat_k_plus_one_minus[0] = x[0] + s*math.cos(x[2])*delta_t
		x_hat_k_plus_one_minus[1] = x[1] + s*math.sin(x[2])*delta_t
		x_hat_k_plus_one_minus[2] = x[2] + u[0]*delta_t
		x_hat_k_plus_one_minus[2] = np.unwrap(x_hat_k_plus_one_minus[2])
		x_hat_k_plus_one_minus[3] = x[3] + u[1]*delta_t
		x_hat_k_plus_one_minus[4] = x[4] + u[2]*delta_t
		return x_hat_k_plus_one_minus

	def h(self,x,delta_t, vel):
		if vel:
			y = x
		else:
			y = x[0:3]
		return y		

	def prediction(self,u,delta_t):
		self.x_hat_k_plus_one_minus = self.f(self.x_hat_k_plus_one_plus,u,delta_t)
		# print(np.isnan(self.x_hat_k_plus_one_minus))
		F_tilde = self.jacobian(self.x_hat_k_plus_one_plus,u,delta_t)
		self.P_k_plus_one_minus = np.dot(F_tilde,np.dot(self.P_k_plus_one_plus,F_tilde.T)) + self.Q_t
		
	def correction(self,y,delta_t):
		if y.size == 5:
			vel = 1
			R = self.R_t_vel
			H = np.identity(5)
		else:
			vel = 0
			R = self.R_t
			H = np.array([[1., 0., 0., 0., 0.],
						[0., 1., 0., 0., 0.],
						[0., 0., 1., 0., 0.]])
		self.y_hat_k_plus_one_minus = self.h(self.x_hat_k_plus_one_minus,delta_t,vel)
		e_y_k_plus_one = y-self.y_hat_k_plus_one_minus
		C_xy = np.dot(self.P_k_plus_one_minus,H.T)
		self.S_k_plus_one = np.dot(H,C_xy)+R
		S_k_plus_one_inv = np.linalg.inv(self.S_k_plus_one)
		K = np.dot(C_xy,S_k_plus_one_inv)
		self.x_hat_k_plus_one_plus = self.x_hat_k_plus_one_minus+np.dot(K,e_y_k_plus_one)
		# print(np.isnan(self.x_hat_k_plus_one_plus))
		temp_4 = np.dot(K,H)
		temp_5 = np.eye(np.shape(temp_4)[0], np.shape(temp_4)[1])-temp_4
		self.P_k_plus_one_plus = np.dot(temp_5,self.P_k_plus_one_minus)

	def get_x_hat_k_plus_one_plus(self):
		return self.x_hat_k_plus_one_plus

	def get_y_hat_plus_one_minus(self):
		return self.y_hat_k_plus_one_minus

	def get_P_k_plus_one_plus(self):
		return self.P_k_plus_one_plus

	def get_S_k_plus_one(self):
		return self.S_k_plus_one

class ros_odom_sub_pub:
	def __init__(self):
		self.initialization = 1
		self.slow_vel_count = 0
		self.frame_id = "/map"
		# Subscribers and Publishers
		self.odom_sub = rospy.Subscriber('cart_odom', Odometry, self.odom_callback)
		self.filter_output_pub = rospy.Publisher('filter_out', Odometry, queue_size = 100)
		self.P_publish = rospy.Publisher('covariance_matrix', Float64MultiArray, queue_size = 100)
		self.x_store = np.empty([5,1])
		self.P_store = np.empty([25,1])
		self.y_store = np.empty([3,1])

	def odom_callback(self,odom_msg):
		# rospy.loginfo("Enterd Callback")
		self.slow_vel_count += 1
		# print self.slow_vel_count
		if(self.initialization==1):
			# rospy.loginfo("Initializing EKF")
			self.initialization = 0
			# Establish x_0
			x_0 = np.empty((5,1))
			x_0[0] = odom_msg.pose.pose.position.x
			x_0[1] = odom_msg.pose.pose.position.y
			orientation_quat = odom_msg.pose.pose.orientation
			orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
			x_0[2] = orientation_euler[2]	
			x_0[3] = 0
			x_0[4] = 0
			self.x_store[:] = x_0
			# P_0
			P_0 = 0.1*np.identity(5)
			self.P_store[:] = P_0.reshape((25,1))
			self.ekf_obj = EKF(x_0,P_0)
			self.time_prev = odom_msg.header.stamp
			self.slow_time_prev = odom_msg.header.stamp
			self.x_prev = x_0[0]
			self.y_prev = x_0[1]
			# rospy.loginfo("EKF Initialized")
			return

		time_curr = odom_msg.header.stamp
		d = time_curr - self.time_prev
		delta_t = d.to_sec()
		# print delta_t
		# Establish u_k from imu_msg 
		imu_msg = rospy.wait_for_message("/L01/imu_raw", Imu)
		u = np.empty([3,1])
		u[0][0] = imu_msg.angular_velocity.z
		u[1][0] = imu_msg.linear_acceleration.x
		u[2][0] = imu_msg.linear_acceleration.y

		# Predict new x_hat
		# rospy.loginfo("Predicting")
		self.ekf_obj.prediction(u,delta_t)
		
		# Establish y_k+1 from odom msg
		y = np.empty([3,1])
		y[0][0] = odom_msg.pose.pose.position.x
		y[1][0] = odom_msg.pose.pose.position.y
		orientation_quat = odom_msg.pose.pose.orientation
		orientation_euler = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
		y[2][0] = orientation_euler[2]
		self.y_store = np.append(self.y_store,y,axis=1)
		# Can possibly create fake velocities here?
		if(self.slow_vel_count % 100 == 0):
			# rospy.loginfo("Adding velocity")
			# Find Current values
			slow_time_curr = odom_msg.header.stamp
			x_curr = y[0]
			y_curr = y[1]
			
			slow_time_delta = slow_time_curr - self.slow_time_prev
			delta_t_slow = slow_time_delta.to_sec()
			#print self.slow_time_prev
			#print slow_time_curr
			#print delta_t_slow

			# Find velocitie estimates
			if delta_t_slow > 0:
				x_vel = (x_curr - self.x_prev)/delta_t_slow
				y_vel = (y_curr - self.y_prev)/delta_t_slow
			else:
				x_vel = 0
				y_vel = 0
			y_new = np.empty([5,1])
			y_new[0] = y[0]
			y_new[1] = y[1]
			y_new[2] = y[2]
			y_new[3] = x_vel
			y_new[4] = y_vel
			# Set previous values to current values
			x_prev = x_curr
			y_prev = y_curr
			self.slow_time_prev = slow_time_curr
			y = y_new

		# Given measurements, correct x_hat estimate
		# rospy.loginfo("Correcting")
		self.ekf_obj.correction(y,delta_t)
		
		odom_output = Odometry()
		# odom_output.header.stamp = time_curr
		odom_output.header.stamp = rospy.Time.now()
		odom_output.header.frame_id = self.frame_id
		x_hat_k_plus_one_plus = self.ekf_obj.get_x_hat_k_plus_one_plus()
		self.x_store = np.append(self.x_store,x_hat_k_plus_one_plus,axis=1)
		#print self.x_store
		rotation_quat = tf.transformations.quaternion_from_euler(0,0,x_hat_k_plus_one_plus[2])
		odom_output.pose.pose = Pose(Point(x_hat_k_plus_one_plus[0], x_hat_k_plus_one_plus[1], 0), Quaternion(*rotation_quat))
		
		odom_output.twist.twist = Twist(Vector3(x_hat_k_plus_one_plus[3], x_hat_k_plus_one_plus[4], 0), Vector3(0,0,u[0]))

		
		s = self.ekf_obj.get_P_k_plus_one_plus()
		self.P_store = np.append(self.P_store,s.reshape(25,1),axis=1)
		P_msg = Float64MultiArray()
		P_msg.layout.dim.append(MultiArrayDimension())
		P_msg.layout.dim[0].size = 5
		P_msg.layout.dim[0].stride = 25
		P_msg.layout.dim.append(MultiArrayDimension())
		P_msg.layout.dim[1].size = 5
		P_msg.layout.dim[1].stride = 5
		P_msg.data = s.reshape(25)
		
		
		# rospy.loginfo("Publishing y")
		self.P_publish.publish(P_msg)

		odom_output.pose.covariance[0] = s[0][0]
		odom_output.pose.covariance[7] = s[1][1]
		odom_output.pose.covariance[35] = s[2][2]
		# rospy.loginfo("Publishing x")
		self.filter_output_pub.publish(odom_output)

		self.time_prev = time_curr
		if(self.slow_vel_count % 1000 == 0):
			print self.slow_vel_count
		if(self.slow_vel_count==22000):
			#print "Saving to ekf.mat"
			#scipy.io.savemat('ekf',{'x': self.x_store, 'P': self.P_store})
			self.odom_sub.unregister()
			print "Save to csv"
			np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/ekf_y.csv",self.y_store,delimiter=",")
			np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/ekf_x.csv",self.x_store,delimiter=",")
			np.savetxt("/home/kyle/catkin_ws/src/ASEN_Project/bag/ekf_P.csv",self.P_store,delimiter=",")
			print "Save complete"
			rospy.signal_shtudown("Ended EKF")

def main():
	rospy.init_node('ekf', anonymous=True)
	my_ros = ros_odom_sub_pub()
	rospy.spin()

if __name__ == '__main__':	
	try:
		main()
	except rospy.ROSInterruptException:
		pass
