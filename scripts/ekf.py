#!/usr/bin/python2

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import math

# Global Variables
initilization = True

class EKF:
	def __init__(self):
		global initialization
		self.odom_sub = rospy.Subscriber('cart_odom', Odometry, self.odom_callback)
		self.filted_output = rospy.Subscriber('filter_out', Odometry, self.odom_callback)
			
	def jacobian(self,x,u,delta_t):
		s = np.sqrt([x[3]^2+x[4]^2])
		F_tilde = np.identity(5)
		A_tilde = np.zeros(5)
		A_tilde[0,2] = -s*math.sin(x[2])
		A_tilde[0,2] = s*math.cos(x[2])
		if(s > 0):
			A_tilde[1,4] = x[4]*cos(x[3])/s
			A_tilde[1,5] = x[5]*cos(x[3])/s
			A_tilde[2,4] = x[4]*sin(x[3])/s
			A_tilde[2,5] = x[5]*sin(x[3])/s
		
		F_tilde = np.identity(5)+A_tilde*delta_t
		return F_tilde

	def f(self,x,u,delta_t):
		s = np.srqt([x[3]^2+x[4]^2])
		x_hat_k_plus_one_minus = np.zeros(shape(x))
		x_hat_k_plus_one_minus[0] = x[0] + s*cos(x[2])*delta_t
		x_hat_k_plus_one_minus[1] = x[1] + s*sin(x[2])*delta_t
		x_hat_k_plus_one_minus[2] = x[2] + u[0]*delta_t
		x_hat_k_plus_one_minus[2] = np.unwrap(x_hat_k_plus_one_minus[2])
		x_hat_k_plus_one_minus[3] = x[3] + u[1]*delta_t
		x_hat_k_plus_one_minus[4] = x[4] + u[2]*delta_t
		return x_hat_k_plus_one_minus

	def h(self,x,delta_t, vel):
		if vel:
			y = x
		else:
			y = x[0:2]
		return y		

	def prediction(self,x,u,P,delta_t,Q_t):
		x_hat_k_plus_one_minus = self.f(x,u,delta_t)
		F_tilde = self.jacobian(x,u,delta_t)
		P_k_plus_one_minus = np.dot(F_tilde,np.dot(P,F_tilde.T)) + Q_t
		return x_hat_k_plus_one_minus,P_k_plus_one_minus
		
	def correction(self,y,x,P,R):
		if y.size() == 5:
			vel = 1
		else:
			vel = 0
		y_hat_k_plus_one_minus = self.h(x,delta_t,vel)
		if vel == 1:
			H = np.identity(5)
		else:
			H = np.array([[1., 0., 0., 0., 0.],
						[0., 1., 0., 0., 0.],
						[0., 0., 1., 0., 0.]])
		e_y_k_plus_one = y-y_hat_k_plus_one_minus
		temp_1 = np.dot(P,H)
		temp_2 = np.dot(H,temp_1)+R
		temp_3 = np.linalg.inv(temp_2)
		K = np.dot(temp_1,temp_3)
		x_hat_k_plus_one_plus = x+np.dot(K,e_y_k_plus_one)
		temp_4 = np.dot(K,H)
		temp_5 = np.identity(temp_4.shape)-temp_4
		P_k_plus_one_plus = np.dot(temp_5,P)
		return x_hat_k_plus_one_plus,P_k_plus_one_plus

	def odom_callback(self,odom_msg):
		global initilization, slow_vel_count
		if(initilization)
			initialization = false
			# Establish x_0,P_0
			self.x_hat_k_plus_one_plus
			self.P_k_plus_one_plus

		# Establish u_k from imu_msg 
		imu_msg = rospy.wait_for_mesage("/L01/imu_raw", Imu)
		u = np.empty([3,1])
		u[0][0] = imu_msg.angular_velocity.z
		u[1][0] = imu_msg.linear_acceleration.x
		u[2][0] = imu_msg.linear_acceleration.y

		# Predict new x_hat
		x_hat_k_plus_one_minus,P_k_plus_one_minus = self.predict(self.x_hat_k_plus_one_plus,u,self.P_k_plus_one_plus,delta_t,Q_t)
		
		# Establish y_k+1 from odom msg
		y = np.empty([3,1])
		y[0][0] = odom_msg.pose.pose.position.x
		y[1][0] = odom_msg.pose.pose.position.y
		orientation_quat = odom.pose.pose.orientation
		orientation_euler = tf.transformations.euler_form_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
		y[2][0] = orientation_euler[2]

		# Can possibly create fake velocities here?
		if(slow_vel_count % 100):
			# Find Current values
			slow_time_curr = odom_msg.header.stamp
			x_curr = y[0]
			y_curr = y[1]
			slow_time_delta = slow_time_curr - slow_time_prev

			# Find velocitie estimates
			x_vel = (x_curr - x_prev)/slow_time_delta
			y_vel = (y_curr - y_prev)/slow_time_delta
			y_new = np.empty([5,1])
			y_new[0] = y[0]
			y_new[1] = y[1]
			y_new[2] = y[2]
			y_new[3] = x_vel
			y_new[4] = y_vel
			# Set previous values to current values
			x_prev = x_curr
			y_prev = y_curr
			slow_time_prev = slow_time_prev
			y = y_new

		# Given measurements, correct x_hat estimate
		self.x_hat_k_plus_one_plus,self.P_k_plus_one_plus = self.correct(y,x_hat_k_plus_one_minus,P_k_plus_one_minus,R_t)

#	def imu_callback(self,imu):
#		l = 1

def main():
	rospy.init_node('ekf', anonomyous=True)
	my_filter = EKF()
	rospy.spin()

if __name__ == '__main__':	
	try:
		main()
	except rospy.ROSInterruptException:
		pass
