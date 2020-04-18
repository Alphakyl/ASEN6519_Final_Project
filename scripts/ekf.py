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
#		self.imu_sub = rospy.Subscriber('/L01/imu_raw', Imu, self.imu_callback)
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

	def h(self,x,delta_t):
		y = x[0:2]
		return y		

	def prediction(self,x,u,P,delta_t,Q_t):
		x_hat_k_plus_one_minus = self.f(x,u,delta_t)
		F_tilde = self.jacobian(x,u,delta_t)
		P_k_plus_one_minus = np.dot(F_tilde,np.dot(P,F_tilde.T)) + Q_t
		return x_hat_k_plus_one_minus,P_k_plus_one_minus
		
	def correction(self,y,x,P,R):
		y_hat_k_plus_one_minus = self.h(x,delta_t)
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

	def odom_callback(self,odom):
		global initilization, x_hat_k_plus_one_plus, P_k_plus
		if(initilization)
			# establish x_0,P_0
		# establish u_k from imu_msg 
		x_hat_k_plus_one_minus,P_k_plus_one_minus = self.predict(x_hat_k_plus_one_plus,u,P_k_plus,delta_t,Q_t)
		# establish y_k+1 from odom msg
		x_hat_k_plus_one_plus,P_k_plus_one_plus = self.correct(y,x_hat_k_plu_one_minus,P_k_plus_one_minus,R_t)

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
