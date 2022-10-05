#!/usr/bin/env python3
import rospy
import math
import time
import random 
import numpy as np
import matplotlib.pyplot as plt

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

robot_x = 0.0
robot_y = 0.0
robot_th = 0.0
robot_v = 0.0
robot_w = 0.0
robot_data_save = np.empty([1,5])

def odom_callback(odom_msg) :
	global robot_x,robot_y,robot_th,robot_v,robot_w,robot_data_save
	robot_x = odom_msg.pose.pose.position.x
	robot_y = odom_msg.pose.pose.position.y
	robot_quat = [odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w]
	(roll,pitch,yaw) = euler_from_quaternion(robot_quat)
	robot_th = yaw
	robot_v = odom_msg.twist.twist.linear.x
	robot_w = odom_msg.twist.twist.angular.z
    
def flatControl(input_table):
	global robot_x,robot_y,robot_th,robot_v,robot_w,robot_data_save

	vel =AckermannDrive()
	stop_running = input_table[-1,0]
	print(stop_running)
	loop_rate = 31.0
	r = rospy.Rate(loop_rate)
	count = 0

	odom_topic = "/car_1/base/odom"
	odom_sub = rospy.Subscriber(odom_topic,Odometry,odom_callback)

	start_time = rospy.get_time()
	while not rospy.is_shutdown():
		current_time = count * 1/(float(loop_rate))
		
		v_cmd = np.interp(current_time,input_table[:,0],input_table[:,1])
		delta_cmd = np.interp(current_time,input_table[:,0],input_table[:,2])
		vel.speed =  v_cmd
		vel.steering_angle = delta_cmd
		# print (vel.steering_angle)
		vel_pub.publish(vel)
		robot_data = [[robot_x,robot_y,robot_th,robot_v,robot_w]]
		robot_data_save = np.append(robot_data_save,robot_data,axis=0)

		if count >= stop_running*loop_rate :
			print('stop')
			vel.speed=  0
			vel.steering_angle = 0
			vel_pub.publish(vel)
			robot_data = [[robot_x,robot_y,robot_th,robot_v,robot_w]]
			robot_data_save = np.append(robot_data_save,robot_data,axis=0)
			# print('DONE')
			break
		count =count+1
		r.sleep()

	end_time = rospy.get_time()

	rospy.loginfo(end_time - start_time)

if __name__ == '__main__':
	try :
		print('Kinematic Diff Flat Open Loop')
		rospy.init_node('F1gazebo_Test',anonymous = True)
		vel_topic = "/car_1/command"
		vel_pub = rospy.Publisher(vel_topic,AckermannDrive,queue_size = 10)

		data = np.loadtxt("/home/yashom/catkin_ws/src/audubon_gazebo/test/jfr_wp/IMS_size1_vmax5.txt")
		flatControl(data)
		exp_data = robot_data_save[2:len(robot_data_save[:,0]),:] 
		# with open("robot_save.txt","w") as w:
		# 	w.write(str(robot_data_save))
		# 	w.close()

		x_exp =exp_data[:,0]
		y_exp =exp_data[:,1]

		v_interp =exp_data[:,0]
		v_exp = exp_data[:,1]

		fig = plt.figure(1)	

		# plt.subplot(6,1,1)
		# plt.plot(texp,x_exp,'r')

		# plt.subplot(6,1,2)
		# plt.plot(texp,y_exp,'r')

		
		# plt.subplot(6,1,3)
		# plt.plot(texp,th_exp,'r')

		# plt.subplot(6,1,4)
		# plt.plot(texp,v_exp,'r')

		# plt.subplot(6,1,5)
		# plt.plot(texp,w_exp,'r')

		# plt.subplot(6,1,6)
		plt.plot(x_exp,y_exp,'r')

		plt.show()
	except rospy.ROSInterruptException :
		rospy.loginfo ("Node Terminated")