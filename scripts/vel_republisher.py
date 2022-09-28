#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import sqrt

class Vel_Republisher:
    
    def __init__(self) -> None:
        self.repub_val = Float64()
        self.odom_world  = Odometry()

        self.vel_repub = rospy.Publisher('/car_1/ground_velocity', Float64, queue_size=10)
        self.odom_sub = rospy.Subscriber('/car_1/ground_truth', Odometry, self.odom_callback, queue_size=10)

    def odom_callback(self, odom):
        vel = odom.twist.twist.linear
        vel_odom = sqrt(vel.x**2 + vel.y**2)
        self.vel_repub.publish(vel_odom)



if __name__ == '__main__':
    try:
        rospy.init_node('vel_republisher')

        Vel_Republisher()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("vel_republisher node shutdown")    