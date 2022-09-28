#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
class Path_Publisher():
    def __init__(self):
        self.filter_path_publisher_ = rospy.Publisher('/path/filtered', Path, queue_size = 1)
        
        self.filter_subscription_ = rospy.Subscriber(
            '/car_1/base/odom',
            Odometry,
            self.filter_listener_callback)
        self.filter_subscription_  #prevent unused variable warning
        self.filter_path = Path()


        self.filter_last_called = 0

    def filter_listener_callback(self, msgin):
        # only add new msgs to the path msg at the allowed rate 
        now = rospy.get_time()
        if now - self.filter_last_called < 0.1:
            return
        self.filter_last_called = now 
        # rospy.loginfo("received filtered odometry")
        newpose = PoseStamped()
        self.filter_path.header = msgin.header
        newpose.header = msgin.header
        #newpose.header.frame_id = "base_link"
        newpose.pose = msgin.pose.pose
        self.filter_path.poses.append(newpose)
        # rospy.loginfo("publishing filter path")
        self.filter_path_publisher_.publish(self.filter_path)
    

def main():
    rospy.init_node('path_publisher', anonymous = True)
    republisher = Path_Publisher()

    rospy.loginfo("Starting the Path Publisher")
    rospy.spin()

if __name__ == '__main__':
    main()