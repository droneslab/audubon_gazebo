#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Path_Publisher():
    def __init__(self):
        self.filter_path_publisher_ = rospy.Publisher('/path/filtered', Path, queue_size = 1)
        # Added a new subscriber that subscribes to the gazebo/rviz resetting node
        self.clear_path = rospy.Subscriber('/clear_path_msg', String, self.clearpath)
        self.filter_subscription_ = rospy.Subscriber(
            'jet2/vicon_odom',
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

    # Function that clears the published pose messages when "clear" is received
    def clearpath(self,data):
        if data.data == "clear":
            self.filter_path.poses.clear()
    

def main():
    rospy.init_node('path_publisher', anonymous = True)
    republisher = Path_Publisher()

    rospy.loginfo("Starting the Path Publisher")
    rospy.spin()

if __name__ == '__main__':
    main()