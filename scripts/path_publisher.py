#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

class Path_Publisher():
    def __init__(self):
        self.filter_path_publisher_ = rospy.Publisher('/path/filtered', Path, queue_size = 1)
        # Added a new subscriber that subscribes to the gazebo/rviz resetting node
        self.clear_service = rospy.Service('clear_path', SetBool, self.clearpath)
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

    # Function that clears the published pose messages when "clear" is received
    def clearpath(self, req):
        if req.data == 1:
            try:
                self.filter_path.poses.clear()
                return SetBoolResponse(True, "Success")
            except Exception as e:
                return SetBoolResponse(False, "Could not clear path")
        
        
    

def main():
    rospy.init_node('path_publisher', anonymous = True)
    republisher = Path_Publisher()

    rospy.loginfo("Starting the Path Publisher")
    rospy.spin()

if __name__ == '__main__':
    main()