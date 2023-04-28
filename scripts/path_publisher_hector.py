#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import tf
import geometry_msgs.msg


class Path_Publisher():
    def __init__(self):
        self.filter_path_publisher_ = rospy.Publisher('/path/hector_filtered', Path, queue_size = 1)
        # Added a new subscriber that subscribes to the gazebo/rviz resetting node
        self.clear_path = rospy.Subscriber('/clear_path_msg', String, self.clearpath)
        self.filter_subscription_ = rospy.Subscriber(
            '/slam_out_pose', #'/pf/pose/odom', #'/amcl_pose',
            PoseStamped,
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
        newpose.pose = msgin.pose
        self.filter_path.poses.append(newpose)
        # rospy.loginfo("publishing filter path")
        self.filter_path_publisher_.publish(self.filter_path)

    # Function that clears the published pose messages when "clear" is received
    def clearpath(self,data):
        if data.data == "clear":
            self.filter_path.poses.clear()




def odom_callback(data, tf_broadcaster):
    current_time = rospy.Time.now()

    translation = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    rotation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    
    tf_broadcaster.sendTransform(translation, rotation , current_time, "base_link" , "odom" )


def main():
    rospy.init_node('path_publisher', anonymous = True)
    republisher = Path_Publisher()

    
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber("jet2/vicon_odom", Odometry, odom_callback, tf_broadcaster)

    rospy.loginfo("Starting the Path Publisher")
    rospy.spin()

if __name__ == '__main__':
    main()