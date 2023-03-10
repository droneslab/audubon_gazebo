#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import Float64
import rospy

twists = {}
poses = {}
car = "jet2"

def pose_callback(msg):
    global poses, twists
    seqID = msg.header.seq

    if seqID in twists.keys():
        odom = Odometry()
        odom.header.seq = seqID
        odom.header.stamp = msg.header.stamp
        odom.pose.pose = msg.pose
        odom.twist.twist = twists[seqID]
        odom.header.frame_id = msg.header.frame_id
        vel = (odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2) ** 0.5
        velPub.publish(vel)
        twists.pop(seqID)
        repub.publish(odom)
    else:
        poses[seqID] = msg.pose
     
def twist_callback(msg):
    global poses, twists
    seqID = msg.header.seq

    if seqID in poses.keys():
        odom = Odometry()
        odom.header.seq = seqID
        odom.header.stamp = msg.header.stamp
        odom.pose.pose = poses[seqID]
        odom.twist.twist = msg.twist
        odom.header.frame_id = msg.header.frame_id
        vel = (odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2) ** 0.5
        velPub.publish(vel)
        poses.pop(seqID)
        repub.publish(odom)
    else:
        twists[seqID] = msg.twist

if __name__ == '__main__':
    try:
        rospy.init_node('vicon_repacker', anonymous = True)
        pose_sub = rospy.Subscriber(f"/vrpn_client_node/{car}/pose", PoseStamped, pose_callback,queue_size=100)
        twist_sub = rospy.Subscriber(f"/vrpn_client_node/{car}/twist", TwistStamped, twist_callback,queue_size=100)
        repub = rospy.Publisher(f"/{car}/vicon_odom_timed", Odometry, queue_size=1)
        velPub = rospy.Publisher(f"/{car}/speed", Float64, queue_size=1)

        rospy.spin()

    except Exception as e:
            print(e)