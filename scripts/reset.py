# import rospy
# from std_srvs.srv import Empty

# rospy.wait_for_service('/gazebo/reset_world')
# reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
# reset_world()

# Ctrl-C the code since pub/subs are weird when invoked once
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive


# Get the gazebo reset service
rospy.wait_for_service('/gazebo/set_model_state')

# Init node for the path publishing subscriber
rospy.init_node('clearing_node')
try:
    
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # Give "clear" string to the path publishing subscriber which clears the pose list
    clear_string_pub = rospy.Publisher('/clear_path_msg',String,queue_size=1)
    # Send ackermann message to stop stray/latched messages
    vel_pub = rospy.Publisher('/car_1/command',AckermannDrive,queue_size=1)

    while not rospy.is_shutdown():
        state_msg = ModelState()
        state_msg.model_name = 'car_1'
        state_msg.pose.position.x =  0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z =  0

        # Currently set for IMS to rotate car yaw by pi radians for HMPC testing
        # Make w 1 and z 0 for normal rotation
        state_msg.pose.orientation.w = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1

        set_state(state_msg)
        ack_msg = AckermannDrive()
        vel_pub.publish(ack_msg)
        clear_string_pub.publish("clear")


except rospy.ServiceException :
    print("Service call failed")

