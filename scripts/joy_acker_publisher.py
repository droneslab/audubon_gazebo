import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
import sys
import math

if(len(sys.argv)<2):
    print("Setting default max_speed:10 \t max_angle:45 degrees")
    max_speed=10
    max_angle=45
else:
    max_speed = float(sys.argv[1])
    max_angle = float(sys.argv[2])

# Actual value of the speed
global throttle_ms
throttle_ms = 0

# Actual value of the steering angle
global steering_angle
steering_angle = 0

def callback(data):
    global throttle_ms,steering_angle,max_speed,max_angle

    # Values from joystick
    throttle_input = data.axes[5]
    steering_input = data.axes[0]

    # XBOX RT starts at +1 and ends at -1 ie [1,-1]
    throttle_ms = (max_speed/2)*(1-throttle_input)
    # throttle_ms = (max_speed/2)+(max_speed/2)*((-1)*throttle_input)

    # XBOX LS is +1 at left and -1 at right
    # steering_angle = steering_input*max_angle*math.pi/180

    # # Non Linear-ish
    if(abs(steering_input)<0.5):
        steering_angle = 0.9*(steering_input)*max_angle*math.pi/180
    else:
        steering_angle = steering_input*max_angle*math.pi/180


def talker():
    global throttle_ms,steering_angle
    ack_pub = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=10)
    rospy.init_node('joy_ack_publisher', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ack_msg = AckermannDrive()
        ack_msg.speed = throttle_ms
        ack_msg.steering_angle = steering_angle
        ack_pub.publish(ack_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass