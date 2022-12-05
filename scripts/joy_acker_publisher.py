#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
import sys
import math

# max_speed and max_angle are values that make the calculations normalized
# Checks for user inputted values, if none are provided, defaults to max speed: 10 m/s and max angle: 45 degrees
if(len(sys.argv)<2):
    print("Setting default max_speed:10 \t max_angle:45 degrees")
    max_speed=10
    max_angle=45
else:
    max_speed = float(sys.argv[1])
    max_angle = float(sys.argv[2])

# Speed value given to AckermannDrive message
global throttle_ms
throttle_ms = 0

# Steering Angle value given to AckermannDrive message
global steering_angle
steering_angle = 0

def callback(data):
    '''
        Callback function that subscribes to joy topic and gets sensor_msgs.msg.Joy messages\n
        Input: Joy messages from joy \n
        Computes: Desired speed and steering angles using the current inputs and the max speed and max angle \n
        Sets: Global variables throttle_ms and steering_angle for AckermannDrive message speed and steering angle values
    '''
    global throttle_ms,steering_angle,max_speed,max_angle

    # Values from joystick
    throttle_input = data.axes[5]
    reverse_input = data.axes[2]
    multiplier = data.axes[7]
    steering_input = data.axes[0]
    # max_speed+=(1*multiplier)

    # XBOX RT starts at +1 and ends at -1 ie [1,-1]
    # throttle_ms = (max_speed/2)+(max_speed/2)*((-1)*throttle_input)
    throttle_ms = (max_speed/2)*(1-throttle_input)
    
    # throttle_ms = (max_speed)*((reverse_input-throttle_input)/2)
    # print(reverse_input,throttle_input,reverse_input-throttle_input,throttle_ms)
    

    # XBOX LS is +1 at left and -1 at right
    # steering_angle = steering_input*max_angle*math.pi/180

    # # Non Linear-ish
    if(abs(steering_input)<0.5):
        steering_angle = 0.5*(steering_input)*max_angle*math.pi/180
    else:
        steering_angle = 0.5*steering_input*max_angle*math.pi/180


def talker():
    '''
    Publisher Subscriber funtion \n
    Publishes AckermannDriver messages to topic car_1/command \n
    Subscribes to Joy messages from joy node \n

    '''
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