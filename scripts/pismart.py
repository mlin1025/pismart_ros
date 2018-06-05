#!/usr/bin/env python
#

import rospy
from geometry_msgs.msg import Twist
from pismart.pismart import PiSmart
from pismart.motor import Motor
from pismart.led import LED 


p = PiSmart()
motorA = Motor("MotorA")
motorB = Motor("MotorB")
p.motor_switch(1)

def callback(twistData):
    leds = LED();
    motorRight = 60 * twistData.linear.x + 40 * twistData.linear.y
    motorLeft = 60 * twistData.linear.x - 40 * twistData.linear.y

    print motorRight
    print motorLeft

    motorA.speed = motorRight
    motorB.speed = motorLeft

    if motorRight or motorLeft:
        leds.brightness = 100
    else:
        leds.brightness = 0

def listener():
    print "start listening"
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pismart', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        motorA.stop()
        motorB.stop()
        p.motor_switch(0)
