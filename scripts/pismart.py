#!/usr/bin/env python
#

import rospy
import rosnode
from geometry_msgs.msg import Vector3
from pismart.pismart import PiSmart
from pismart.motor import Motor
from pismart.led import LED 


p = PiSmart()
motorA = Motor("MotorA")
motorB = Motor("MotorB")
p.motor_switch(1)

def ledCallback(light):
	leds = LED()
	leds.brightness = light

def motorCallback(speed):
    
    motorRight = max(min(speed.x,100),-100)
    motorLeft = max(min(speed.y,100),-100)

    print motorRight
    print motorLeft

    motorA.speed = motorRight
    motorB.speed = motorLeft

def listener():
    print "start listening"
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    nodeNameList = rosnode.get_node_names();

    createNode = False
    i = 0;
    while True:
    	pass
    	nameExist = False
    	for name in nodeNameList:
    		if name == 'pismart' + str(i):
    			nameExist = True
    			break

    	if not nameExist:
    		nodeName = 'pismart' + str(i)
    		break

    	i = i + 1
    

    rospy.init_node(nodeName, anonymous=True)
    rospy.Subscriber(nodeName+'_motor_speed', Vector3, motorCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        motorA.stop()
        motorB.stop()
        p.motor_switch(0)
