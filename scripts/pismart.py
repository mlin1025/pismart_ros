#!/usr/bin/env python
#

import rospy
import rosnode
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from pismart.pismart import PiSmart
from pismart.motor import Motor
from pismart.led import LED 


p = PiSmart()
motorA = Motor("MotorA")
motorB = Motor("MotorB")
p.motor_switch(1)

def ledCallback(light):
	leds = LED()
	leds.brightness = light.data

def motorCallback(speed):
    
    motorRight = max(min(speed.x,100),-100)
    motorLeft = max(min(speed.y,100),-100)

    motorA.speed = int(motorRight)
    motorB.speed = int(motorLeft)

def listener():
    print "start listening"
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    nodeNameList = rosnode.get_node_names();
    createNode = False
    i = 0
    while True:
    	nameExist = False
    	for name in nodeNameList:
                print name[:9]
    		if name[:9] == '/pismart' + str(i):
    			nameExist = True
    			break

    	if not nameExist:
    		nodeName = 'pismart' + str(i)
    		break

    	i = i + 1
    

    rospy.init_node(nodeName, anonymous=False)
    rospy.Subscriber(nodeName+'_motor_speed', Vector3, motorCallback)
    rospy.Subscriber(nodeName+'_led', Int32, ledCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        motorA.stop()
        motorB.stop()
        p.motor_switch(0)
