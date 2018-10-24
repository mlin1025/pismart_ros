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
global time

def ledCallback(light):
    leds = LED()
    leds.brightness = light.data

def motorCallback(speed):
    
    motorRight = max(min(speed.x,100),-100)
    motorLeft = max(min(speed.y,100),-100)

    motorA.speed = int(motorRight)
    motorB.speed = int(motorLeft)
    global time
    time = rospy.Time.now()

def listener():
    nodeNameList = rosnode.get_node_names();
    createNode = False
    i = 0
    while True:
        nameExist = False
        for name in nodeNameList:
            if name[:9] == '/pismart' + str(i):
                print name[:9]
                nameExist = True
                break

        if not nameExist:
            nodeName = 'pismart' + str(i)
            print nodeName
            break

        i = i + 1
    

    rospy.init_node(nodeName, anonymous=False)
    rospy.Subscriber(nodeName+'_motor_speed', Vector3, motorCallback)
    rospy.Subscriber(nodeName+'_led', Int32, ledCallback)
    print "start listening"
    global time
    time = rospy.Time.now()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (rospy.Time.now() - time).to_sec() > 0.5:
            motorA.speed = 0
            motorB.speed = 0
        r.sleep()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        motorA.stop()
        motorB.stop()
        p.motor_switch(0)
