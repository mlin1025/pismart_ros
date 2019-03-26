#!/usr/bin/env python
#

import rospy
import rosnode
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from pismart.pismart import PiSmart
from pismart.motor import Motor
from pismart.led import LED
import threading
from mpu6050 import mpu6050


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
    print time

def imuThread():
    pub = rospy.Publisher(rospy.get_name()[1:] + "_imu", Imu, queue_size=1)
    sensor = mpu6050(0x68)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        imuData = Imu()
        angular = sensor.get_gyro_data()
        #accel = sensor.get_accel_data()

        imuData.header.stamp = rospy.Time.now()

        imuData.angular_velocity.x = angular['x']
        imuData.angular_velocity.y = angular['y']
        imuData.angular_velocity.z = angular['z']

        #imuData.linear_acceleration.x = accel['x']
        #imuData.linear_acceleration.y = accel['y']
        #imuData.linear_acceleration.z = accel['z']
        pub.publish(imuData)
        r.sleep()

        

def listener():
    #nodeNameList = rosnode.get_node_names();
    #createNode = False
    #i = 0
    #while True:
    #    nameExist = False
    #    for name in nodeNameList:
    #        if name[:9] == '/pismart' + str(i):
    #            print name[:9]
    #            nameExist = True
    #            break

    #    if not nameExist:
    #        nodeName = 'pismart' + str(i)
    #        print nodeName
    #        break

    #    i = i + 1
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
        nodeName = 'pismart0'
        print nodeName
        rospy.init_node(nodeName, anonymous=False)
        rospy.Subscriber(nodeName+'_motor_speed', Vector3, motorCallback)
        rospy.Subscriber(nodeName+'_led', Int32, ledCallback)
        print "start listening"

        t = threading.Thread(target=imuThread)
        t.start()
        listener()
        t.join()
    except KeyboardInterrupt:
        motorA.speed = 0
        motorB.speed = 0
        motorA.stop()
        motorB.stop()
        p.motor_switch(0)
