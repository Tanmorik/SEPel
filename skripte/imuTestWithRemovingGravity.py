#!/usr/bin/env python3
import rospy
from rospy import sleep
from std_msgs.msg import UInt8
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from time import time
import matplotlib.pyplot as plt
import numpy as np
import math

from tf.transformations import euler_from_quaternion 

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                    x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                    x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], 
                    dtype=np.float64)

def quaternion_conjugate(quaternion):
    w0, x0, y0, z0 = quaternion
    return np.array([w0, -x0, -y0, -z0], dtype=np.float64)

def quaternion_inverse(quaternion):
    w0, x0, y0, z0 = quaternion
    return np.array([-w0, x0, y0, z0], dtype=np.float64)



class evaluate_velocity():
    def __init__(self):
        #rospy.init_node("imuoriantation")

        delta = 0

        self.imudata = Imu()

        self.imuaccx = []
        self.imuaccy = []
        self.imuaccz = [] #TODO imu vel auf den starwert der Odometry legen
        self.imuoriroll = []
        self.imuoripitch = []
        self.imuoriyaw = [] #TODO imu vel auf den starwert der Odometry legen

        rospy.Subscriber("/imu/data",Imu, self.imu_callback)

        while not rospy.is_shutdown() and delta < 1:
            rospy.sleep(1)
            delta+= 1

        xmean = (np.mean(self.imuaccx))
        ymean = (np.mean(self.imuaccy))
        zmean = (np.mean(self.imuaccz))
        orirollmean = (np.mean(self.imuoriroll))
        oripitchmean = (np.mean(self.imuoripitch))
        oriyawmean = (np.mean(self.imuoriyaw))
        betrag = np.linalg.norm([xmean, ymean, zmean])

        print("acc mean:", xmean, ymean, zmean)
        print("roll mean:", orirollmean,"pitch mean", oripitchmean,"yaw mean:", oriyawmean)
        print("Betrag:",betrag)
        #print(math.atan2(xmean, zmean)*180/math.pi)
        rospy.signal_shutdown("reason")


#http://wiki.ros.org/tf2/Tutorials/Quaternions
    def imu_callback(self, msg):
        self.imudata = msg
        quaternion = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z],
            dtype=np.float64
            )
        quaterniontest = msg.orientation
        print(quaterniontest)
        accquaternion = np.array([
            0,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z],
            dtype=np.float64
        )
        #Dreheung auf Erdkoordinatensystem
        v_0 = quaternion_multiply(
            quaternion_multiply(
                quaternion, accquaternion), 
                quaternion_conjugate(quaternion))
        
        #print("Gedreht:",v_2,"Ursprung:", accquaternion)

        #print(msg.orientation.x)
        euler = euler_from_quaternion(quaternion)

        
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        #print(roll)
        #print("Acc X:", msg.linear_acceleration.x,"Acc Y:", msg.linear_acceleration.y,"Acc Z:", msg.linear_acceleration.z)
        #print("AccRot X:",v_0[1],                    "AccRot Y:",v_0[2],                    "AccRot Z:",v_0[3])
        g = np.array([0, 0, 0, 9.81])
        v_1 = v_0 - g
        v_2 = quaternion_multiply(
            quaternion_multiply(
                quaternion_inverse(quaternion), v_1), 
                quaternion_inverse(quaternion_conjugate(
                    quaternion)))
        #print(v_1)

        self.imuoriroll.append(roll)
        self.imuoripitch.append(pitch)
        self.imuoriyaw.append(yaw)

        self.imuaccx.append(v_1[0])
        self.imuaccy.append(v_1[1])
        self.imuaccz.append(v_1[2])
            

if __name__ == "__main__":
    rospy.init_node("imuoriantation")
    evaluate_velocity()
    #euler = euler_from_quaternion((0,0,0.7072,0.7072))
    #print(euler)
    """
    p = [0, 1, 0, 0]
    r = [0.707, 0, 0.707, 0]
    rcon = quaternion_conjugate(r)
    print(rcon)
    first = quaternion_multiply(r,p)
    print(first)
    second = quaternion_multiply(first, rcon)
    print(second)
    """
