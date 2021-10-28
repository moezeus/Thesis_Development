#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from thesis.msg import laser_scan_data 
import time
import math

RUN = False

angle = []
ranges = []

def callback(msg):
    global angle
    global ranges

    angle = []
    ranges = []

    # print(len(msg.ranges),msg.angle_min,msg.angle_max,msg.angle_increment)
    # print("something")
    for i in range(len(msg.ranges)):
        # sudut = np.degree
        if not math.isinf(msg.ranges[i]):
            angle.append(np.degrees(msg.angle_min+i*msg.angle_increment))
            ranges.append(msg.ranges[i])
    # angle = msg.angle_min
    # ranges = msg.ranges[0]

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.angle = angle
    pub_msg.ranges = ranges
    pub.publish(pub_msg)
    # print("something again")
    # exit()


rospy.init_node('laser_scan_data')

freq = 50 # Hz

pub_msg = laser_scan_data()
pub_msg.header.frame_id = 'laser_scan_data'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

rospy.Subscriber('/prius/front_middle_laser/scan', LaserScan, callback)

pub = rospy.Publisher('/filtered_data', laser_scan_data, queue_size=1)
rate = rospy.Rate(freq) # Hz

# client.loop_start()

print("Waiting data from LiDAR...")
while not RUN:
    RUN = True
    # RUN = RUN_gnss_front and RUN_gnss_rear
    time.sleep(0.02) # 20 ms
    pass
print("Data from LiDAR received.")
print("Laser_Scan_Data_Running")

rospy.spin()