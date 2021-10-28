#!/usr/bin/env python3
# adaptasi dari kode AGV_Container untuk Thesis

import rospy
import time
import sys
import numpy as np
from thesis.msg import State_Estimator
from nav_msgs.msg import Odometry

sensor_data = {
    'x_front': 0.,
    'y_front': 0., 
    'x_rear': 0., 
    'y_rear': 0.
}

#nilai x_est dan y_est disesuaikan dengan titik awal mobil di Gazebo
temp_msg = {
    'x_est': 225.,  
    'y_est': 119.,
    'v_est': 0.,
    'yaw_est': 0.,
    'yaw_gnss_fr': 0.
}

RUN = False

def to_euler(x, y, z, w):
    """Return as xyz (roll pitch yaw) Euler angles."""
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([roll, pitch, yaw])

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def Read_Data(msg):
    global sensor_data
    global temp_msg
    # global RUN_gnss_front
    sensor_data['x_front'] = msg.pose.pose.position.x
    sensor_data['y_front'] = msg.pose.pose.position.y

    delta_t = 0.2 #ms (5 Hz)
    # ganti estimasi kecepatan dari posisi menjadi data dari ego_pose (dalam m/s)
    # temp_msg['v_est'] = np.sqrt((temp_msg['x_est']-sensor_data['x_front'])**2 + (temp_msg['y_est']-sensor_data['y_front'])**2) / delta_t
    temp_msg['v_est'] = np.absolute(msg.twist.twist.linear.x)
    

    # ganti dari dy/dx jadi data dari IMU
    # temp_msg['yaw_gnss_fr'] = np.arctan2(sensor_data['y_front']-temp_msg['y_est'],sensor_data['x_front']-temp_msg['x_est'])
    yaw = to_euler(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    yaw = np.array(yaw)
    temp_msg['yaw_gnss_fr'] = wrap_angle(yaw[2])
    # temp_msg['yaw_gnss_fr'] = np.unwrap(temp_msg['yaw_gnss_fr'])
    # print(temp_msg['yaw_gnss_fr'])
    temp_msg['x_est'] = sensor_data['x_front']
    temp_msg['y_est'] = sensor_data['y_front']
    
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.yaw_gnss_fr = temp_msg['yaw_gnss_fr']
    pub_msg.x_est = temp_msg['x_est']
    pub_msg.y_est = temp_msg['y_est']
    pub_msg.v_est = temp_msg['v_est']
    pub.publish(pub_msg)

rospy.init_node('estimator')
freq = 50 # Hz

pub_msg = State_Estimator()
pub_msg.header.frame_id = 'state_estimator'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

rospy.Subscriber('/ego_pose', Odometry, Read_Data)
pub = rospy.Publisher('/state_estimator', State_Estimator, queue_size=1)
rate = rospy.Rate(freq) # Hz


print("Waiting data from GNSS...")
while not RUN:
    RUN = True
    # RUN = RUN_gnss_front and RUN_gnss_rear
    time.sleep(0.02) # 20 ms
    pass
print("Data from GNSS received.")
print("State estimator program is now running")

rospy.spin()

