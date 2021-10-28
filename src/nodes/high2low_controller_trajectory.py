#!/usr/bin/env python3

import rospy
import time
import sys
import os
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from thesis.msg import State_Estimator
from trajectory import Controller

# state ditambahkan trajectory
state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0., 't':0.,}

def callbackStateEstimator(msg):
    global state
    global RECEIVED_STATE_VAL
    state['x'] = msg.x_est
    state['y'] = msg.y_est
    state['yaw'] = msg.yaw_gnss_fr
    state['v'] = msg.v_est
    RECEIVED_STATE_VAL = True

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


rospy.init_node('controller')
freq = 20 # Hz

RECEIVED_STATE_VAL = False

rospy.Subscriber('/state_estimator', State_Estimator, callbackStateEstimator)
pub = rospy.Publisher('/control_signal', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz

freq = rospy.get_param('~freq', 20.) # Hz
ff_1 = rospy.get_param('~ff_1', 0.0)
ff_2 = rospy.get_param('~ff_2', 0.0)
kp = rospy.get_param('~kp', 0.11)
ki = rospy.get_param('~ki', 0.30)
kd = rospy.get_param('~kd', 0.015)
sat_long_max = rospy.get_param('~sat_long_max', 0.3)
sat_long_min = rospy.get_param('~sat_long_min', -2.9)
kv_yaw = rospy.get_param('~kv_yaw', 2.25)
kv_lat = rospy.get_param('~kv_lat', 0.75)
min_vel_move = rospy.get_param('~min_vel_move', 0.5)
max_throttle_move = rospy.get_param('~max_throttle_move', 0.3)
min_throttle_move = rospy.get_param('~min_throttle_move', 0.3)
length = rospy.get_param('~length', 1.7)
ks = rospy.get_param('~ks', 0.75)
kv = rospy.get_param('~kv', 1.00)
lateral_dead_band = rospy.get_param('~lateral_dead_band', 0.025)
sat_lat_max = rospy.get_param('~sat_lat_max', 0.6109)
sat_lat_min = rospy.get_param('~sat_lat_min', -0.4887)
waypoints_path = rospy.get_param('~waypoints_path', 'test_try.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../waypoints/' + waypoints_path)   #ceck posisi file waypoint
print("waypoints file:",waypoints_path)

feed_forward_params = np.array([ff_1, ff_2])
sat_long = np.array([sat_long_min, sat_long_max])
sat_lat = np.array([sat_lat_min, sat_lat_max])
waypoints = np.load(waypoints_path)
# TEMP: override v setpoint
for i in range(len(waypoints)):
    waypoints[i,3] = 1 # speed in m/s

# trajectory - acuan waktu awal 
time_zero = rospy.get_time()

# Create the controller object
controller = Controller(kp, ki, kd, feed_forward_params, sat_long,\
                        ks, kv, length, lateral_dead_band, sat_lat,\
                        waypoints, min_vel_move,\
                        max_throttle_move, min_throttle_move, kv_yaw, kv_lat)
print("Controller name:", controller.name)

pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'agv_path_following_control_' + controller.name
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
last_time = pub_msg.header.stamp.to_sec() - 1./freq
last_time = rospy.Time.now().to_sec() - 1./freq

while not rospy.is_shutdown():
    ### Calculate the actual sampling time
    pub_msg.header.stamp = rospy.Time.now()
    delta_t = pub_msg.header.stamp.to_sec() - last_time
    last_time = pub_msg.header.stamp.to_sec()

    ### Send the message
    # Header
    pub_msg.header.seq += 1

    # relative time with trajectory waypoints
    state['t'] = rospy.get_time() - time_zero

    if (RECEIVED_STATE_VAL):

        ### Calculate the control signal
        cs_long, cs_lat, cs_handbrake, x_target, y_target = controller._calculate_trajectory_control_signal(delta_t, state['x'],
                                                        state['y'], state['v'],
                                                        state['yaw'], state['t'])
        # Store calculated value to pub_msg                                                        
        pub_msg.pose.covariance[0] = cs_lat #rad
        pub_msg.pose.covariance[1] = cs_long # * 100 #percentage
        pub_msg.pose.covariance[2] = cs_handbrake
        pub_msg.pose.covariance[4] = state['x']
        pub_msg.pose.covariance[5] = state['y']
        pub_msg.pose.covariance[6] = x_target #x target
        pub_msg.pose.covariance[7] = y_target #y target
        pub_msg.pose.covariance[9] = controller._waypoints[controller._closest_idx, 3]    #speed setpoint
        pub_msg.pose.covariance[11] = 100 if (cs_long == 0) else 0  #brake_regen
    else:
        print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        controller._update_error_trajectory(state['x'], state['y'], state['v'], state['yaw'], state['t'],delta_t)

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
