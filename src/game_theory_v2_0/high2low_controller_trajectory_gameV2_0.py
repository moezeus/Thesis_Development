#!/usr/bin/env python3
from __future__ import division
import rospy
import time
import sys
import os
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from thesis.msg import State_Estimator, Autonomous_Game
from RobustDecisionMaking.get_params import get_params
from RobustDecisionMaking.env_sim import Game_Theory_Controller_V0
from trajectory_gameV2_0 import Controller
# untuk menulis csv
import csv

# state ditambahkan trajectory
state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0., 't':0.,}

params = get_params()

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
pub = rospy.Publisher('/control_signal_game', Autonomous_Game, queue_size=1)
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
# waypoints_path = rospy.get_param('~waypoints_path', 'test_try.npy')
# waypoints_path = os.path.abspath(sys.path[0] + '/../waypoints/' + waypoints_path)   #ceck posisi file waypoint
# print("waypoints file:",waypoints_path)

feed_forward_params = np.array([ff_1, ff_2])
sat_long = np.array([sat_long_min, sat_long_max])
sat_lat = np.array([sat_lat_min, sat_lat_max])
# waypoints = np.load(waypoints_path)
# TEMP: override v setpoint
# for i in range(len(waypoints)):
#     waypoints[i,3] = 1 # speed in m/s

# trajectory - acuan waktu awal 
time_zero = rospy.get_time()

# Create the controller object
controller = Controller()
print("Controller name:", controller.name)

pub_msg = Autonomous_Game()
pub_msg.header.frame_id = 'agv_trajectory_control_' + "game_theory"
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
last_time = pub_msg.header.stamp.to_sec() - 1./freq
last_time = rospy.Time.now().to_sec() - 1./freq

Game_Theory_Controller = Game_Theory_Controller_V0()

num_waypoints = 0               #waypoints number initialization
Generate_waypoints = True       #initialize flag to generate waypoints using game theory
iterate_waypoints = 0           #watpoints iteration number
duration_tr = 0                 #trajectory duration
x_tr = 0
y_tr = 0

pmap = []
dt = []

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
        # cs_long, cs_lat, cs_handbrake = controller._calculate_trajectory_control_signal(delta_t, state['x'],
        #                                                 state['y'], state['v'],
        #                                                 state['yaw'], state['t'])

        if Generate_waypoints: 
            x_tr, y_tr, distance, sig_x, sig_y, dt, V_curv = Game_Theory_Controller._calculate_game_theory_action_V0(delta_t, state['x'],
                                                        state['y'], state['v'],
                                                        state['yaw'], state['t'])
        # update csv pmap
        # https://stackoverflow.com/questions/67981176/python-write-array-output-in-csv-file-for-each-for-loop-iteration
        # with open('pmap.csv', 'a') as f:
        #     writer = csv.writer(f, delimiter=',')
        #     writer.writerows(pmap)
        # print(type(float(x_tr)), type(float(y_tr)), type(state['x']))
        # print(type(np.array(pmap)))
        num_waypoints = len(sig_x)
        duration_tr = (np.sum(dt)/num_waypoints)
        duration_tr = duration_tr + 0.5*duration_tr
        # print(duration_tr, num_waypoints)

        if iterate_waypoints < num_waypoints-1: 
                
            # buat parameter control dalam numpy block
            control_param = np.block([[sig_x[iterate_waypoints], state['x']],
                                    [sig_y[iterate_waypoints], state['y']],
                                    [state['v'], state['yaw']],
                                    [V_curv[iterate_waypoints], dt[iterate_waypoints]], 
                                    [distance,distance]])

            cs_long, cs_lat, cs_handbrake = controller._calculate_trajectory_control_signal(control_param)

            # Store calculated value to pub_msg                                                        
            pub_msg.action_steer = cs_lat #rad
            pub_msg.action_throttle = cs_long # * 100 #percentage
            pub_msg.action_brake = cs_handbrake
            pub_msg.actual_x = state['x']
            pub_msg.actual_y = state['y']
            pub_msg.target_x = float(x_tr) #x target
            pub_msg.target_y = float(y_tr) #y target
            pub_msg.potential_x = sig_x #x trajectory
            pub_msg.potential_y = sig_y #y trajectory
            pub_msg.actual_yaw = sig_x[iterate_waypoints] #x sub sigmoid
            pub_msg.actual_speed = sig_y[iterate_waypoints] #y sub sigmoid
            # pub_msg.pmap = pmap #potential map

            Generate_waypoints = False
            iterate_waypoints += 1

        else :   #waypoint terakhir
            # buat parameter control dalam numpy block
            control_param = np.block([[sig_x[iterate_waypoints], state['x']],
                                    [sig_y[iterate_waypoints], state['y']],
                                    [state['v'], state['yaw']],
                                    [V_curv[iterate_waypoints], dt[iterate_waypoints]], 
                                    [distance,distance]])

            cs_long, cs_lat, cs_handbrake = controller._calculate_trajectory_control_signal(control_param)

            # Store calculated value to pub_msg                                                        
            pub_msg.action_steer = cs_lat #rad
            pub_msg.action_throttle = cs_long # * 100 #percentage
            pub_msg.action_brake = cs_handbrake
            pub_msg.actual_x = state['x']
            pub_msg.actual_y = state['y']
            pub_msg.target_x = float(x_tr) #x target
            pub_msg.target_y = float(y_tr) #y target
            pub_msg.potential_x = sig_x #x trajectory
            pub_msg.potential_y = sig_y #y trajectory
            pub_msg.actual_yaw = sig_x[iterate_waypoints] #x sub sigmoid
            pub_msg.actual_speed = sig_y[iterate_waypoints] #y sub sigmoid
            # pub_msg.pmap = pmap #potential map

            Generate_waypoints = True #enable flag untuk search waypoint lagi
            iterate_waypoints = 0

    else:
        print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        # controller._update_error_trajectory(state['x'], state['y'], state['v'], state['yaw'], state['t'],delta_t)

    # Publish the message
    pub.publish(pub_msg)
    print("xsub: "+str(x_tr)+" ysub: "+str(y_tr))
    # print("num: "+str(num_waypoints), " iter: "+str(iterate_waypoints))
    print("total time: "+ str(sum(dt)))
    time.sleep(duration_tr)

    ### Wait until the next loop
    rate.sleep()
