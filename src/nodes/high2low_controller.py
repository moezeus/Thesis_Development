#!/usr/bin/env python3

import rospy
import time
import sys
import os
import numpy as np
# from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from thesis.msg import State_Estimator
# import paho.mqtt.client as mqtt
# sys.path.append(os.path.abspath(sys.path[0] + '/../src/python'))
#  Pemilihan kontroller
from stanley_2d import Controller
# from trajectory import Controller
# from lyapunov_rl import Controller

#---------------------- MQTT -------------------------------#
# DEBUG_MQTT_SUB_MSG = False
# mqtt_autonomous = 0
# mqtt_autonomous_bef = mqtt_autonomous
# mqtt_steer = 0
# mqtt_throttle = 0
# mqtt_brake = 0
# mqtt_locker = 0
# mqtt_speed_control_mode = 0
# mqtt_speed_setpoint = 0
# mqtt_brake_regen = 0

# def on_mqtt_message(client, userdata, message):
#     global mqtt_autonomous
#     global mqtt_steer
#     global mqtt_throttle
#     global mqtt_brake
#     global mqtt_locker
#     global mqtt_brake_regen

#     if DEBUG_MQTT_SUB_MSG:
#         print(rospy.Time.now(), "MQTT received", message.topic, (message.payload))
#     # print("Topic=", message.topic, "QoS", message.qos, "Retain", message.retain)

#     if message.topic == "control/autonomous":
#         mqtt_autonomous = int(message.payload)
#     if message.topic == "control/steer":
#         mqtt_steer = float(message.payload)
#     if message.topic == "control/throttle":
#         mqtt_throttle = float(message.payload)
#     if message.topic == "control/brake":
#         mqtt_brake = float(message.payload)
#     if message.topic == "control/locker":
#         mqtt_locker = int(message.payload)
#     if message.topic == "control/brake_regen":
#         mqtt_brake_regen = float(message.payload)

#-------------------------------------------------------------#

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}
# state ditambahkan trajectory

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
# rospy.Subscriber('/ukf_states', ukf_states, callback)
# lyap_pub = rospy.Publisher('/lyap_gain', Float32, queue_size=1)
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
# waypoints_path = '2021-02-16-12-20-13.npy'
waypoints_path = os.path.abspath(sys.path[0] + '/../waypoints/' + waypoints_path)   #ceck posisi file waypoint
print("waypoints file:",waypoints_path)

feed_forward_params = np.array([ff_1, ff_2])
sat_long = np.array([sat_long_min, sat_long_max])
sat_lat = np.array([sat_lat_min, sat_lat_max])
waypoints = np.load(waypoints_path)
# TEMP: override v setpoint
for i in range(len(waypoints)):
    waypoints[i,3] = 1 # speed in m/s

#-------------- MQTT Client setup -------------#
# print("Creating new MQTT client ...")
# client = mqtt.Client("ros_control_node")
# client.on_message = on_mqtt_message

# print("Connecting to broker ...")
# broker_address = "localhost"
# client.connect(host="localhost", port=1883)

# print("Subscribing to topics ...")
# client.subscribe("control/autonomous")
# client.subscribe("control/steer")
# client.subscribe("control/throttle")
# client.subscribe("control/brake")
# client.subscribe("control/locker")

# # topic = "test/publish/topic"
# # print("Publishing message to topic",topic)
# # client.publish(topic,"OFF")

# client.loop_start()
#-----------------------------------------------#


# Create the controller object
controller = Controller(kp, ki, kd, feed_forward_params, sat_long,\
                        ks, kv, length, lateral_dead_band, sat_lat,\
                        waypoints, min_vel_move,\
                        max_throttle_move, min_throttle_move, kv_yaw, kv_lat)
print("Controller name:", controller.name)
# print("Input mode: MANUAL (not AUTONOMOUS)")

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

    # Control action
    # if (mqtt_autonomous == 1):
    #     if (mqtt_autonomous is not mqtt_autonomous_bef):
    #             print("[INFO] [", rospy.Time.now(), "] Input mode changed to: AUTONOMOUS")
    if (RECEIVED_STATE_VAL):
        # debug message
        # if (mqtt_autonomous is not mqtt_autonomous_bef):
        print("[INFO] [", rospy.Time.now(), "] State Variable has been received. Starting autonomous mode.")

        ### Calculate the control signal
        cs_long, cs_lat, cs_handbrake = controller.calculate_control_signal(delta_t, state['x'],
                                                        state['y'], state['v'],
                                                        state['yaw'])
        # Store calculated value to pub_msg                                                        
        pub_msg.pose.covariance[0] = cs_lat #rad
        # pub_msg.pose.covariance[0] = np.interp(cs_lat,[-28,35],[-1,1]) #convert to gazebo max-min steer
        pub_msg.pose.covariance[1] = cs_long # * 100 #percentage
        pub_msg.pose.covariance[2] = cs_handbrake
        # pub_msg.pose.covariance[3] = mqtt_locker
        pub_msg.pose.covariance[4] = controller._e_lat
        pub_msg.pose.covariance[5] = controller._e_yaw
        pub_msg.pose.covariance[6] = controller._waypoints[controller._closest_idx, 0] #x target
        pub_msg.pose.covariance[7] = controller._waypoints[controller._closest_idx, 1] #y target
        # pub_msg.pose.covariance[8] = mqtt_speed_control_mode
        pub_msg.pose.covariance[9] = controller._waypoints[controller._closest_idx, 3]    #speed setpoint
        # pub_msg.pose.covariance[10] = mqtt_autonomous
        pub_msg.pose.covariance[11] = 100 if (cs_long == 0) else 0  #brake_regen
    else:
        # if (mqtt_autonomous is not mqtt_autonomous_bef):
        print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")

        controller._update_error(state['x'], state['y'], state['v'], state['yaw'])

        # pub_msg.pose.covariance[0] = mqtt_steer
        # pub_msg.pose.covariance[1] = mqtt_throttle
        # pub_msg.pose.covariance[2] = mqtt_brake
        # pub_msg.pose.covariance[3] = mqtt_locker
        # pub_msg.pose.covariance[4] = controller._e_lat
        # pub_msg.pose.covariance[5] = controller._e_yaw
        # pub_msg.pose.covariance[6] = controller._waypoints[controller._closest_idx, 0]
        # pub_msg.pose.covariance[7] = controller._waypoints[controller._closest_idx, 1]
        # pub_msg.pose.covariance[8] = mqtt_speed_control_mode
        # pub_msg.pose.covariance[9] = mqtt_speed_setpoint
        # pub_msg.pose.covariance[10] = mqtt_autonomous
        # pub_msg.pose.covariance[11] = mqtt_brake_regen
        # print("flag")

    # mqtt_autonomous_bef = mqtt_autonomous

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()

