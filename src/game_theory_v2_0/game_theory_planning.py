# Game Theory Planning
# Husnul Amri (2021)
# github : @moezeus
# adapted from RobustDecisionMaking (Gokul)

from RobustDecisionMaking import get_params, env_sim
from trajectory_gameV2_0 import Controller
import numpy as np
import time
from thesis.msg import Autonomous_Game
import rospy
# import os

# ROS thing
rospy.init_node('controller')
freq = 20 # Hz
pub = rospy.Publisher('/game_theory_AV', Autonomous_Game, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = Autonomous_Game()
pub_msg.header.frame_id = 'agv_trajectory_control_' + "game_theory"
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()


params = get_params.get_params()
game_decision = env_sim.Game_Theory_Controller_V0()
controller = Controller()

# init vehicle position 
# finish position stated on get_params
X_AV = 225.8
Y_AV = 119.9 
yaw_AV = [3.14]
V_AV = params.v_nominal

X_finish = params.final_x
Y_finish = params.final_y

# set initial tracking flag
tracking_flag = False

# roadbound coordinate
# upper bound
o1 = [[125, 121.4],
        [125, 122.4],
        [230, 122.4],
        [230, 121.4]]
# lower bound
o2 = [[125, 115],
        [125, 116],
        [230, 116],
        [230, 115]]
obs = [o1,o2]

iterate_waypoints = 0
num_waypoints = 0
sig_x = []
sig_y = []
V_ref_tr = []
dt = []
yaw_rate = []
x_trans = X_AV
y_trans = Y_AV

print("Waiting.....")
time.sleep(5) #mnunggu node untuk live plot
print("Simulation Start!")
# 1. coba dlu pake yang lama
while X_AV > X_finish:

    if tracking_flag: 
        if iterate_waypoints < num_waypoints-2: 

            # buat parameter control dalam numpy block
            control_param = np.block([[X_AV, X_AV],
                                    [Y_AV, Y_AV],
                                    [iterate_waypoints, yaw_AV[iterate_waypoints]],
                                    [dt[iterate_waypoints], V_ref_tr[iterate_waypoints]],
                                    [x_trans, y_trans]])

            # X_AV, Y_AV = controller._calculate_trajectory_control_signal(control_param, sig_x, sig_y)
            X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
            Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]

            # Store calculated value to pub_msg                                                        
            # pub_msg.action_steer = cs_lat #rad
            # pub_msg.action_throttle = cs_long # * 100 #percentage
            # pub_msg.action_brake = cs_handbrake
            pub_msg.actual_x = X_AV
            pub_msg.actual_y = Y_AV
            pub_msg.target_x = float(x_tr) #x target
            pub_msg.target_y = float(y_tr) #y target
            pub_msg.potential_x = sig_x #x trajectory
            pub_msg.potential_y = sig_y #y trajectory
            pub_msg.actual_yaw = sig_x[iterate_waypoints] #x sub sigmoid
            pub_msg.actual_speed = sig_y[iterate_waypoints] #y sub sigmoid
            pub_msg.target_brake = float(yaw_AV[iterate_waypoints])
            # pub_msg.pmap = pmap #potential map

            # Generate_waypoints = False
            print("atas")
            iterate_waypoints += 1

        else :   #waypoint terakhir
            # buat parameter control dalam numpy block
            control_param = np.block([[X_AV, X_AV],
                        [Y_AV, Y_AV],
                        [iterate_waypoints, yaw_AV[iterate_waypoints]],
                        [dt[iterate_waypoints], V_ref_tr[iterate_waypoints]],
                        [x_trans, y_trans]])

            # X_AV, Y_AV = controller._calculate_trajectory_control_signal(control_param, sig_x, sig_y)
            X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
            Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]

            # Store calculated value to pub_msg                                                        
            # pub_msg.action_steer = cs_lat #rad
            # pub_msg.action_throttle = cs_long # * 100 #percentage
            # pub_msg.action_brake = cs_handbrake
            pub_msg.actual_x = X_AV
            pub_msg.actual_y = Y_AV
            pub_msg.target_x = float(x_tr) #x target
            pub_msg.target_y = float(y_tr) #y target
            pub_msg.potential_x = sig_x #x trajectory
            pub_msg.potential_y = sig_y #y trajectory
            pub_msg.actual_yaw = sig_x[iterate_waypoints] #x sub sigmoid
            pub_msg.actual_speed = sig_y[iterate_waypoints] #y sub sigmoid
            pub_msg.target_brake = float(yaw_AV[iterate_waypoints])
            # pub_msg.pmap = pmap #potential map

            print("bawah")
            tracking_flag = False #enable flag untuk search waypoint lagi
            iterate_waypoints = 0
    
    else: 
        # create decision
        yaw_AV = yaw_AV[-1]
        x_tr, y_tr, distance, sig_x, sig_y, tracking_flag, V_ref_tr, dt, yaw_AV = game_decision._calculate_game_theory_action_V0(X_AV,
                                                        Y_AV, V_AV,yaw_AV, tracking_flag)
        num_waypoints = len(sig_x)

        if not tracking_flag: #to acomodate maintain and deccelerate action
            control_param = np.block([[X_AV, X_AV],[Y_AV, Y_AV],[iterate_waypoints, yaw_AV[iterate_waypoints]], [dt[iterate_waypoints], V_ref_tr[iterate_waypoints]],[x_trans,y_trans]])
            # X_AV, Y_AV = controller._calculate_trajectory_control_signal(control_param, sig_x, sig_y)
            X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
            Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]

            pub_msg.actual_x = X_AV
            pub_msg.actual_y = Y_AV
            pub_msg.target_x = float(x_tr) #x target
            pub_msg.target_y = float(y_tr) #y target
            pub_msg.potential_x = sig_x #x trajectory
            pub_msg.potential_y = sig_y #y trajectory
            pub_msg.actual_yaw = sig_x[iterate_waypoints] #x sub sigmoid
            pub_msg.actual_speed = sig_y[iterate_waypoints] #y sub sigmoid
            pub_msg.target_brake = float(yaw_AV[iterate_waypoints])

        else: 
            pass    
        
        print("bikin WP dlu")

    print(X_AV, Y_AV, yaw_AV[iterate_waypoints], V_ref_tr[iterate_waypoints])
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    pub.publish(pub_msg)

    time.sleep(0.1)

print("Simulation Done!")