# Game Theory Planning
# Husnul Amri (2021)
# github : @moezeus
# adapted from RobustDecisionMaking (Gokul)

from RobustDecisionMaking import get_params, env_sim, path_generation
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
# game_decision = env_sim._calculate_game_theory_action_V0()
# controller = Controller()

# init vehicle position 
# finish position stated on get_params
X_AV = 225.8
Y_AV = 119.9 
yaw_AV = [3.14]
yaw_pub = 0
V_AV = [params.v_nominal]
X_finish = params.final_x
Y_finish = params.final_y

# other vehicles
X_oth = 225.8
Y_oth = 116.6 
yaw_oth = 3.14
V_oth = params.v_nom_others
X_finish_oth = params.final_x
Y_finish_oth = 116.6 
others_case = 0 # 0 for cooperative, 1 for aggresive

# set initial tracking and return flag
tracking_flag = False
return_flag = False

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
iterate_return = 0
num_waypoints = 0
sig_x = []
sig_y = []
V_ref_tr = []
dt = []
yaw_rate = []
x_trans = X_AV
y_trans = Y_AV
traj_tracking = [[],[],[]]

other_L0 = 0
other_L1 = 0

print("Waiting.....")
time.sleep(5) #mnunggu node untuk live plot
print("Simulation Start!")
# 1. coba dlu pake yang lama

# buat fungsi untuk update vehicle state block
def veh_state_update(yaw_AV, V_AV, iteration): 
    yaw_AV = yaw_AV[iteration]
    # if V_AV[-1] == 0:
    V_AV = params.v_nominal
    # else: 
    #     V_AV = V_AV[-1]
    x = [X_AV, X_oth]
    y = [Y_AV, Y_oth]
    yaw = [yaw_AV, yaw_oth]
    v = [V_AV, V_oth]
    AV_cars = [1,0]
    final_x = [params.final_x, X_finish_oth]
    final_y = [params.final_y, Y_finish_oth]
    final_yaw = [params.final_yaw, params.final_yaw]
    initial_state = np.block([x, y, yaw, v, AV_cars, final_x, final_y,final_yaw])

    return initial_state


while X_AV > X_finish:
    if tracking_flag and not return_flag: 
        if iterate_waypoints < num_waypoints-2: 
            # game theory decision making
            # update dt for decision 
            iteration = iterate_waypoints
            veh_state = veh_state_update(yaw_AV, V_AV, iterate_waypoints)
            Action_AV, return_flag, other_L0, other_L1, return_sig, traj_return = env_sim._calculate_game_theory_action_V0(veh_state, sig_x, sig_y, iteration, traj_tracking)
            
            if Action_AV == 0:  #maintain
                # update AV position
                X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
                Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
                V_AV = [V_ref_tr[iterate_waypoints]]
                yaw_pub = yaw_AV[iterate_waypoints]
                
                # update other vehicle position
                dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
                if others_case:     #egoistic
                    if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_max * 1.5      #accelerate
                    else : 
                        if V_oth == params.v_max * 1.5:     #keep the speed
                            V_oth = V_oth
                        else : 
                            V_oth = params.v_nom_others
                else: #cooperative
                    if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_nom_others*0.5       #deccelerate
                    else : 
                        V_oth = params.v_nom_others

                X_oth = X_oth + V_oth * np.cos(yaw_oth) * dt[iterate_waypoints]
                Y_oth = Y_oth
                V_oth = V_oth

                iterate_waypoints += 1
            elif Action_AV == 1: #decelerate
                X_AV = X_AV
                Y_AV = Y_AV
                V_AV = [0]
                yaw_pub = yaw_AV[iterate_waypoints]

                # update other vehicle position
                dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
                if others_case:     #egoistic
                    if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_max * 1.5      #accelerate
                    else : 
                        if V_oth == params.v_max * 1.5:     #keep the speed
                            V_oth = V_oth
                        else : 
                            V_oth = params.v_nom_others
                else: #cooperative
                    if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_nom_others*0.5       #deccelerate
                    else : 
                        V_oth = params.v_nom_others 

                X_oth = X_oth + V_oth * np.cos(yaw_oth) * dt[iterate_waypoints]
                Y_oth = Y_oth
                V_oth = V_oth

            elif Action_AV == 2: #return to lane before
                tracking_flag = False #go track the new waypoints
                num_return = len(return_sig[0])
                # print("Here!")
                # print(num_return)
                iterate_waypoints = 0
                sig_x = return_sig[0]
                sig_y = return_sig[1]
                yaw_AV = []
            
            # Generate_waypoints = False
            # print("atas")
            # iterate_waypoints += 1

        else :   #waypoint terakhir
            iteration = iterate_waypoints
            veh_state = veh_state_update(yaw_AV, V_AV, iterate_waypoints)
            Action_AV, return_flag, other_L0, other_L1, return_sig, traj_return = env_sim._calculate_game_theory_action_V0(veh_state, sig_x, sig_y, iteration, traj_tracking)
            
            if Action_AV == 0:  #maintain
                # update AV position
                X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
                Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
                V_AV = [V_ref_tr[iterate_waypoints]]
                yaw_pub = yaw_AV[iterate_waypoints]

                # update other vehicle position
                dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
                if others_case:     #egoistic
                    if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_max * 1.5      #accelerate
                    else : 
                        if V_oth == params.v_max * 1.5:     #keep the speed
                            V_oth = V_oth
                        else : 
                            V_oth = params.v_nom_others
                else: #cooperative
                    if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_nom_others*0.5       #deccelerate
                    else : 
                        V_oth = params.v_nom_others

                X_oth = X_oth + V_oth * np.cos(yaw_oth) * dt[iterate_waypoints]
                Y_oth = Y_oth
                V_oth = V_oth

                tracking_flag = False #enable flag untuk search waypoint lagi
                iterate_waypoints = 0
            elif Action_AV == 1: #decelerate
                X_AV = X_AV
                Y_AV = Y_AV
                V_AV = [0]
                yaw_pub = yaw_AV[iterate_waypoints]

                # update other vehicle position
                dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
                if others_case:     #egoistic
                    if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_max * 1.5      #accelerate
                    else : 
                        if V_oth == params.v_max * 1.5:     #keep the speed
                            V_oth = V_oth
                        else : 
                            V_oth = params.v_nom_others
                else: #cooperative
                    if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                        V_oth = params.v_nom_others*0.5       #deccelerate
                    else : 
                        V_oth = params.v_nom_others 

                X_oth = X_oth + V_oth * np.cos(yaw_oth) * dt[iterate_waypoints]
                Y_oth = Y_oth
                V_oth = V_oth
                
            elif Action_AV == 2: #return to lane before
                tracking_flag = False #go track the new waypoints
                num_return = len(return_sig[0])
                yaw_AV = []
                iterate_waypoints = 0
                # print("Here!")
                # print(num_return)
                sig_x = return_sig[0]
                sig_y = return_sig[1]

            # print("bawah")
            params.lane_bef = Y_AV

    elif return_flag and not tracking_flag: 
        # Action_AV, return_flag, other_L0, other_L1, return_sig, traj_return
        # traj return = V_ref_tr, dt, yaw_AV
        # print("Here!")
        # print(traj_return)
        if iterate_return < num_return - 2: 
            X_AV = X_AV + traj_return[0][iterate_return] * np.cos(traj_return[2][iterate_return]) * traj_return[1][iterate_return]
            Y_AV = Y_AV + traj_return[0][iterate_return] * np.sin(traj_return[2][iterate_return]) * traj_return[1][iterate_return]
            V_AV = [traj_return[0][iterate_return]]

            # update other vehicle position
            dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
            if others_case:     #egoistic
                if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_max * 1.5      #accelerate
                else : 
                    if V_oth == params.v_max * 1.5:     #keep the speed
                        V_oth = V_oth
                    else : 
                        V_oth = params.v_nom_others
            else: #cooperative
                if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_nom_others*0.5       #deccelerate
                else : 
                    V_oth = params.v_nom_others 

            time_other = (traj_return[0][iterate_return]*traj_return[1][iterate_return])/V_oth
            X_oth = X_oth + V_oth * np.cos(yaw_oth) * time_other
            Y_oth = Y_oth
            V_oth = V_oth
            yaw_pub = traj_return[2][iterate_return]


            iterate_return += 1
        else: 
            X_AV = X_AV + traj_return[0][iterate_return] * np.cos(traj_return[2][iterate_return]) * traj_return[1][iterate_return]
            Y_AV = Y_AV + traj_return[0][iterate_return] * np.sin(traj_return[2][iterate_return]) * traj_return[1][iterate_return]
            V_AV = [V_ref_tr[iterate_return]]

            # update other vehicle position
            dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
            if others_case:     #egoistic
                if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_max * 1.5      #accelerate
                else : 
                    if V_oth == params.v_max * 1.5:     #keep the speed
                        V_oth = V_oth
                    else : 
                        V_oth = params.v_nom_others
            else: #cooperative
                if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_nom_others*0.5       #deccelerate
                else : 
                    V_oth = params.v_nom_others  

            time_other = (traj_return[0][iterate_return]*traj_return[1][iterate_return])/V_oth
            X_oth = X_oth + V_oth * np.cos(yaw_oth) * time_other
            Y_oth = Y_oth
            V_oth = V_oth

            iterate_return = 0
            yaw_pub = traj_return[2][iterate_return]
            yaw_AV = [traj_return[2][iterate_return]]
            return_flag = False
            
    elif not tracking_flag and not return_flag: 
        # generate optimal path 
        initial_state = veh_state_update(yaw_AV, V_AV, iterate_waypoints) 
        x_tr, y_tr, sig_x, sig_y, tracking_flag, V_ref_tr, dt, yaw_AV = path_generation._path_generation(initial_state)
        params.lane_bef = Y_AV
        # x_tr, y_tr, distance, sig_x, sig_y, tracking_flag, V_ref_tr, dt, yaw_AV = game_decision._calculate_game_theory_action_V0(X_AV,
        #                                                 Y_AV, V_AV,yaw_AV, tracking_flag)
        num_waypoints = len(sig_x)
        traj_tracking = [V_ref_tr, dt, yaw_AV]
        yaw_pub = yaw_AV[iterate_waypoints]

        if not tracking_flag: #to acomodate maintain and deccelerate action
            # control_param = np.block([[X_AV, X_AV],[Y_AV, Y_AV],[iterate_waypoints, yaw_AV[iterate_waypoints]], [dt[iterate_waypoints], V_ref_tr[iterate_waypoints]],[x_trans,y_trans]])
            # X_AV, Y_AV = controller._calculate_trajectory_control_signal(control_param, sig_x, sig_y)
            # update AV position
            X_AV = X_AV + V_ref_tr[iterate_waypoints] * np.cos(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
            Y_AV = Y_AV + V_ref_tr[iterate_waypoints] * np.sin(yaw_AV[iterate_waypoints]) * dt[iterate_waypoints]
            V_AV = [V_ref_tr[iterate_waypoints]]

            # update other vehicle position
            dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
            if others_case:     #egoistic
                if X_oth>X_AV and dist_AV_others < 8 and dist_AV_others > 2 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_max * 1.5      #accelerate
                else : 
                    if V_oth == params.v_max * 1.5:     #keep the speed
                        V_oth = V_oth
                    else : 
                        V_oth = params.v_nom_others
            else: #cooperative
                if X_oth>X_AV and dist_AV_others < 8 and Y_AV < 119.9 and Y_AV > 116.6: #AV try to overtaking
                    V_oth = params.v_nom_others*0.5       #deccelerate
                else : 
                    V_oth = params.v_nom_others               

            X_oth = X_oth + V_oth * np.cos(yaw_oth) * 0.1
            Y_oth = Y_oth
            V_oth = V_oth
        else:
            # iterate_waypoints += 1
            pass
        # print("bikin WP dlu")

    # print(X_AV, Y_AV, params.lane_bef, params.t_step_DT, other_L0, other_L1)
    # print("other")
    # print(X_oth, Y_oth, X_AV, Y_AV)
    dist_AV_others = np.sqrt((X_AV-X_oth)**2+(Y_AV-Y_oth)**2)
    print("dist")
    print(dist_AV_others)

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
    pub_msg.target_brake = float(yaw_pub)
    pub_msg.error_lateral = X_oth
    pub_msg.error_yaw = Y_oth
    pub_msg.error_speed = yaw_oth
    # pub_msg.pmap = pmap #potential map
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    pub.publish(pub_msg)

    time.sleep(0.1)

print("Simulation Done!")