# This code is written in Python 3 environment

import numpy as np
import RobustDecisionMaking.get_params as get_params
from scipy.signal import savgol_filter

class Controller(object):
    def __init__(self):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        self.name = "trajectory"
        self.veh_pos = [] #vehicle position now
        self.veh_target = [] #target position trajectory
        self.time_param = [] #parameter waktu
        self.dt_target = 0 #parameter waktu target posisi trajectory
        self.brake = [] #parameter brake dari decision making
        self.control_param = []
        # Parameter trajectory
        self.e_dx = 0
        self.e_dy = 0
        self.y_transform = 0
        self.x_transform = 0
        self.u1 = 0
        self.u2 = 0
        self.prius_wheel_radius = 0.31265
        self.v_ref = 0
        self.cs_integral = 0
        self.cs_integral_x = 0
        self.cs_integral_y = 0
        self.cs_bef = 0
        self.sum_csI = 0
        self.y_next = 0
        self.x_next = 0
        self.e_dx_dot = 0
        self.e_dy_dot = 0
        self.yaw_wp = 0
        self.t_trajectory = 0
        self._l = 0

        self._closest_idx = 0
             
    def _update_error_trajectory(self, x, y, iteration):
        # Waypoints (n, 5) -> x, y, yaw, v, t
        iteration = int(iteration)
        y2 = self.veh_target[1][iteration]
        x2 = self.veh_target[0][iteration]
        y1 = y
        x1 = x
        self.e_dx = x2-x1
        self.e_dy = y2-y1

    # def _calculate_trajectory_control_signal(self, dt, x, y, v, yaw, t):
    def _calculate_trajectory_control_signal(self, control_param, sig_x, sig_y):
        self.control_param = control_param
        params = get_params.get_params()
        final_x = params.final_x
        final_y = params.final_y
        # self._l = params.lr + params.lf  
        self._l = 1      

        # x, y, v(iteration), yaw
        self.veh_pos = [control_param[0][1], 
                        control_param[1][1],
                        control_param[2][0],
                        control_param[2][1]]
        
        # x_target, y_target
        self.veh_target = [list(sig_x), list(sig_y)]

        # self.brake = [control_param[4][0]]

        # dt (sampling time estimator), t (time now)
        self.time_param = [control_param[3][0],control_param[3][1]]
        self.dt_target = self.time_param[0]   #waktu target posisi trajectory adalah 0.5s 

        print(self.dt_target)

        vehicle_transform_x = control_param[4][0]
        vehicle_transform_y = control_param[4][1] 

        # print("t: "+str(self.dt_target))

        # gain PID trajectory
        td_1 = 0.1
        td_2 = 0.1
        kv_1 = 0.40
        kv_2 = 0.43
        ki_1 = 0
        ki_2 = 0

        # transform and calculate input state
        # self.y_transform = y + self._l*np.sin(yaw)
        # self.x_transform = x + self._l*np.cos(yaw)
        vehicle_transform_y_next = self.veh_pos[1] + self._l*np.sin(self.veh_pos[3])
        vehicle_transform_x_next = self.veh_pos[0] + self._l*np.cos(self.veh_pos[3])
        self.y_transform = self.veh_pos[1]
        self.x_transform = self.veh_pos[0]

        print(vehicle_transform_x_next, self._l*np.cos(self.veh_pos[3]))

        self._update_error_trajectory(self.x_transform,
                                      self.y_transform,
                                      self.veh_pos[2])

        iteration = int(self.veh_pos[2])
                
        # ubah persamaan (28 Agustus 2021)
        # self.u1 = td_1*(self.e_dx_dot)/dt + kv_1*self.e_dx
        # self.u2 = td_2*(self.e_dy_dot)/dt + kv_2*self.e_dy
        self.cs_integral_x = self.cs_integral_x + self.e_dx*self.dt_target
        self.cs_integral_y = self.cs_integral_y + self.e_dy*self.dt_target

        # proteksi integral windup
        # if self.cs_integral_x > 50:
        #     self.cs_integral_x = 0
        # if self.cs_integral_y > 10:
        #     self.cs_integral_y = 0

        self.u1 = td_1*(self.e_dx)/self.dt_target \
            + kv_1*(self.e_dx) + ki_1 * self.cs_integral_x
        self.u2 = td_2*(self.e_dy)/self.dt_target \
            + kv_2*(self.e_dy) + ki_2 * self.cs_integral_y
        # print("edx: "+str(self.e_dx)+" edy: "+str(self.e_dy))
        # print("intx: "+str(self.cs_integral_x)+" inty: "+str(self.cs_integral_y))

        # if self.u1 > 0:
        #     self.u1 = -self.u1

        # calculate v references and error
        self.v_ref = (self.u1*np.cos(self.veh_pos[3])+self.u2*np.sin(self.veh_pos[3]))
        # print("yaw: "+str(self.veh_pos[3])+" sin:"+str(np.sin(self.veh_pos[3]))+" cos:"+str(np.cos(self.veh_pos[3])))
        # print("control input: "+str(self.u1)+ ","+str(self.u2))

        self.v_ref = self.time_param[1]
        # self.v_ref = self.v_ref / (abs(self.e_dx)+abs(self.e_dy))
        # self.v_ref = 1

        # stop throttle if exceed reference speed
        if self.veh_pos[2] > self.v_ref:
        # if self.veh_pos[2] > 2:
            cs_long = 0
            cs_handbrake = 0.2
            # print("over speed")
        else:
            cs_long = 0.1
            cs_handbrake = 0
            # print("maju")

        if self.v_ref<0:
            cs_long = 0
            cs_handbrake = 0.1
            # print("idle")

        # if distance = 0 (deccelerate)
        if self.brake == 0: 
            cs_handbrake = 1

        # hand-brake set at final waypoints
        if self.veh_pos[0] < final_x:
            cs_long = 0
            cs_handbrake = 1
            print("finish!")

        # add trajectory lateral control
        omega = (-self.u1*np.sin(self.veh_pos[3]))/self._l + (self.u2*np.cos(self.veh_pos[3]))/self._l
        delta = np.arctan((self._l*omega)/self.v_ref)
        cs_lat = delta
        
        # omega = self.veh_pos[2]

        self.cs_bef = cs_lat

        # print("x_goal: "+str(final_x)+" y_goal: "+str(final_y))
        # print("x_now: "+str(self.veh_pos[0])+" y_now:"+str(self.veh_pos[1]))
        # print("x_target: "+str(self.veh_target[0])+" y_target:"+str(self.veh_target[1]))
        # print("v_ref: "+str(self.v_ref)+" v_now:"+str(self.veh_pos[2]))
        # print("cs_lat: "+str(cs_lat)+" cs_long:"+str(cs_long)+" cs_hb:"+str(cs_handbrake))

        x_new = self.veh_pos[0] + self.v_ref * np.cos(self.veh_pos[3]) * self.dt_target
        y_new = self.veh_pos[1] + self.v_ref * np.sin(self.veh_pos[3]) * self.dt_target
        # yaw_new = self.veh_pos[3] + omega * self.dt_target

        return x_new, y_new


print("Compiling the Controller class ...")
print("The Controller class has been compiled !")
#########################################################################################
