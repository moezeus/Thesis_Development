# This code is written in Python 3 environment

import numpy as np
from scipy.signal import savgol_filter

class Controller(object):
    def __init__(self, kp, ki, kd, feed_forward_params, sat_long,
                 ks, kv, length, lateral_dead_band, sat_lat,
                 waypoints, min_vel_move,
                 max_throttle_move, min_throttle_move, kv_yaw, kv_lat):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        self.name = "trajectory"

        # The parameters of the longitudinal controller
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._ff_params = feed_forward_params
        self._sat_long_max = max(sat_long[0], sat_long[1])
        self._sat_long_min = min(sat_long[0], sat_long[1])
        self._ev = 0.
        self._ev_last = 0.0
        self._ev_sum = 0.
        self._ev_sum_max = 0.  # This value will be updated in each iteration
        self._ev_sum_min = 0.  # This value will be updated in each iteration

        # The parameters of the lateral controller
        self._ks = ks
        self._kv = kv
        self._l = length
        self._dead_band_lat = lateral_dead_band
        self._sat_lat_max = np.fmax(sat_lat[0], sat_lat[1])
        self._sat_lat_min = np.fmin(sat_lat[0], sat_lat[1])
        self._e_lat = 0.
        self._e_yaw = 0.

        # Parameter trajectory
        self.e_dx = 0
        self.e_dy = 0
        self.y_transform = 0
        self.x_transform = 0
        self.u1 = 0
        self.u2 = 0
        self.prius_wheel_radius = 0.31265
        self.v_ref = 0
        self.yaw_trajectory = np.arctan2(waypoints[1,1]-waypoints[0,1],waypoints[1,0]-waypoints[0,0])
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

        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        # Waypoints (6 param) -> x,y,yaw,v,curvature,t
        self._waypoints = waypoints
        self._closest_idx = 0

        # INTERSECTING WP PATCH
        wp_loop = 0  # todo for looping through sam
        self.inter_wp_min_dist = 0.10  # in meter
        self.bounded_wp_dist_front = 5  # in meter
        self.bounded_wp_dist_rear = 3  # in meter
        self.wp_loop = wp_loop

        self.bounded_wp_s_idx, self._idx_wp, self.bounded_wp_e_idx = 0, 0, (len(
            self._waypoints)-1)
        if self.wp_loop == 0:  # TODO: looping waypoint (wp_loop > 0)
            self.bounded_wp_s_idx = self._idx_wp - \
                int(self.bounded_wp_dist_rear/self.inter_wp_min_dist)
            self.bounded_wp_s_idx = 0 if self.bounded_wp_s_idx < 0 else self.bounded_wp_s_idx
            self.bounded_wp_e_idx = self._idx_wp + \
                int(self.bounded_wp_dist_front/self.inter_wp_min_dist)
            self.bounded_wp_e_idx = (len(self._waypoints)-1) if self.bounded_wp_e_idx > (
                len(self._waypoints)-1) else self.bounded_wp_e_idx
        self.loc_waypoints = self._waypoints[self.bounded_wp_s_idx:self.bounded_wp_e_idx]
        ##########

        # Additional Throttle Control (UNTUK TA)
        self._max_throttle = max_throttle_move
        self._min_throttle = min_throttle_move
        self._min_vel_move = min_vel_move
        self._kv_yaw = kv_yaw
        self._kv_lat = kv_lat

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def reset_integral_derivative(self):
        self._ev_sum = 0.0
        self._ev_last = 0.0

    def get_error(self):
        return self._ev, self._e_lat, self._e_yaw

    def get_closest_index(self):
        return self._closest_idx

    def get_instantaneous_setpoint(self):
        out = np.copy(self._waypoints[self._closest_idx])
        if out[3] <= 0.05:
            None
        else:
            out[3] = np.fmax(out[3] / (1.0 + self._kv_lat*self._e_lat **
                                       2 + self._kv_yaw*self._e_yaw**2), self._min_vel_move)
        return out
    
    def _update_error_trajectory(self, x, y, v, yaw, t, dt):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature, t

        self._closest_idx = np.argmin(
            np.square(self._waypoints[:, 5] - t))

        # Find the lateral or crosstrack error
        if self._closest_idx == 0:
            idx = 1
        else:
            idx = self._closest_idx
        y2 = self._waypoints[idx, 1]
        x2 = self._waypoints[idx, 0]
        y_next = self._waypoints[idx + 1, 1]
        x_next = self._waypoints[idx + 1, 0]
        y1 = y
        x1 = x
        self.e_dx_dot = y_next-y2
        self.e_dy_dot = x_next-x2
        self.e_dx = x2-x1
        self.e_dy = y2-y1
        self.yaw_wp = self._waypoints[idx, 2]
        self.t_trajectory = self._waypoints[idx, 5]
        # print(self._closest_idx, self.e_dx, self.e_dy)

    def _feed_forward_longitudinal(self, v):
        if v < 0.:
            return 0.
        else:
            return self._ff_params[0] * (1. - np.exp(- self._ff_params[1] * v))

    def _feed_forward_lateral(self, yaw):
        temp = self._l * self._waypoints[self._closest_idx, 4]
        # temp = self._l * yaw
        if np.abs(temp) >= 1.:
            temp = np.sign(temp)
        # From -pi/2 to pi/2
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max), self._sat_lat_min)

    def _calculate_trajectory_control_signal(self, dt, x, y, v, yaw, t):
        # gain PID trajectory
        td_1 = 0
        td_2 = 0
        kv_1 = 3
        kv_2 = 10
        ki_1 = 0.1
        ki_2 = 0.8

        # transform and calculate input state
        self.y_transform = y + self._l*np.sin(yaw)
        self.x_transform = x + self._l*np.cos(yaw)

        self._update_error_trajectory(self.x_transform, self.y_transform, v, yaw, t, dt)
                
        # ubah persamaan (28 Agustus 2021)
        # self.u1 = td_1*(self.e_dx_dot)/dt + kv_1*self.e_dx
        # self.u2 = td_2*(self.e_dy_dot)/dt + kv_2*self.e_dy
        self.cs_integral_x = self.cs_integral_x + self.e_dx*dt
        self.cs_integral_y = self.cs_integral_y + self.e_dy*dt

        self.u1 = td_1*(self.e_dx)/dt + kv_1*self.e_dx + ki_1 * self.cs_integral_x
        self.u2 = td_2*(self.e_dy)/dt + kv_2*self.e_dy + ki_2 * self.cs_integral_y

        # calculate v references and error
        self.v_ref = (self.u1*np.cos(yaw)+self.u2*np.sin(yaw))

        # stop throttle if exceed reference speed
        if v > self.v_ref:
            cs_long = 0
            cs_handbrake = 1
        else:
            cs_long = 1
            cs_handbrake = 0

        if self.v_ref<0:
            cs_long = 0
            cs_handbrake = 0

        # hand-brake set at final waypoints
        stopping_dist = 0.1
        wp_interpoint_dist = 0.01
        if len(self._waypoints) - self._closest_idx < int(stopping_dist/wp_interpoint_dist):
            cs_long = 0
            cs_handbrake = 1

        # add trajectory lateral control
        omega = (-self.u1*np.sin(yaw))/self._l + (self.u2*np.cos(yaw))/self._l
        delta = np.arctan((self._l*omega)/self.v_ref)
        cs_lat = delta
        
        self.cs_bef = cs_lat

        print(v, self.v_ref, self.e_dx, self.e_dy)

        return cs_long, cs_lat, cs_handbrake


print("Compiling the Controller class ...")
print("The Controller class has been compiled !")
#########################################################################################
