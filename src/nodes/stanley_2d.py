# This code is written in Python 3 environment

import numpy as np
# from numba import njit, float64, int64
# from numba.experimental import jitclass

# Stanley Controller  with Feed Forward Term
# spec = [('_kp', float64), ('_ki', float64), ('_kd', float64), ('_ff_params', float64[:]),
#         ('_sat_long_max', float64), ('_sat_long_min', float64),
#         ('_ev', float64), ('_ev_last', float64), ('_ev_sum', float64),
#         ('_ev_sum_max', float64), ('_ev_sum_min', float64),
#         ('_ks', float64), ('_kv', float64), ('_l', float64),
#         ('_dead_band_lat', float64), ('_sat_lat_max', float64),
#         ('_sat_lat_min', float64), ('_e_lat', float64), ('_e_yaw', float64),
#         ('_waypoints', float64[:, :]), ('_closest_idx', int64),
#         ('_min_vel_move', float64), ('_max_throttle', float64), ('_min_throttle', float64),
#         ('_kv_yaw', float64), ('_kv_lat', float64),
#         ("inter_wp_min_dist"), ("bounded_wp_dist_front", float64), ("bounded_wp_dist_rear", float64),
#         ("wp_loop", float64), ("bounded_wp_s_idx", float64),
#         ("_idx_wp", float64), ("bounded_wp_e_idx", float64), ("loc_waypoints", float64[:, :])]

# @jitclass(spec)
class Controller(object):
    def __init__(self, kp, ki, kd, feed_forward_params, sat_long,\
                 ks, kv, length, lateral_dead_band, sat_lat,\
                 waypoints, min_vel_move,\
                 max_throttle_move, min_throttle_move, kv_yaw, kv_lat):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        self.name = "stanley"

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
        self._ev_sum_max = 0. # This value will be updated in each iteration
        self._ev_sum_min = 0. # This value will be updated in each iteration

        # The parameters of the lateral controller
        self._ks = ks
        self._kv = kv
        self._l = length
        self._dead_band_lat = lateral_dead_band
        self._sat_lat_max = np.fmax(sat_lat[0], sat_lat[1])
        self._sat_lat_min = np.fmin(sat_lat[0], sat_lat[1])
        self._e_lat = 0.
        self._e_yaw = 0.

        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._waypoints = waypoints
        self._closest_idx = 0

        ##### INTERSECTING WP PATCH
        wp_loop = 0 #todo for looping through sam
        self.inter_wp_min_dist = 0.10 # in meter
        self.bounded_wp_dist_front = 5 # in meter
        self.bounded_wp_dist_rear = 3 # in meter
        self.wp_loop = wp_loop

        self.bounded_wp_s_idx, self._idx_wp, self.bounded_wp_e_idx = 0, 0, (len(self._waypoints)-1)
        if self.wp_loop == 0 : #TODO: looping waypoint (wp_loop > 0)
            self.bounded_wp_s_idx = self._idx_wp - int(self.bounded_wp_dist_rear/self.inter_wp_min_dist)
            self.bounded_wp_s_idx = 0 if self.bounded_wp_s_idx < 0 else self.bounded_wp_s_idx
            self.bounded_wp_e_idx = self._idx_wp + int(self.bounded_wp_dist_front/self.inter_wp_min_dist)
            self.bounded_wp_e_idx = (len(self._waypoints)-1) if self.bounded_wp_e_idx > (len(self._waypoints)-1) else self.bounded_wp_e_idx
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
            out[3] = np.fmax(out[3] / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2), self._min_vel_move)
        return out

    def _update_error(self, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature

        # Find the closest waypoint
        # self._closest_idx = np.argmin(np.sum(np.square(self._waypoints[:, :2] - np.array([x, y])), axis=-1))
        loc_closest_idx = np.argmin(np.sum(np.square(self.loc_waypoints[:, :2] - np.array([x, y])), axis=-1))
        self._closest_idx = loc_closest_idx + 1 + self.bounded_wp_s_idx

        # Find the yaw error
        self._e_yaw = self._waypoints[self._closest_idx, 2] - yaw
        self._e_yaw = (self._e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)

        # Find the lateral or crosstrack error
        if self._closest_idx == 0:
            idx = 1
        else:
            idx = self._closest_idx
        y2 = self._waypoints[idx, 1]
        x2 = self._waypoints[idx, 0]
        y1 = self._waypoints[idx - 1, 1]
        x1 = self._waypoints[idx - 1, 0]
        dy = y2 - y1
        dx = x2 - x1
        c = dx*y1 - dy*x1
        self._e_lat = (dy*x + c - dx*y) \
                        / (np.sqrt(dx**2 + dy**2) + 10**(-32))

        # Find the velocity error
        ## vAdam
        temp = self._waypoints[self._closest_idx, 3]
        if temp <= 0.1:
            None
        else:
            temp = temp / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
            temp = np.fmax(temp, self._min_vel_move)
        self._ev = temp - v

        ## vD override
        # temp = 0.9
        # self._ev = temp - v

        ## vD
        temp = self._waypoints[self._closest_idx, 3]
        temp = temp / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
        self._ev = temp - v

        #### LOOPING WP PATCH
        # min_idx = loc_closest_idx
        # self._idx_wp = min_idx + 1 + self.bounded_wp_s_idx
        self._idx_wp = self._closest_idx
        if self.wp_loop == 0 : #TODO: looping waypoint (wp_loop > 0)
            self.bounded_wp_s_idx = self._idx_wp - int(self.bounded_wp_dist_rear/self.inter_wp_min_dist)
            self.bounded_wp_s_idx = 0 if self.bounded_wp_s_idx < 0 else self.bounded_wp_s_idx
            self.bounded_wp_e_idx = self._idx_wp + int(self.bounded_wp_dist_front/self.inter_wp_min_dist)
            self.bounded_wp_e_idx = (len(self._waypoints)-1) if self.bounded_wp_e_idx > (len(self._waypoints)-1) else self.bounded_wp_e_idx
        self.loc_waypoints = self._waypoints[self.bounded_wp_s_idx:self.bounded_wp_e_idx]
        ####

    def _feed_forward_longitudinal(self, v):
        if v < 0.:
            return 0.
        else:
            return self._ff_params[0] * (1. - np.exp(- self._ff_params[1] * v))

    def _feed_forward_lateral(self):
        temp = self._l * self._waypoints[self._closest_idx, 4]
        if np.abs(temp) >= 1.:
            temp = np.sign(temp)
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max), self._sat_lat_min) # From -pi/2 to pi/2

    # def _feed_forward_lateral(self):
    #     temp = self._l * self._waypoints[self._closest_idx, -1]
    #     return np.fmax(np.fmin(np.arctan(temp), self._sat_lat_max), self._sat_lat_min) # From -pi/2 to pi/2

    def calculate_control_signal(self, dt, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._update_error(x, y, v, yaw)

        # Longitudinal control

        # PID Control
        ff_long = self._feed_forward_longitudinal(self._waypoints[self._closest_idx, 3])

        ev_dot = (self._ev - self._ev_last) / dt

        self._ev_sum_max = np.fmax((self._sat_long_max - ff_long)/self._ki, 0.)
        self._ev_sum_min = np.fmin((self._sat_long_min - ff_long)/self._ki, 0.)
        self._ev_sum = self._ev_sum + self._ev * dt
        self._ev_sum = np.fmax(np.fmin(self._ev_sum, self._ev_sum_max), self._ev_sum_min)

        # cs_long = ff_long +\
        #             self._kp * self._ev +\
        #             self._ki * self._ev_sum +\
        #             self._kd * ev_dot
        # cs_long = np.fmax(np.fmin(cs_long, self._sat_long_max), self._sat_long_min)
        temp_kp, temp_ki, temp_kd = 2, 0.1, 0.
        cs_long = temp_kp*self._ev + temp_ki*self._ev_sum + temp_kd*ev_dot
        cs_long = np.fmax(np.fmin(cs_long, 0.1), 0.)#1.), 0.)

        self._ev_last = self._ev

        # # Throttle Control
        # cs_long = self._waypoints[self._closest_idx, 3]
        # if cs_long <= 0.05:
        #     None
        # else:
        #     cs_long = cs_long / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
        #     cs_long = np.fmin(np.fmax(cs_long, self._min_throttle), self._max_throttle)

        ### vD1
        # _throttle_max_cvtr_0 = 0.1
        # _k_cvtr = 3.
        # _throttle_max_cvtr_t = _throttle_max_cvtr_0 / (1 + _k_cvtr*(self._waypoints[self._closest_idx, 4]**2))
        # cs_long = _throttle_max_cvtr_t / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2)
        # cs_long = np.fmin(np.fmax(cs_long, self._min_throttle), self._max_throttle)

        stopping_dist = 2.
        wp_interpoint_dist = 0.01
        cs_handbrake = 0
        if len(self._waypoints) - self._closest_idx < int(stopping_dist/wp_interpoint_dist):
            cs_long = 0
            cs_handbrake = 100

        # Lateral Control

        # STANLEY Controller
        # v = cs_long#*5
        # v = 1
        temp = 0.0
        self._dead_band_lat = 0.
        if np.abs(self._e_lat) > self._dead_band_lat:
            temp = self._e_lat

        a = self._feed_forward_lateral()
        b = self._e_yaw
        self._ks = 0.8
        c = np.arctan(self._ks * temp / (self._kv + v))
        d = a + b + c # Use Stanley !

        cs_lat = max(min(d, self._sat_lat_max), self._sat_lat_min)

        # add trajectory 
        

        # debug
        # print(self._e_lat, cs_long)

        return cs_long, cs_lat, cs_handbrake

print("Compiling the Controller class ...")
# controller = Controller(0.5, 0.1, 0.1, np.array([1., 2.]),
#                         np.array([-1., 1.]), 2.0, 0.1, 2.5,
#                         0.01, np.array([-np.pi/3., np.pi/3.]),
#                         np.random.randn(100, 5), 0.5,
#                         0.2, 0.08, 0.75, 0.75)
# controller.update_waypoints(np.random.randn(100, 5))
# controller.reset_integral_derivative()
# _ = controller.get_error()
# _ = controller.get_instantaneous_setpoint()
# # controller._update_error(0., 0., 1.0, 0.)
# # _ = controller._feed_forward_longitudinal(2.5)
# # _ = controller._feed_forward_lateral()
# _ = controller.calculate_control_signal(0.01, 0., 0., 1.0, 0.)
print("The Controller class has been compiled !")
#########################################################################################
