import matplotlib.pyplot as plt
import rospy
# import tf
# from nav_msgs.msg import Odometry
from thesis.msg import Autonomous_Game
# from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], '--')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, max(self.x_data)+5)
        self.ax.set_ylim(-7, 7)
        return self.ln
    
    # def getYaw(self, pose):
    #     quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
    #             pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     yaw = euler[2] 
    #     return yaw   

    def pos_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        self.x_data.append(msg.action_steer)
        self.y_data.append(msg.action_throttle)
        # self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # self.x_data.append(x_index+1)
    
    def update_plot(self, frame):
        self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        self.ln.set_data(self.x_data, self.y_data)
        if len(self.x_data)>500 : 
            self.x_data = list(np.array(self.x_data)[400:])
            self.y_data = list(np.array(self.y_data)[400:])
        return self.ln


rospy.init_node('visualization')
vis = Visualiser()
# sub = rospy.Subscriber('/dji_sdk/odometry', Odometry, vis.odom_callback)
sub = rospy.Subscriber('/test_visualize', Autonomous_Game, vis.pos_callback)
# rospy.spin()

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 

# while True : 
#     ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)


