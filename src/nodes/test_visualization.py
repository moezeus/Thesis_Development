import rospy
import numpy as np
from thesis.msg import Autonomous_Game

rospy.init_node('data_source')
freq = 20 # Hz

pub = rospy.Publisher('/test_visualize', Autonomous_Game, queue_size=1)
rate = rospy.Rate(freq) # Hz

pub_msg = Autonomous_Game()
pub_msg.header.frame_id = "test plot"
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

x = 0

print("its running bro") 

while not rospy.is_shutdown():
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    
    y = np.cos(x)

    pub_msg.action_steer = x #rad
    pub_msg.action_throttle = y #rad

    x += 0.01

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
