import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd


b = bagreader('/home/lattepanda/catkin_ws/src/visual_odometry/bags/odom_bag.bag')
odom_data_file = b.odometry_data()
imu_data_file = b.message_by_topic('/imu_k64')
b.plot_odometry(save_fig=True)