#!/usr/bin/env python3

import rospy
import message_filters
from nav_msgs.msg import Odometry
import time
import os
os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
from pymavlink import mavutil
# K64F_IP = '192.168.1.10' # Freedom K64F IP Address
# K64F_PORT='8150'         # Freedom K64F UPD port
# ADDRESS = 'upd:'+ K64F_IP + ':' + K64F_PORT
connection_in = mavutil.mavlink_connection('udp:0.0.0.0:8151', input=True)

def mavlink_receiver():
    pub = rospy.Publisher('ekf_data_fused', Odometry, queue_size=10)
    rospy.init_node('mavlink_manager_in', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        ekf_data_fused = connection_in.recv_match(type='ODOMETRY', blocking=True) # connection_in.messages['Odometry']
        # print("ecco sta merda di msg")
        # print(connection_in)
        ekf_data_fused_ros = Odometry()
        ekf_data_fused_ros.header.stamp = current_time
        ekf_data_fused_ros.header.frame_id = "odom"
        ekf_data_fused_ros.pose.pose.position.x = ekf_data_fused.x
        ekf_data_fused_ros.pose.pose.position.y = ekf_data_fused.y
        ekf_data_fused_ros.pose.pose.orientation.w = ekf_data_fused.q[0]
        ekf_data_fused_ros.pose.pose.orientation.x = ekf_data_fused.q[3]
        ekf_data_fused_ros.twist.twist.linear.x = ekf_data_fused.vx
        ekf_data_fused_ros.twist.twist.linear.y = ekf_data_fused.vy
        pub.publish(ekf_data_fused_ros)
        rate.sleep()

if __name__ == '__main__':
    try:
        mavlink_receiver()
    except rospy.ROSInterruptException:
        pass