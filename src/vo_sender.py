#!/usr/bin/env python3

import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import rosbag
import bagpy
from bagpy import bagreader
import os
os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
from pymavlink import mavutil
# K64F_IP = '192.168.1.10' # Freedom K64F IP Address
# K64F_PORT='8150'         # Freedom K64F UPD port
# ADDRESS = 'upd:'+ K64F_IP + ':' + K64F_PORT
connection = mavutil.mavlink_connection('udpout:192.168.1.10:8150')
# connection_out = mavutil.mavlink_connection('udpin:192.168.1.10:8151')

# odom_bag = rosbag.Bag('~/Desktop/odom.bag', 'w')

def mavlink_send(odom_sub):#, imu_sub):#,plan_sub):
    ### POSE DATA
    # Preparing odometry mavlink message
    time_usec = 0 
    frame_id = 0
    child_frame_id = 0
    # Position
    x = odom_sub.pose.pose.position.x
    y = odom_sub.pose.pose.position.y
    z = odom_sub.pose.pose.position.z
    # Quaternion
    q = [odom_sub.pose.pose.orientation.w,odom_sub.pose.pose.orientation.x,odom_sub.pose.pose.orientation.y,odom_sub.pose.pose.orientation.z]
    # Linear velocity
    vx = odom_sub.twist.twist.linear.x
    vy = odom_sub.twist.twist.linear.y
    vz = odom_sub.twist.twist.linear.z
    # Angular velocity
    rollspeed = odom_sub.twist.twist.angular.x
    pitchspeed = odom_sub.twist.twist.angular.y
    yawspeed = odom_sub.twist.twist.angular.z
    # The following fields are not used
    pose_covariance = []
    velocity_covariance = []
    for i in range(21):
        pose_covariance.append(0)
        velocity_covariance.append(0)
    reset_counter = 0
    estimator_type = 0
    # Send odometry mavlink message
    connection.mav.odometry_send(time_usec,\
                                frame_id,\
                                child_frame_id,\
                                x,y,z,\
                                q,\
                                vx,vy,vz,\
                                rollspeed,pitchspeed,yawspeed,\
                                pose_covariance,velocity_covariance,\
                                reset_counter,estimator_type)

    
    # odom_bag.write('/odomoetry/filtered', odom_sub)

    # ### PLANNER DATA
    # # Preparing set_position_target mavlink message
    # time_boot_ms = 0
    # target_system = 0 # not used
    # target_component = 0 # not used
    # coordinate_frame = 1 # NED
    # type_masks = 0 # not used
    # # Desired position
    # x = plan_sub.pose.pose.position.x
    # y = plan_sub.pose.pose.position.y
    # z = plan_sub.pose.pose.position.z
    # # Desired velocity
    # vx = plan_sub.twist.twist.linear.x
    # vy = plan_sub.twist.twist.linear.y
    # vz = plan_sub.twist.twist.linear.z
    # # Start point
    # afx = plan_sub.pose.pose.orientation.x 
    # afy = plan_sub.pose.pose.orientation.y
    # afz = 0  # not used
    # # Desired yaw and yaw rate
    # yaw = plan_sub.pose.pose.orientation.w
    # yaw_rate = plan_sub.twist.twist.angular.z
    # # Setnd set_position_target mavlink message
    # connection.mav.set_position_target_local_ned_send(time_boot_ms, \
    #                                                   target_system, \
    #                                                   target_component,
    #                                                   coordinate_frame, \
    #                                                   type_masks, \
    #                                                   x,y,z,\
    #                                                   vx,vy,vz,\
    #                                                   afx,afy,afz,\
    #                                                   yaw,yaw_rate)

#   # Preparing IMU mavlink message
#     time_usec = 0 
#     frame_id = 0
#     child_frame_id = 0
#     # Linear acceleration
#     ax = imu_sub.linear_acceleration.x
#     ay = imu_sub.linear_acceleration.y
#     az = imu_sub.linear_acceleration.z
#     # Quaternion
#     q = [imu_sub.orientation.w,imu_sub.orientation.x,imu_sub.orientation.y,imu_sub.orientation.z]
#         # Angular velocity
#     rollspeed = odom_sub.angular_velocity.x
#     pitchspeed = odom_sub.angular_velocity.y
#     yawspeed = odom_sub.angular_velocity.z
#     # The following fields are not used
#     reset_counter = 0
#     estimator_type = 0
#     # Send IMU mavlink message
#     connection.mav.imu_send(time_usec,frame_id,child_frame_id, ax, ay, az,q, rollspeed,pitchspeed,yawspeed,reset_counter,estimator_type)                      

                                 

def mavlink_manager():
    rospy.init_node('mavlink_manager', anonymous=True)
    
    # # Subscribe to messages
    # odom_sub  = message_filters.Subscriber('/odom', Odometry)
    # plan_sub = message_filters.Subscriber('/traj_plann', Odometry)
    # # Syncronize
    # ts = message_filters.ApproximateTimeSynchronizer([odom_sub,plan_sub], queue_size=10, slop=0.5)
    
    # # Run callback
    # ts.registerCallback(callback)

    odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, callback=mavlink_send, queue_size=1)
    # imu_sub =rospy.Subscriber("/imu/data", Imu, callback=mavlink_send, queue_size=1)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        mavlink_manager()
    except rospy.ROSInterruptException:
        pass