#!/usr/bin/env python3

from math import cos, sin, sqrt,pi
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
import time
import os
from scipy.spatial.transform import Rotation as R
os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
from pymavlink import mavutil
# K64F_IP = '192.168.1.10' # Freedom K64F IP Address
# K64F_PORT='8150'         # Freedom K64F UPD port
# ADDRESS = 'upd:'+ K64F_IP + ':' + K64F_PORT
connection_in = mavutil.mavlink_connection('udp:0.0.0.0:8151', input=True)

def mavlink_receiver():
    # pub = rospy.Publisher('ekf_data_fused', Odometry, queue_size=10)
    # rospy.init_node('mavlink_manager_in', anonymous=True)
    # rate = rospy.Rate(5) # 10hz
    # while not rospy.is_shutdown():
    #     current_time = rospy.Time.now()
    #     ekf_data_fused = connection_in.recv_match(type='ODOMETRY', blocking=True) # connection_in.messages['Odometry']
    #     # print("ecco sta merda di msg")
    #     # print(connection_in)
    #     ekf_data_fused_ros = Odometry()
    #     ekf_data_fused_ros.header.stamp = current_time
    #     ekf_data_fused_ros.header.frame_id = "odom"
    #     ekf_data_fused_ros.pose.pose.position.x = ekf_data_fused.x
    #     ekf_data_fused_ros.pose.pose.position.y = ekf_data_fused.y
    #     ekf_data_fused_ros.pose.pose.orientation.w = ekf_data_fused.q[0]
    #     ekf_data_fused_ros.pose.pose.orientation.x = ekf_data_fused.q[3]
    #     ekf_data_fused_ros.twist.twist.linear.x = ekf_data_fused.vx
    #     ekf_data_fused_ros.twist.twist.linear.y = ekf_data_fused.vy
    #     pub.publish(ekf_data_fused_ros)
    #     rate.sleep()

    pub_enc = rospy.Publisher('encoder', Odometry, queue_size=10)
    pub_mag = rospy.Publisher('magnetometer', Odometry,queue_size=10)
    pub_imu = rospy.Publisher('imu_k64', Imu, queue_size=10)
    pub_imu2 = rospy.Publisher('imu_ext', Imu, queue_size=10)
    global pos_l_old
    global pos_l_new
    global pos_r_old
    global pos_r_new
    global psi_old
    global oldT
    oldT = 0
    pos_l_old = 0
    pos_l_new = 0
    pos_r_old = 0
    pos_r_new = 0
    psi_old = 0
    x_0 = 0
    y_0 = 0
    
    r = 0.02
    B = 0.185
    rospy.init_node('mavlink_manager_in', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        imu = connection_in.recv_match(type='SCALED_IMU', blocking=True)
        imu_ros = Imu()
        imu_ros.header.stamp = current_time
        imu_ros.header.frame_id = "odom"  # odom before
        imu_ros.linear_acceleration.x = imu.xacc/1000*9.81
        imu_ros.linear_acceleration.y = imu.yacc/1000*9.81
        imu_ros.linear_acceleration.z = imu.zacc/1000*9.81
        imu_ros.linear_acceleration_covariance[0] = imu.xmag/10**7
        imu_ros.linear_acceleration_covariance[4] = imu.ymag/10**7
        imu_ros.linear_acceleration_covariance[8] = imu.xgyro/10**7
        # imu_ros.orientation.z = imu.zmag/10000
        # imu_ros.orientation.w = sqrt(1-(imu.zmag/10000)**2)
        # imu_ros.orientation.x = 0
        # imu_ros.orientation.y = 0
        imu_ros.orientation_covariance[8] = imu.ygyro/10**5
        pub_imu.publish(imu_ros)
        # rospy.loginfo("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        # Magnetometer is treated as a separate Odom topic
        magneto_ros = Odometry()
        magneto_ros.header.stamp = current_time
        magneto_ros.header.frame_id = "odom"
        magneto_ros.pose.pose.orientation.z = imu.zmag/10000
        magneto_ros.pose.pose.orientation.w = sqrt(1-(imu.zmag/10000)**2)
        magneto_ros.pose.pose.orientation.x = 0
        magneto_ros.pose.pose.orientation.y = 0
        pub_mag.publish(magneto_ros)
        
        imu_ex = connection_in.recv_match(type='SCALED_IMU2', blocking=True)
        imu_ros2 = Imu()
        imu_ros2.header.stamp = current_time
        imu_ros2.header.frame_id = "odom"  # odom before
        imu_ros2.linear_acceleration.x = imu_ex.xacc/1000*9.81
        imu_ros2.linear_acceleration.y = imu_ex.yacc/1000*9.81
        imu_ros2.linear_acceleration.z = imu_ex.zacc/1000*9.81
        imu_ros2.linear_acceleration_covariance[0] = 0.03
        imu_ros2.linear_acceleration_covariance[4] = 0.04
        imu_ros2.linear_acceleration_covariance[8] = 0.05
        imu_ros2.angular_velocity.z = imu_ex.zgyro/1000*pi/180 # rad/second
        imu_ros2.angular_velocity.x = imu_ex.xgyro/1000*pi/180 # rad/second
        imu_ros2.angular_velocity.y = imu_ex.ygyro/1000*pi/180 # rad/second
        imu_ros2.orientation.z = 0
        imu_ros2.orientation.w = 0
        imu_ros2.orientation.x = 0
        imu_ros2.orientation.y = 0
        imu_ros2.angular_velocity_covariance[0] = 2.5/10000
        imu_ros2.angular_velocity_covariance[4] = 2.7/10000
        imu_ros2.angular_velocity_covariance[8] = 2.6/10000
        pub_imu2.publish(imu_ros2)

        encoders = connection_in.recv_match(type='WHEEL_DISTANCE', blocking=True) # connection_in.messages['Odometry']
        # # print(connection_in)
        # if pos_l_old== 0:
        #     pos_l_old = encoders.distance[0]
        #     pos_r_old = encoders.distance[1]
        #     psi_old = 0
            
        encoders_ros = Odometry()
        encoders_ros.header.stamp = current_time
        encoders_ros.header.frame_id = "odom"
        rot = R.from_quat([0, 0, magneto_ros.pose.pose.orientation.z, magneto_ros.pose.pose.orientation.w])
        [psi,_,_] = rot.as_euler("zyx", degrees = False)
        
        # Integration of psi based on encoder positions
        pos_l_new = encoders.distance[0]
        pos_r_new = encoders.distance[1]
        # currentT = (float)encoders_ros.header.stamp
        # deltaT = currentT - oldT
        # oldT = currentT
        deltaPos_l = pos_l_new-pos_l_old
        deltaPos_r = pos_r_new-pos_r_old
        psi_new = psi_old + (1/B)*(deltaPos_r-deltaPos_l)  # Rad
        VR = deltaPos_r
        VL = deltaPos_l
        pos_l_old = pos_l_new
        pos_r_old = pos_r_new
        psi_old = psi_new

        # if (x_0 == 0) and (y_0 == 0):
        #     x_0 = (encoders.distance[0]+encoders.distance[1])/2*cos(psi_new)
        #     y_0 = (encoders.distance[0]+encoders.distance[1])/2*sin(psi_new)

        # encoders_ros.pose.pose.position.x = (encoders.distance[0]+encoders.distance[1])/2*cos(psi_new) #-x_0
        # encoders_ros.pose.pose.position.y = (encoders.distance[0]+encoders.distance[1])/2*sin(psi_new) #-y_0
        encoders_ros.pose.pose.position.x = x_0 + (VR+VL)/2*cos(psi_new)
        encoders_ros.pose.pose.position.y = y_0 + (VR+VL)/2*sin(psi_new)
        x_0 = encoders_ros.pose.pose.position.x
        y_0 = encoders_ros.pose.pose.position.y
        print('Psi value from frdm', psi_new*180/pi, encoders_ros.pose.pose.position.x, encoders_ros.pose.pose.position.y, (encoders.distance[0]+encoders.distance[1]), x_0, y_0)
        # encoders_ros.pose.pose.position.y = 0
        pub_enc.publish(encoders_ros)
        rate.sleep()

if __name__ == '__main__':
    try:
        mavlink_receiver()
    except rospy.ROSInterruptException:
        pass