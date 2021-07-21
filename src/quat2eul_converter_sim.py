import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import message_filters
from scipy.spatial.transform import Rotation as R

class Quat2Eul_visualOdom(): 
    def callback(self,sub_imu,sub_filter):
        self.msg.pose.pose.orientation = sub_imu.orientation
        [yaw_imu, pitch_rs, roll_rs] = self.get_rotation(self.msg)
        self.msg.pose.pose.orientation = sub_filter.orientation 
        [yaw_fil, pitch_imu, roll_imu] = self.get_rotation(self.msg) 
        print("Euler angles: Yaw Imu:", yaw_imu,"Yaw Filter:", yaw_fil)
        # print("Positions in x,y: Ximu:")

    def get_rotation(self,Odom):
        orientation_q = Odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        EuAn = r.as_euler('zyx', degrees=True)
        return EuAn 

    def __init__(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('quat2eul', anonymous=True)

        # sub_ekf= message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
        sub_imu = message_filters.Subscriber("/imu/data_raw",Imu,queue_size = 10)
        sub_filter = message_filters.Subscriber("/imu/filtered",Imu,queue_size = 10)
        self.msg = Odometry()
        ts = message_filters.ApproximateTimeSynchronizer([sub_imu,sub_filter], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    Quat2Eul_visualOdom()