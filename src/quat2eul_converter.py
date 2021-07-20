import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import message_filters
from scipy.spatial.transform import Rotation as R

class Quat2Eul_visualOdom(): 
    def callback(self,sub_ekf,sub_RSimag,sub_imudata):
        self.msg.pose.pose.orientation = sub_imudata.orientation
        [yaw_ekf, pitch_ekf, roll_ekf] = self.get_rotation(sub_ekf) 
        [yaw_rs, pitch_rs, roll_rs] = self.get_rotation(sub_RSimag) 
        [yaw_imu, pitch_imu, roll_imu] = self.get_rotation(self.msg) 
        print('Euler angles: ekf:', yaw_ekf,"RSimag:", yaw_rs,"RSimu:", yaw_imu)

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

        sub_ekf= message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
        sub_RSimag = message_filters.Subscriber("/rtabmap/odom",Odometry,queue_size = 10)
        sub_imudata = message_filters.Subscriber("/imu/data",Imu,queue_size = 10)
        self.msg = Odometry()
        ts = message_filters.ApproximateTimeSynchronizer([sub_ekf,sub_RSimag,sub_imudata], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    Quat2Eul_visualOdom()