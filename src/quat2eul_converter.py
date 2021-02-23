import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

def callback(data):
    [yaw, pitch, roll] = get_rotation(data) 
    print('Euler angles', roll, pitch, yaw)

def get_rotation(Odom):
    orientation_q = Odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    r = R.from_quat(orientation_list)
    EuAn = r.as_euler('zyx', degrees=False)
    return EuAn 

def quat2eul():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('quat2eul', anonymous=True)

    rospy.Subscriber("/odometry/filtered", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    quat2eul()