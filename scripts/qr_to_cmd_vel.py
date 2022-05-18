#! /usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

def qr_callback(msg):
    # print(msg.pose.position.x)
    cmd_pub =rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twt = Twist()
    # twt.linear.x = msg.pose.position.x

    """
    Pose Stamped info
    psoe.position.x is the horizontal position
    pose.position.z is the distance from robot 0.1 is 10cm
    """
    
    if msg.pose.position.x > 0.1:
       twt.angular.z = 0.1
    elif msg.pose.position.x < -0.1:
       twt.angular.z = 0.1
    # twt.angular.z = msg.pose.position.x
    # twt.angular.z = -msg.pose.orientation.z
    cmd_pub.publish(twt)

if __name__ == "__main__":
    rospy.init_node("QR_conveter")
    
    rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, qr_callback)
    rospy.spin()

