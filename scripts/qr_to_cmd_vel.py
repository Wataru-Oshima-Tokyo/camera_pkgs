#! /usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

def qr_callback(msg):
    print(msg.pose.position.x)


if __name__ == "__main__":
    rospy.init_node("QR_conveter")
    rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Suscriber("/visp_auto_tracker/object_position", PoseStamped, qr_callback)
    rospy.Spin()