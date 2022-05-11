#! /usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

def qr_callback(msg):
    print(msg.pose.position.x)
    cmd_pub =rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twt = Twist()
    twt.linear.x = msg.pose.position.x
    twt.linear.y = msg.pose.position.y
    twt.angular.z = msg.pose.orientation.z
    cmd_pub.publish(twt)

if __name__ == "__main__":
    rospy.init_node("QR_conveter")
    rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, qr_callback)
    rospy.spin()

