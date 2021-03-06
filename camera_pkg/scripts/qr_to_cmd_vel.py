#! /usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

class QR_TRACK:
    def __init__(self):
        self.RUN = False
        self.twt = Twist()
        rospy.loginfo("start")
        self.cmd_pub =rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, self.qr_callback)
        rospy.Subscriber("/visp_auto_tracker/status", Int8, self.qr_status_callback)
        
        

    def qr_status_callback(self, msg):
        if msg.data == 1:
            self.RUN = False
        else:
            self.RUN = True


    def qr_callback(self, msg):
        # print(msg.pose.position.x)
        
        
        # twt.linear.x = msg.pose.position.x

        """
        Pose Stamped info
        psoe.position.x is the horizontal position
        pose.position.z is the distance from robot 0.1 is 10cm
        pose.orientation.z can be used for adjusting horizontal positiion
        """
        
        if msg.pose.position.x > 0.05:
            self.twt.angular.z = - 5 * msg.pose.position.x
            # twt.angular.z = -0.1
        elif msg.pose.position.x < -0.05:
            self.twt.angular.z = -5 * msg.pose.position.x
            # twt.angular.z = 0.1
        else:
            self.twt.angular.z = 0
        
        if msg.pose.position.z > 0.35:
            self.twt.linear.x =0.1
        elif msg.pose.position.z < 0.3:
            self.twt.linear.x =-0.1
            # self.twt.angular.z *=-1
        else:
            self.twt.linear.x =0
        # twt.angular.z = msg.pose.position.x
        # twt.angular.z = -msg.pose.orientation.z
        if self.twt.linear.x ==0 and self.twt.angular.z ==0:
            if msg.pose.orientaion.z >0.1:
                self.twt.linear.y = -0.1
            elif msg.pose.orientation.z <-0.1:
                self.twt.linear.y = 0.1
        if self.RUN:
            self.cmd_pub.publish(self.twt)

if __name__ == "__main__":
    rospy.init_node("QR_conveter")
    qrt = QR_TRACK()
    rospy.spin()

