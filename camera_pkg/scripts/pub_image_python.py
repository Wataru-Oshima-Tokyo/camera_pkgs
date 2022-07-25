#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

if __name__ == "__main__":
    rospy.init_node('VideoPublisher', anonymous=True)

    VideoRaw = rospy.Publisher('usb_cam/color/image', Image, queue_size=10)

    cam = cv2.VideoCapture(0)
    if cam.isOpened(): 
        w  = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)) # float `width`
        h = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))  # float `height`
        fps = cam.get(cv2.CAP_PROP_FPS)
    print(w,h,fps)
    rate = rospy.Rate(100)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        meta, frame = cam.read()
        frame = cv2.resize(frame, (w/2, h/2))
        # cv2.imshow("frame", frame)
        # I want to publish the Canny Edge Image and the original Image
        msg_frame =bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        VideoRaw.publish(msg_frame)
        # cv2.waitKey(3)
        rate.sleep()