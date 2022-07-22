#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

if __name__ == "__main__":
    rospy.init_node('VideoPublisher', anonymous=True)

    VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=10)

    cam = cv2.VideoCapture(0)
    if cam.isOpened(): 
        w  = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)) # float `width`
        h = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))  # float `height`
        fps = cam.get(cv2.CAP_PROP_FPS)
    print(w,h,fps)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        meta, frame = cam.read()
        frame = cv2.resize(frame, (w, h))
        cv2.imshow("frame", frame)
        # I want to publish the Canny Edge Image and the original Image
        msg_frame = CvBridge().cv2_to_imgmsg(frame)

        VideoRaw.publish(msg_frame, "RGB8")
        cv2.waitKey(3)
        rate.sleep()