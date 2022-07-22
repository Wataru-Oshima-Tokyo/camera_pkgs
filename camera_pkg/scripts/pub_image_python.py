#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

rospy.init_node('VideoPublisher', anonymous=True)

VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=10)

cam = cv2.VideoCapture(0)
if cam.isOpened(): 
    w  = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)) # float `width`
    h = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))  # float `height`
    fps = cam.get(cv2.CAP_PROP_FPS)
print(w,h,fps)
while cam.isOpened():
    meta, frame = cam.read()
    frame = frame.resize(frame, (w, h))
    # I want to publish the Canny Edge Image and the original Image
    msg_frame = CvBridge().cv2_to_imgmsg(frame)

    VideoRaw.publish(msg_frame, "RGB8")

    time.sleep(0.1)