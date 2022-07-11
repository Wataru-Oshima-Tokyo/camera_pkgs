
#!/usr/bin/env python3
import rospy
from cv2 import destroyAllWindows, waitKey
import cv2 as cv 
import os
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge


drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
cx,cy =0,0
Drew = False
windowName_src = "src"
windonwname_hsv = "hsv"

def get_com():
    M = cv.moments(mask)
    if M['m00']>0:
        c_x  =int(M['m10']/M['m00'])
        c_y = int(M['m01']/M['m00'])
        cv.circle(ROI,(c_x,c_y),5,(0,0,255),-1)


# def canny():
#         self._gray = cv.cvtColor(self._lane_image, cv.COLOR_RGB2GRAY)
#         self._blur = cv.GaussianBlur(self._gray, (5,5),0)
#         # self._canny = cv.Canny(self._blur, 50, 100)
#         self._canny = cv.Canny(self._blur, 50, 300)

def get_hsv(event,x,y,flags,param):
    global mask, ROI, src_hsv
    if event == cv.EVENT_LBUTTONDOWN:
        pixel = src_hsv[y,x]

        #you might want to adjust the ranges(+-10, etc):
        upper =  np.array([pixel[0] + 10, pixel[1] + 10, pixel[2] + 40])
        lower =  np.array([pixel[0] - 10, pixel[1] - 10, pixel[2] - 40])
        print(pixel, lower, upper)
        bin_img = cv.inRange(src_hsv, lower, upper)
        contours, _ = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=lambda x: cv.contourArea(x))
        mask = np.zeros_like(bin_img)
        cv.drawContours(mask, [contour], -1, color=255, thickness=-1)
        # image_mask = cv.inRange(src_hsv,lower,upper)
        get_com()
        cv.imshow("mask",ROI)

def draw_circle(event,x,y,flags,param):
    global src,cx,cy,ix,iy,drawing,mode, Drew

    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv.rectangle(src,(ix,iy),(x,y),(0,255,0),-1)
            else:
                cv.circle(src,(x,y),5,(0,0,255),-1)

    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv.rectangle(src,(ix,iy),(x,y),(0,255,255),2)
            cx, cy = x,y
            print(cx,cy, ix,iy)
            Drew = True
            cv.destroyAllWindows()
        else:
            cv.circle(src,(x,y),5,(0,0,255),-1)

def process_image(msg):
    global src, src_hsv,ROI
    cv.namedWindow(windowName_src)
    cv.setMouseCallback(windowName_src,draw_circle)
    try:
        bridge = CvBridge()
        src = bridge.imgmsg_to_cv2(msg, "bgr8")
        w,h,c = src.shape
        src = cv.resize(src, (w, h))
        ROI = np.copy(src)
        if not Drew:
            cv.imshow(windowName_src,src)
        else:
            cv.rectangle(ROI,(ix,iy),(cx,cy),(0,255,0),2)
            ROI[0:iy, 0:w] = 0
            ROI[iy:cy, 0:ix] = 0
            ROI[iy:cy, cx:w] = 0
            ROI[cy:h, 0:w] = 0
            cv.imshow("ROI",ROI)
            cv.namedWindow(windonwname_hsv)
            cv.setMouseCallback(windonwname_hsv,get_hsv)
            src_hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)
            cv.imshow("hsv", src_hsv)
        k = cv.waitKey(3) & 0xFF
        if k == ord('m'):
            mode = not mode
        elif k == ord('d'):
            cv.destroyAllWindows()
            Drew = not Drew
            print(cx,cy, ix,iy)

    except Exception as e:
        print(e)

def start():
    rospy.init_node('Outlet_detection')
    rospy.loginfo('Outlet detection start')
    rospy.Subscriber("/camera/color/image_raw", Image, process_image)
    rospy.spin()
    cv.destroyAllWindows()



if __name__ =="__main__":
    try:
        start()
    except rospy.ROSInterruptException:
        pass
