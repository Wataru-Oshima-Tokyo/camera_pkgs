
#!/usr/bin/env python3
from cv2 import destroyAllWindows, waitKey
import cv2 as cv 
import os
import numpy as np

drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
cx,cy =0,0
Drew = False

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
    global mask
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
    global cx,cy,ix,iy,drawing,mode, Drew

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

if __name__ =="__main__":
    home_path = os.path.expanduser('~')
    windowName_src = "src"
    windonwname_hsv = "hsv"
    # print(home_path)
    # ws_path = home_path + "/catkin_ws/src/camera_pkgs/camera_pkg/img/"
    # src = cv.imread(ws_path + "outlet.jpg")
    cv.namedWindow(windowName_src)
    
    
    cv.setMouseCallback(windowName_src,draw_circle)
    


    #make a region of interest
    cap = cv.VideoCapture(0)
    if cap.isOpened(): 
        w  = int(cap.get(cv.CAP_PROP_FRAME_WIDTH)) # float `width`
        h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))  # float `height`
        fps = cap.get(cv.CAP_PROP_FPS)
    
    print(w,h,fps)
    while 1:
        ret, src = cap.read()
        # w,h,c = src.shape
        
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
            break
        elif k ==27:
            break
    # cv.destroyAllWindows()

    
    
    # cv.waitKey(0)
            
    cap.release()
    cv.destroyAllWindows()
 