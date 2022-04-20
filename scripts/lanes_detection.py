#retrieved from https://www.youtube.com/watch?v=eLTLtUVuuy4
#author: ProgrammingKnowledge
#Modified by Wataru Oshima
#Date 04/16, 2022

import argparse
#from ast import arg
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

class LANE_DETECTION:
    def __init__(self):
        self._src =None
        self._lane_image =None
        self._gray =None
        self._canny =None
        self._image =None
        self._mask =None
        self.lines=None
        self.combo_image=None
        self.right_line=None
        self.left_line=None
        self.average_lines=None

    def make_coordinate(self, line_parameters):
        # print(line_parameters)
        slope, intercept = line_parameters
        y1 = self._lane_image.shape[0]
        y2 = int(y1*(3/5))
        # print(y1, y2)
        x1 = int((y1-intercept)/slope)
        x2 =int((y2-intercept)/slope)
        return np.array([x1,y1,x2,y2])

    def avarage_slope_inercept(self):
        left_fit=[]
        right_fit=[]
        for line in self.lines:
            x1,y1,x2,y2 =line.reshape(4)
            parameters = np.polyfit((x1,x2),(y1,y2),1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope<0:
                left_fit.append((slope,intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        # print(right_fit_average, left_fit_average)
        if left_fit:
            self.left_line = self.make_coordinate(left_fit_average)
        if right_fit:
            self.right_line = self.make_coordinate(right_fit_average)
        
        

    def canny(self):
        self._gray = cv.cvtColor(self._lane_image, cv.COLOR_RGB2GRAY)
        self._blur = cv.GaussianBlur(self._gray, (5,5),0)
        # self._canny = cv.Canny(self._blur, 50, 100)
        self._canny = cv.Canny(self._blur, 50, 300)


    def display_lines(self):
        self._line_image = np.zeros_like(self._lane_image)
        # self._line_image = np.zeros_like(self._mask)
        avg_lines = np.array([self.left_line, self.right_line])
        if avg_lines is not None:
            for x1,y1,x2,y2 in avg_lines:
                cv.line(self._line_image, (x1,y1), (x2,y2), (255,0,0),10)

    def region_of_interest(self):
        # height = self._src.shape[0]
        # print(height)
        # height = height-200
        # triangle = np.array([[(300, height), (1000, height), (600,300)]])
        # # triangle = np.array([[(20, height), (600, height), (300,300)]])

        # masked_image = np.zeros_like(self._canny)
        # cv.fillPoly(masked_image, triangle, 255)
        # self._mask = cv.bitwise_and(self._canny, masked_image)

        # below is when you want to shapen a rectangle

        self._mask= np.copy(self._canny) 
        h, w = self._src.shape[:2]
        RESIZE = (w//3, h//3)
        search_top = (h//4)*3
        search_bot = search_top + 20
        self._mask[0:search_top, 0:w] = 0
        # self._mask[search_bot:h, 0:w] = 0
        
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="please put a file name with the path")
    parser.add_argument("detection", help="1/0")
    args = parser.parse_args()

    videofilename = args.name
    arg2 = args.detection
    flag=None
    try:
        flag = int(arg2)
    except Exception as e:
        flag = 1
    if  not videofilename:
        videofilename = "./videos/road.AVI"
    # cap = cv.VideoCapture("./videos/testrun.mp4")
    # cap = cv.VideoCapture("./videos/road.AVI")
    cap = cv.VideoCapture(videofilename)
    

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))  
    out = cv.VideoWriter('record_result.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))


    ld =LANE_DETECTION()
    if cap.isOpened():
        print("start detecting")
        while(cap.isOpened()):
            _, ld._src = cap.read()
            if flag:
                # ld._src= cv.imread("./videos/testrun.png")
                # ld._src= frame
                ld._lane_image= np.copy(ld._src) 
                ld.canny()
                ld.region_of_interest()
                ld.lines = cv.HoughLinesP(ld._mask, 2, np.pi/180, 100, np.array([]),  minLineLength=30, maxLineGap=5)
                # print(ld.lines)
                if ld.lines is not None:
                    try:
                        ld.avarage_slope_inercept()
                        ld.display_lines()
                        ld.combo_image = cv.addWeighted(ld._lane_image, 0.8, ld._line_image,1,1)
                    except Exception as e:
                        ld.combo_image = ld._src
                else:
                    ld.combo_image = ld._src
                cv.imshow("canny", ld._canny)
                cv.imshow("mask", ld._mask)
                cv.imshow("combo", ld.combo_image)
                out.write(ld.combo_image)
            else:
                cv.imshow("original", ld._src)
            # plt.imshow(ld._src)
            # plt.show()
            cv.waitKey(0)
        cap.release()
        out.release()
        cv.destroyAllWindows()
    else:
        print("cannot find the file")
