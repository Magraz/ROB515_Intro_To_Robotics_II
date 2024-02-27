#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ROB515_Intro_To_Robotics_II/RobotSystems/ArmPi')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class TrackBox():
    def __init__(self):
        self.ArmIK = ArmIK()

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.count = 0
        self.track = False
        self._stop = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

        self.__target_color = ()

        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, world_Y = 0, 0
        self.world_x, world_y = 0, 0

        # The angle at which the gripper closes when clamping
        self.servo1 = 500

        self.roi = ()
        self.rect = None
        self.area_max = 0
        self.areaMaxContour = 0
        self.frame_lab = None
        self.img = None
        self.size = (640, 480)
        self.square_length = 3
        self.t1 = 0
        self.last_x = 0
        self.last_y = 0

    # Set detection color
    def setTargetColor(self,target_color):
        #print("COLOR", target_color)
        self.__target_color = target_color
        return (True, ())

    # Find the contour with the largest area
    # The argument is a list of contours to compare
    def getAreaMaxContour(self,contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  # 返回最大的轮廓

    # 初始位置
    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)
    
    # 变量重置
    def reset(self):
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    # app initialization call
    def init(self):
        print("ColorTracking Init")
        self.initMove()

    # app starts gameplay call
    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    # app stops gameplay call
    def stop(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Stop")

    # app exit gameplay call
    def exit(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Exit")

    def img_preprocess(self):
        img_copy = self.img.copy()
        img_h, img_w = self.img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not self.__isRunning:
            return self.img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        
        #If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        
        self.frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #Convert image to LAB space
    
    def get_contour(self, color):
        self.detect_color = color
        frame_mask = cv2.inRange(self.frame_lab, color_range[color][0], color_range[color][1])  #Perform bit operations on original image and mask
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
        self.areaMaxContour, self.area_max = self.getAreaMaxContour(contours)  # Find the maximum contour
    
    def find_largest_area(self, index):
        if self.areaMaxContour is not None:
            if self.area_max > self.max_area:#Find the largest area
                self.max_area = self.area_max
                self.color_area_max = index

    def get_current_world_xy(self):
        self.rect = cv2.minAreaRect(self.areaMaxContour)
        box = np.int0(cv2.boxPoints(self.rect))

        self.roi = getROI(box) #Get roi area
        self.get_roi = True

        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, self.square_length)  # Get the center coordinates of the wooden block
        self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size) #Convert to real world coordinates
        
        
        cv2.drawContours(self.img, [box], -1, self.range_rgb[self.detect_color], 2)
        cv2.putText(self.img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detect_color], 1) #draw center point
        self.distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2)) #Compare the last coordinates to determine whether to move
        self.last_x, self.last_y = self.world_x, self.world_y

    def select_color_to_detect(self):

        if self.color_area_max == 'red':  #Red is the largest
            self.color = 1
        elif self.color_area_max == 'green':  #Green is the biggest
            self.color = 2
        elif self.color_area_max == 'blue':  #blue is the biggest
            self.color = 3
        else:
            self.color = 0
    
        if len(self.color_list) == 3:  #multiple judgments
            # take the average
            self.color = int(round(np.mean(np.array(self.color_list))))
            self.color_list = []
        if self.color == 1:
            self.detect_color = 'red'
            self.draw_color = range_rgb["red"]
        elif self.color == 2:
            self.detect_color = 'green'
            self.draw_color = range_rgb["green"]
        elif self.color == 3:
            self.detect_color = 'blue'
            self.draw_color = range_rgb["blue"]
        else:
            self.detect_color = 'None'
            self.draw_color = range_rgb["black"]
    
    def get_avg_world_xy(self, distance):

        if self.distance < distance:
            self.center_list.extend((self.world_x, self.world_y))
            self.count += 1
            if self.start_count_t1:
                self.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                self.rotation_angle = self.rect[2]
                self.start_count_t1 = True
                self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                self.count = 0
                self.center_list = []
                self.start_pick_up = True
        else:
            self.t1 = time.time()
            self.start_count_t1 = True
            self.count = 0
            self.center_list = []
        
    def run_tracking(self, img):

        self.img = img

        self.img_preprocess()

        if not self.start_pick_up:
            for color in color_range:
                if color in self.__target_color:
                    self.get_contour(color)
                    
                    if self.area_max > 2500:  # Found the largest area
                        
                        self.get_current_world_xy()
                        self.track = True

                        if self.action_finish:
                            # Cumulative judgment
                            self.get_avg_world_xy(distance=0.3)

        return self.img

    def run_sorting(self, img):

        self.img = img

        self.img_preprocess()

        self.color_area_max = None
        self.max_area = 0
        self.areaMaxContour_max = 0

        if not self.start_pick_up:
            for i in color_range:
                if i in __target_color:
                    self.get_contour()
                    self.find_largest_area(index = i)
            if self.max_area > 2500:  # Found the largest area

                self.get_current_world_xy()

                if not self.start_pick_up:
                    self.select_color_to_detect()
                    self.get_avg_world_xy(distance=0.5)

                else:
                    if not self.start_pick_up:
                        self.draw_color = (0, 0, 0)
                        self.detect_color = "None"

        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)

        return self.img
    
    def move_first_detection(self):
        self.action_finish = False
        self.setBuzzer(0.1)               
        result = self.ArmIK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0) # If the running time parameter is not filled in, the running time will be adaptive.

        if result == False:
            self.unreachable = True
        else:
            self.unreachable = False
        time.sleep(result[2]/1000) #The third item of the return parameter is the time
        self.start_pick_up = False
        self.first_move = False
        self.action_finish = True
    
    def move_pick_up(self):
        self.action_finish = False
        if not self.__isRunning: # Stop and exit flag detection
            continue
        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # Claws spread
        # Calculate the angle by which the gripper needs to be rotated
        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)
        
        if not self.__isRunning:
            continue
        self.ArmIK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0)  # lower the altitude
        time.sleep(2)
        
        if not self.__isRunning:
            continue
        Board.setBusServoPulse(1, self.servo1, 500)  # Gripper closed
        time.sleep(1)
        
        if not self.__isRunning:
            continue
        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0)  # Robotic arm raised
        time.sleep(1)
        
        if not self.__isRunning:
            continue
        #Classify and place blocks of different colors
        result = self.ArmIK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0)   
        time.sleep(result[2]/1000)
        
        if not self.__isRunning:
            continue
        servo2_angle = getAngle(coordinate[self.detect_color][0], coordinate[self.detect_color][1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        if not self.__isRunning:
            continue
        self.ArmIK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], coordinate[self.detect_color][2] + 3), -90, -90, 0, 500)
        time.sleep(0.5)
        
        if not self.__isRunning:
            continue
        self.ArmIK.setPitchRangeMoving((coordinate[self.detect_color]), -90, -90, 0, 1000)
        time.sleep(0.8)
        
        if not self.__isRunning:
            continue
        Board.setBusServoPulse(1, self.servo1 - 200, 500)  #Open your claws and drop the object
        time.sleep(0.8)
        
        if not self.__isRunning:
            continue                    
        self.ArmIK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0)
        time.sleep(0.8)

        self.initMove()  #Return to initial position
        time.sleep(1.5)

        self.detect_color = 'None'
        self.first_move = True
        self.get_roi = False
        self.action_finish = True
        self.start_pick_up = False

    def move_track(self):
        if not self.__isRunning: #Stop and exit flag detection
            continue
        self.ArmIK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
        time.sleep(0.02)                    
        self.track = False
    
    def move_stop(self):
        self._stop = False
        Board.setBusServoPulse(1, self.servo1 - 70, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def move(self):

        # Different color wood quick placement coordinates(x, y, z)
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

        while True:
            if self.__isRunning:

                if self.first_move and self.start_pick_up: # When an object is first detected

                    self.move_first_detection()

                elif not self.first_move and not self.unreachable: #This is not the first time an object is detected

                    if self.track: #If it is the tracking stage

                        self.move_track()

                    if self.start_pick_up: #If the object has not moved for a while, start gripping

                        self.move_pick_up()
                        
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:

                    self.move_stop()
                    
                time.sleep(0.01)

if __name__ == '__main__':

    track = TrackBox()

    # Run child thread
    th = threading.Thread(target=track.move)
    th.setDaemon(True)
    th.start()

    track.init()
    track.start()

    __target_color = ('green')
    track.setTargetColor(__target_color)

    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            # Frame = run(frame)
            frame = track.run_tracking(frame)           
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                # ArmIK.setPitchRangeMoving((-15, 6,  3), -90, -90, 0, 1000)
                # Board.setBusServoPulse(1, servo1 - 280, 500)
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
