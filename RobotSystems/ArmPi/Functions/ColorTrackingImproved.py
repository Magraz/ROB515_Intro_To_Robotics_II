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

# Run child thread
# th = threading.Thread(target=move)
# th.setDaemon(True)
# th.start()

class TrackBox():
    ArmIK = ArmIK()

    range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
    }

    count = 0
    track = False
    _stop = False
    get_roi = False
    center_list = []
    first_move = True
    __isRunning = False
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True

    __target_color = ()

    rotation_angle = 0
    unreachable = False
    world_X, world_Y = 0, 0
    world_x, world_y = 0, 0

     # The angle at which the gripper closes when clamping
    servo1 = 500

    # Set detection color
    def setTargetColor(self,target_color):
        #print("COLOR", target_color)
        TrackBox.__target_color = target_color
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
        Board.setBusServoPulse(1, TrackBox.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        TrackBox.ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)
    
    # 变量重置
    def reset(self):
        TrackBox.count = 0
        TrackBox._stop = False
        TrackBox.track = False
        TrackBox.get_roi = False
        TrackBox.center_list = []
        TrackBox.first_move = True
        TrackBox.__target_color = ()
        TrackBox.detect_color = 'None'
        TrackBox.action_finish = True
        TrackBox.start_pick_up = False
        TrackBox.start_count_t1 = True

    # app initialization call
    def init(self):
        print("ColorTracking Init")
        self.initMove()

    # app starts gameplay call
    def start(self):
        self.reset()
        TrackBox.__isRunning = True
        print("ColorTracking Start")

    # app stops gameplay call
    def stop(self):
        TrackBox._stop = True
        TrackBox.__isRunning = False
        print("ColorTracking Stop")

    # app exit gameplay call
    def exit(self):
        TrackBox._stop = True
        TrackBox.__isRunning = False
        print("ColorTracking Exit")
    
    def __init__(self):
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

    def img_preprocess(self):

        img_copy = self.img.copy()
        img_h, img_w = self.img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not TrackBox.__isRunning:
            return self.img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        
        #If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
        if TrackBox.get_roi and TrackBox.start_pick_up:
            TrackBox.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        
        self.frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #Convert image to LAB space
    
    def get_contour(self, color):
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

    def get_current_world_xy(self, color):
        self.rect = cv2.minAreaRect(self.areaMaxContour)
        box = np.int0(cv2.boxPoints(self.rect))

        self.roi = getROI(box) #Get roi area
        TrackBox.get_roi = True

        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, self.square_length)  # Get the center coordinates of the wooden block
        TrackBox.world_x, TrackBox.world_y = convertCoordinate(img_centerx, img_centery, self.size) #Convert to real world coordinates
        
        
        cv2.drawContours(self.img, [box], -1, TrackBox.range_rgb[color], 2)
        cv2.putText(self.img, '(' + str(TrackBox.world_x) + ',' + str(TrackBox.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, TrackBox.range_rgb[color], 1) #draw center point
        self.distance = math.sqrt(pow(TrackBox.world_x - self.last_x, 2) + pow(TrackBox.world_y - self.last_y, 2)) #Compare the last coordinates to determine whether to move
        self.last_x, self.last_y = TrackBox.world_x, TrackBox.world_y

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
            TrackBox.detect_color = 'red'
            self.draw_color = range_rgb["red"]
        elif self.color == 2:
            TrackBox.detect_color = 'green'
            self.draw_color = range_rgb["green"]
        elif self.color == 3:
            TrackBox.detect_color = 'blue'
            self.draw_color = range_rgb["blue"]
        else:
            TrackBox.detect_color = 'None'
            self.draw_color = range_rgb["black"]
    
    def get_avg_world_xy(self, distance):

        if self.distance < distance:
            TrackBox.center_list.extend((TrackBox.world_x, TrackBox.world_y))
            TrackBox.count += 1
            if TrackBox.start_count_t1:
                TrackBox.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                TrackBox.rotation_angle = self.rect[2]
                TrackBox.start_count_t1 = True
                TrackBox.world_X, TrackBox.world_Y = np.mean(np.array(TrackBox.center_list).reshape(TrackBox.count, 2), axis=0)
                TrackBox.count = 0
                TrackBox.center_list = []
                TrackBox.start_pick_up = True
        else:
            self.t1 = time.time()
            TrackBox.start_count_t1 = True
            TrackBox.count = 0
            TrackBox.center_list = []
        
    def run_tracking(self, img):

        self.img = img

        self.img_preprocess()

        #if not TrackBox.start_pick_up:
        for i in color_range:
            if i in TrackBox.__target_color:
                self.get_contour(color=i)
                #print(f'area {self.area_max}')
                if self.area_max > 2500:  # Found the largest area
                    
                    self.get_current_world_xy(color=i)
                    TrackBox.track = True

                    #if TrackBox.action_finish:
                    # Cumulative judgment
                    self.get_avg_world_xy(distance=0.3)

        return self.img

    def run_sorting(self, img):

        self.img = img

        self.img_preprocess()

        self.color_area_max = None
        self.max_area = 0
        self.areaMaxContour_max = 0

        if not TrackBox.start_pick_up:
            for i in color_range:
                if i in __target_color:
                    self.get_contour()
                    self.find_largest_area(index = i)
            if self.max_area > 2500:  # Found the largest area

                self.get_current_world_xy()

                if not TrackBox.start_pick_up:
                    self.select_color_to_detect()
                    self.get_avg_world_xy(distance=0.5)

                else:
                    if not TrackBox.start_pick_up:
                        self.draw_color = (0, 0, 0)
                        TrackBox.detect_color = "None"

        cv2.putText(img, "Color: " + TrackBox.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)

        return self.img
        

if __name__ == '__main__':
    track = TrackBox()
    track.init()
    track.start()

    __target_color = ('green', 'blue', 'red')
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
