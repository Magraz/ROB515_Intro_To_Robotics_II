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

ArmIK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('green')
# Set detection color
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# Find the contour with the largest area
# The argument is a list of contours to compare
def getAreaMaxContour(contours):
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

# The angle at which the gripper closes when clamping
servo1 = 500

# 初始位置
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

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
# 变量重置
def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1
    
    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True

# app initialization call
def init():
    print("ColorTracking Init")
    initMove()

# app starts gameplay call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app stops gameplay call
def stop():
    global _stop 
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")

# app exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")

rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
# Robotic arm moving thread
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move

    # Different color wood quick placement coordinates(x, y, z)
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 3),
        'green': (-15, 6,  3),
        'blue':  (-15 + 0.5, 0 - 0.5,  3),
    }
    while True:
        print(f'World X {world_x}, World Y {world_y}')
        # result = ArmIK.setPitchRangeMoving((2.67, 22.11, 10), -90, -90, 0) # If the running time parameter is not filled in, the running time will be adaptive.
        # if result == False:
        #     unreachable = True
        # time.sleep(1) 
        if __isRunning:
            if first_move and start_pick_up: # When an object is first detected               
                action_finish = False
                #set_rgb(detect_color)
                setBuzzer(0.1)               
                result = ArmIK.setPitchRangeMoving((world_X, world_Y - 10, 5), -90, -90, 0) # If the running time parameter is not filled in, the running time will be adaptive.
                if result == False:
                    result = [0,0,1000]
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2]/1000) # The third item of the return parameter is the time
                start_pick_up = False
                first_move = False
                action_finish = True
            elif not first_move and not unreachable: # This is not the first time an object is detected
                #set_rgb(detect_color)
                if track: # If it is the tracking stage
                    if not __isRunning: # Stop and exit flag detection
                        continue
                    ArmIK.setPitchRangeMoving((world_x, world_y - 10, 5), -90, -90, 0)
                    time.sleep(0.02)                    
                    track = False
                if start_pick_up: #If the object has not moved for a while, start gripping
                    action_finish = False
                    if not __isRunning: # Stop and exit flag detection
                        continue
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Claws spread
                    # Calculate the angle by which the gripper needs to be rotated
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue
                    
                    ArmIK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0)  # lower the altitude
                    time.sleep(2)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1+300, 500)  # Gripper closed
                    time.sleep(1)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    
                    ArmIK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0)  # Robotic arm raised
                    time.sleep(1)
                    
                    if not __isRunning:
                        continue
                    # Classify and place blocks of different colors
                    result = ArmIK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    if result == False:
                        result = [0,0,1000]
                    time.sleep(result[2]/1000)
                    
                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    ArmIK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue
                    ArmIK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开，放下物体
                    time.sleep(0.8)
                    
                    if not __isRunning:
                        continue                    
                    ArmIK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # 回到初始位置
                    time.sleep(1.5)

                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    #set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)

# Run child thread
# th = threading.Thread(target=move)
# th.setDaemon(True)
# th.start()

def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up, first_move
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
    
    if not __isRunning:
        return img
     
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    
    #If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)    
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #Convert image to LAB space
    
    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:
        for i in color_range:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  #Perform bit operations on original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the maximum contour
        if area_max > 2500:  # Found the largest area
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box) #Get roi area
            get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # Get the center coordinates of the wooden block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #Convert to real world coordinates
            
            
            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #draw center point
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #Compare the last coordinates to determine whether to move
            last_x, last_y = world_x, world_y
            track = True
            #print(count,distance)

            # Cumulative judgment
            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
    return img

class TrackBox():
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
        global start_pick_up
        global __isRunning
        global get_roi

        img_copy = self.img.copy()
        img_h, img_w = self.img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not __isRunning:
            return self.img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        
        #If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        
        self.frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #Convert image to LAB space
    
    def get_contour(self):
        global detect_color

        frame_mask = cv2.inRange(self.frame_lab, color_range[detect_color][0], color_range[detect_color][1])  #Perform bit operations on original image and mask
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
        self.areaMaxContour, self.area_max = getAreaMaxContour(contours)  # Find the maximum contour
    
    def find_largest_area(self, index):
        if self.areaMaxContour is not None:
            if self.area_max > self.max_area:#Find the largest area
                self.max_area = self.area_max
                self.color_area_max = index

    def get_current_world_xy(self):
        global get_roi
        global detect_color
        global world_x, world_y

        self.rect = cv2.minAreaRect(self.areaMaxContour)
        box = np.int0(cv2.boxPoints(self.rect))

        self.roi = getROI(box) #Get roi area
        get_roi = True

        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, self.square_length)  # Get the center coordinates of the wooden block
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size) #Convert to real world coordinates
        
        
        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #draw center point
        self.distance = math.sqrt(pow(world_x - self.last_x, 2) + pow(world_y - self.last_y, 2)) #Compare the last coordinates to determine whether to move
        self.last_x, self.last_y = world_x, world_y

    def select_color_to_detect(self):
        global detect_color

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
            detect_color = 'red'
            self.draw_color = range_rgb["red"]
        elif self.color == 2:
            detect_color = 'green'
            self.draw_color = range_rgb["green"]
        elif self.color == 3:
            detect_color = 'blue'
            self.draw_color = range_rgb["blue"]
        else:
            detect_color = 'None'
            self.draw_color = range_rgb["black"]
    
    def get_avg_world_xy(self, distance):
        global start_count_t1
        global world_X, world_Y
        global start_pick_up
        global world_x, world_y
        global count
        global center_list
        global rotation_angle

        if self.distance < distance:
            center_list.extend((world_x, world_y))
            count += 1
            if start_count_t1:
                start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                rotation_angle = self.rect[2]
                start_count_t1 = True
                world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                count = 0
                center_list = []
                start_pick_up = True
        else:
            self.t1 = time.time()
            start_count_t1 = True
            count = 0
            center_list = []
        
    def run_tracking(self, img):
        global start_pick_up
        global track

        self.img = img

        self.img_preprocess()

        if not start_pick_up:
            for i in color_range:
                if i in __target_color:
                    self.get_contour()
            if self.area_max > 2500:  # Found the largest area
                
                self.get_current_world_xy()
                track = True

                if action_finish:
                    # Cumulative judgment
                    self.get_avg_world_xy(distance=0.3)

        return img

    def run_sorting(self, img):
        global start_pick_up
        global detect_color

        self.img = img

        self.img_preprocess()

        self.color_area_max = None
        self.max_area = 0
        self.areaMaxContour_max = 0

        if not start_pick_up:
            for i in color_range:
                if i in __target_color:
                    self.get_contour()
                    self.find_largest_area(index = i)
            if self.max_area > 2500:  # Found the largest area

                self.get_current_world_xy()

                if not start_pick_up:
                    self.select_color_to_detect()
                    self.get_avg_world_xy(distance=0.5)

                else:
                    if not start_pick_up:
                        self.draw_color = (0, 0, 0)
                        detect_color = "None"

        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)

        return img
        

if __name__ == '__main__':
    init()
    start()
    __target_color = ('green', )
    my_camera = Camera.Camera()
    my_camera.camera_open()

    track = TrackBox()

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            # Frame = run(frame)
            track.run_tracking(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                # ArmIK.setPitchRangeMoving((-15, 6,  3), -90, -90, 0, 1000)
                # Board.setBusServoPulse(1, servo1 - 280, 500)
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
