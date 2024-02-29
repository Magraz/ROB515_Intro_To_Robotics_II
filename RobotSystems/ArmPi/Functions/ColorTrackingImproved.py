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

class Status():
    def __init__(self):
        self._stop = False
        self.track = False
        self.get_roi = False
        self.first_move = True
        self.isRunning = False
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.detect_color = 'None'
        self.target_color = ()
        self.center_list = []
        self.count = 0

        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, world_Y = 0, 0
        self.world_x, world_y = 0, 0

        # The angle at which the gripper closes when clamping
        self.servo1 = 500

    
    # 变量重置
    def reset(self):
        self._stop = False
        self.track = False
        self.get_roi = False
        self.first_move = True
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.detect_color = 'None'
        self.target_color = ()
        self.center_list = []
        self.count = 0

    # app starts gameplay call
    def start(self):
        self.reset()
        self.isRunning = True
        print("ColorTracking Start")

    # app stops gameplay call
    def stop(self):
        self._stop = True
        self.isRunning = False
        print("ColorTracking Stop")

    # app exit gameplay call
    def exit(self):
        self._stop = True
        self.isRunning = False
        print("ColorTracking Exit")
    
    # Set detection color
    def setTargetColor(self,target_color):
        self.target_color = target_color
        return (True, ())

class TrackBox():
    def __init__(self, status: Status):
        self.status = status

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

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

    def img_preprocess(self):
        img_copy = self.img.copy()
        img_h, img_w = self.img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not self.status.isRunning:
            return self.img
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        
        #If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
        if self.status.get_roi and self.status.start_pick_up:
            self.status.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
        
        self.frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  #Convert image to LAB space
    
    def get_contour(self, color):
        self.status.detect_color = color
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
        self.status.get_roi = True

        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, self.square_length)  # Get the center coordinates of the wooden block
        self.status.world_x, self.status.world_y = convertCoordinate(img_centerx, img_centery, self.size) #Convert to real world coordinates
        
        
        cv2.drawContours(self.img, [box], -1, self.range_rgb[self.status.detect_color], 2)
        cv2.putText(self.img, '(' + str(self.status.world_x) + ',' + str(self.status.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.status.detect_color], 1) #draw center point
        self.distance = math.sqrt(pow(self.status.world_x - self.last_x, 2) + pow(self.status.world_y - self.last_y, 2)) #Compare the last coordinates to determine whether to move
        self.last_x, self.last_y = self.status.world_x, self.status.world_y

    def select_color_to_detect(self):

        if self.color_area_max == 'red':  #Red is the largest
            self.color = 1
        elif self.color_area_max == 'green':  #Green is the biggest
            self.color = 2
        elif self.color_area_max == 'blue':  #blue is the biggest
            self.color = 3
        else:
            self.color = 0
    
        if len(self.status.color_list) == 3:  #multiple judgments
            # take the average
            self.color = int(round(np.mean(np.array(self.color_list))))
            self.color_list = []
        if self.color == 1:
            self.status.detect_color = 'red'
            self.draw_color = range_rgb["red"]
        elif self.color == 2:
            self.status.detect_color = 'green'
            self.draw_color = range_rgb["green"]
        elif self.color == 3:
            self.status.detect_color = 'blue'
            self.draw_color = range_rgb["blue"]
        else:
            self.status.detect_color = 'None'
            self.draw_color = range_rgb["black"]
    
    def get_avg_world_xy(self, distance):

        if self.distance < distance:
            self.status.center_list.extend((self.status.world_x, self.status.world_y))
            self.status.count += 1
            if self.status.start_count_t1:
                self.status.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                self.rotation_angle = self.rect[2]
                self.status.start_count_t1 = True
                self.status.world_X, self.status.world_Y = np.mean(np.array(self.status.center_list).reshape(self.status.count, 2), axis=0)
                self.status.count = 0
                self.status.center_list = []
                self.status.start_pick_up = True
        else:
            self.t1 = time.time()
            self.status.start_count_t1 = True
            self.status.count = 0
            self.status.center_list = []
        
    def run_tracking(self, img):

        self.img = img

        self.img_preprocess()

        if not self.status.start_pick_up:
            for color in color_range:
                if color in self.status.target_color:
                    self.get_contour(color)
                    
                    if self.area_max > 2500:  # Found the largest area
                        
                        self.get_current_world_xy()
                        self.status.track = True

                        if self.status.action_finish:
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
                if i in target_color:
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
    

class Movement():
    def __init__(self, status_obj: Status):
        self.ArmIK = ArmIK()
        self.status = status_obj

        print(self.status._stop)

        # Different color wood quick placement coordinates(x, y, z)
        self.coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

    # 初始位置
    def initMove(self):
        print("ColorTracking Init")
        Board.setBusServoPulse(1, self.status.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)
    
    def move_first_detection(self):
        self.status.action_finish = False

        self.setBuzzer(0.1)               
        result = self.ArmIK.setPitchRangeMoving((self.status.world_X, self.status.world_Y - 2, 5), -90, -90, 0) # If the running time parameter is not filled in, the running time will be adaptive.

        if result == False:
            self.status.unreachable = True
        else:
            self.status.unreachable = False
        time.sleep(result[2]/1000) #The third item of the return parameter is the time

        self.status.start_pick_up = False
        self.status.first_move = False
        self.status.action_finish = True
    
    def move_pick_up(self):
        self.status.action_finish = False

        if not self.status.isRunning: # Stop and exit flag detection
            return True

        Board.setBusServoPulse(1, self.status.servo1 - 280, 500)  # Claws spread

        # Calculate the angle by which the gripper needs to be rotated
        servo2_angle = getAngle(self.status.world_X, self.status.world_Y, self.status.rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)
        
        if not self.status.isRunning:
            return True

        self.ArmIK.setPitchRangeMoving((self.status.world_X, self.status.world_Y, 2), -90, -90, 0)  # lower the altitude
        time.sleep(2)
        
        if not self.status.isRunning:
            return True

        Board.setBusServoPulse(1, self.status.servo1, 500)  # Gripper closed
        time.sleep(1)
        
        if not self.status.isRunning:
            return True

        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((self.status.world_X, self.status.world_Y, 12), -90, -90, 0)  # Robotic arm raised
        time.sleep(1)
        
        if not self.status.isRunning:
            return True

        #Classify and place blocks of different colors
        result = self.ArmIK.setPitchRangeMoving((self.coordinate[self.status.detect_color][0], self.coordinate[self.status.detect_color][1], 12), -90, -90, 0)   
        time.sleep(result[2]/1000)
        
        if not self.status.isRunning:
            return True

        servo2_angle = getAngle(self.coordinate[self.status.detect_color][0], self.coordinate[self.status.detect_color][1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        if not self.status.isRunning:
            return True

        self.ArmIK.setPitchRangeMoving((self.coordinate[self.status.detect_color][0], self.coordinate[self.status.detect_color][1], self.coordinate[self.status.detect_color][2] + 3), -90, -90, 0, 500)
        time.sleep(0.5)
        
        if not self.status.isRunning:
            return True

        self.ArmIK.setPitchRangeMoving((self.coordinate[self.status.detect_color]), -90, -90, 0, 1000)
        time.sleep(0.8)
        
        if not self.status.isRunning:
            return True

        Board.setBusServoPulse(1, self.status.servo1 - 200, 500)  #Open your claws and drop the object
        time.sleep(0.8)
        
        if not self.status.isRunning:
            return True    

        self.ArmIK.setPitchRangeMoving((self.coordinate[self.status.detect_color][0], self.coordinate[self.status.detect_color][1], 12), -90, -90, 0)
        time.sleep(0.8)

        self.initMove()  #Return to initial position
        time.sleep(1.5)

        self.status.detect_color = 'None'
        self.status.first_move = True
        self.status.get_roi = False
        self.status.action_finish = True
        self.status.start_pick_up = False

        return False

    def move_track(self):
        if not self.status.isRunning: #Stop and exit flag detection
            return True

        self.ArmIK.setPitchRangeMoving((self.status.world_x, self.status.world_y - 2, 5), -90, -90, 0, 20)
        time.sleep(0.02)                    
        self.status.track = False
        
        return False
    
    def move_stop(self):
        self.status._stop = False

        Board.setBusServoPulse(1, self.status.servo1 - 70, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        self.ArmIK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def move(self):

        while True:
            if self.status.isRunning:

                if self.status.first_move and self.status.start_pick_up: # When an object is first detected

                    self.move_first_detection()

                elif not self.status.first_move and not self.status.unreachable: #This is not the first time an object is detected

                    if self.status.track: #If it is the tracking stage

                        exit_loop = self.move_track()

                        if exit_loop:
                            continue

                    if self.status.start_pick_up: #If the object has not moved for a while, start gripping

                        exit_loop = self.move_pick_up()

                        if exit_loop:
                            continue
                        
                    else:
                        time.sleep(0.01)
            else:
                if self.status._stop:

                    self.move_stop()

                time.sleep(0.01)

if __name__ == '__main__':

    status = Status()
    movement = Movement(status)
    track = TrackBox(status)

    # Run child thread
    th = threading.Thread(target=movement.move)
    th.setDaemon(True)
    th.start()

    movement.initMove()
    status.start()

    target_color = ('green')
    status.setTargetColor(target_color)

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
