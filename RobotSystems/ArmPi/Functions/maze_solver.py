import sys
sys.path.append('/home/pi/ROB515_Intro_To_Robotics_II/RobotSystems/ArmPi')
import cv2
import numpy as np
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

CELL_SIZE=1
INIT_POINT = (278,58)
FINAL_POINT = (348,422)

AK = ArmIK()

def get_wall_to_hug(img):

    frame = img.copy()

    # frame = cv2.rotate(frame, cv2.ROTATE_180)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    _, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)

    kernel = np.ones((4, 4), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(thresh, contours, 0, (255, 255, 255), 15)

    kernel = np.ones((10, 10), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    # kernel = np.ones((20, 20), np.uint8)
    # dilation = cv2.dilate(thresh, kernel, iterations=1)

    # kernel = np.ones((10, 10), np.uint8)
    # erosion = cv2.erode(dilation, kernel, iterations=1)
    	
    # diff = cv2.absdiff(dilation,erosion)

    # Create the sharpening kernel 
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]) 
    
    # Sharpen the image 
    sharpened_image = cv2.filter2D(thresh, -1, kernel) 

    # then skeletonize thresh in range 0 to 1 as float32, then convert back to uint8
    # skeleton = skimage.morphology.skeletonize(sharpened_image.astype(np.float32)/255)
    # skeleton = (255*skeleton).clip(0,255).astype(np.uint8)

    thinned = cv2.ximgproc.thinning(sharpened_image)

    indices = np.where(thinned == [255])

    coordinates = zip(indices[0], indices[1])

    points_to_follow = []
    i = 0
    interval = 10

    for point in coordinates:
        if (i % interval) == 0:
            points_to_follow.append(point)
        i+=1

    return thinned, points_to_follow[::-1]

def get_corners(img, unprocessed_img):

    frame = img.copy()

    frame = np.float32(frame)

    dst = cv2.cornerHarris(frame,2,3,0.04)
    
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)
    
    # Threshold for an optimal value, it may vary depending on the image.
    unprocessed_img[dst>0.02*dst.max()]=[0,0,255]

    return unprocessed_img

def plot_points(unprocessed_img, points):
    
    # Threshold for an optimal value, it may vary depending on the image.
    prev_point = []
    cv2.circle(unprocessed_img, (INIT_POINT[0], INIT_POINT[1]), 10, (255,0,0), -1)
    cv2.circle(unprocessed_img, (FINAL_POINT[0], FINAL_POINT[1]), 10, (255,0,0), -1)
    for point in points:
        #print(point[0], point[1])
        #unprocessed_img[point[0], point[1]]=[0,0,255]

        if len(prev_point) != 0:
            cv2.circle(unprocessed_img, (point[1], point[0]), 4, (0,0,255), -1)
            cv2.line(unprocessed_img, (prev_point[1], prev_point[0]), (point[1], point[0]), (0,255,0), 2)
        else:
            cv2.circle(unprocessed_img, (point[1], point[0]), 10, (0,0,255), -1)

        prev_point = point

    return unprocessed_img

def order_points(points):
    point_list = []

    #Find point closest to start
    prev_d = 100000
    start_point = ()

    for point in points:
        d_x = (point[1]-INIT_POINT[0])**2
        d_y = (point[0]-INIT_POINT[1])**2
        d = (d_x + d_y)**0.5

        if d < prev_d:
            start_point = point

        prev_d = d

    prev_d = 100000
    point_list.append(start_point)

    next_point = []
    while len(point_list) != len(points):

        for point in points:

            if not(point in point_list):

                d_x = (point_list[-1][1]-point[1])**2
                d_y = (point_list[-1][0]-point[0])**2
                d = (d_x + d_y)**0.5

                if d < prev_d and d < 70:
                    next_point = point

                prev_d = d

        if len(next_point) > 0:
            point_list.append(next_point)
    
    #Add last point
    point_list.append([FINAL_POINT[1], FINAL_POINT[0]])

    return point_list

def move_arm(points):
    world_xy = []

    for point in points:
        world_x, world_y = convertCoordinate(point[1], point[0], (640, 480))
        result = AK.setPitchRangeMoving((world_x, world_y, 4), -90, -90, 0)
        time.sleep(1)
        print(world_x, world_x)

if __name__ == '__main__':

    my_camera = Camera.Camera()
    my_camera.camera_open() 
    AK.setPitchRangeMoving((5, 5,  5), -90, -90, 0)
    Board.setBusServoPulse(1, 600, 500)
    time.sleep(2)

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            wall_to_hug, points = get_wall_to_hug(frame)
            # img_with_corners = get_corners(wall_to_hug, frame)
            point_list = order_points(points)
            img_with_points = plot_points(frame, point_list)
            cv2.imshow('Frame', wall_to_hug)
            cv2.imshow('Frame2', img_with_points)
            key = cv2.waitKey(1)
            if key == 27:
                # ArmIK.setPitchRangeMoving((-15, 6,  3), -90, -90, 0, 1000)
                # Board.setBusServoPulse(1, servo1 - 280, 500)
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

    # move_arm(point_list)