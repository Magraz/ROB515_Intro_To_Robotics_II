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

start_pt = (0,0)
path_start_pt = (0,0)
path_end_pt = (0,0)
goal_pt = (0,0)

AK = ArmIK()

mouse_x, mouse_y = 0, 0

def get_mouse_xy(event,x,y,flags,param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouse_x, mouse_y = x,y

def get_wall_to_hug(img):

    frame = img.copy()

    # frame = cv2.rotate(frame, cv2.ROTATE_180)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    _, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)

    kernel = np.ones((4, 4), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(thresh, contours, 0, (255, 255, 255), 15)

    kernel = np.ones((20, 20), np.uint8)
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
    interval = 25

    for point in coordinates:
        if (i % interval) == 0:
            points_to_follow.append(point)
        i+=1

    return thinned, points_to_follow

def get_corners(img, unprocessed_img):

    frame = img.copy()

    frame = np.float32(frame)

    dst = cv2.cornerHarris(frame,2,3,0.04)
    
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)
    
    # Threshold for an optimal value, it may vary depending on the image.
    unprocessed_img[dst>0.1*dst.max()]=[0,0,255]

    return unprocessed_img

def plot_points(unprocessed_img, points):
    
    # Threshold for an optimal value, it may vary depending on the image.
    prev_point = points[0]

    cv2.putText(unprocessed_img, "Start", (start_pt[0], start_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
    cv2.putText(unprocessed_img, "Goal", (goal_pt[0], goal_pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

    for point in points:
        if len(prev_point) != 0:
            cv2.circle(unprocessed_img, (point[1], point[0]), 4, (0,0,255), -1)
            cv2.arrowedLine(unprocessed_img, (prev_point[1], prev_point[0]), (point[1], point[0]), (0,255,0), 2)
        else:
            cv2.circle(unprocessed_img, (point[1], point[0]), 10, (0,0,255), -1)

        prev_point = point

    return unprocessed_img

def order_points(points):
    point_list = []

    max_dist = 180

    point_list.append((start_pt[1], start_pt[0]))
    point_list.append((path_start_pt[1], path_start_pt[0]))
    
    prev_d = 100000
    ps_pt = ()

    for point in points:
        d_x = (point[1]-path_start_pt[0])**2
        d_y = (point[0]-path_start_pt[1])**2
        d = (d_x + d_y)**0.5

        if d < prev_d:
            ps_pt = point
            prev_d = d

    point_list.append(ps_pt)

    prev_d = 100000
    next_point = ()

    while len(point_list) != len(points):

        for point in points:

            if not(point in point_list):
                #print(f'point {point}')

                d_x = (point_list[-1][1]-point[1])**2
                d_y = (point_list[-1][0]-point[0])**2
                d = (d_x + d_y)**0.5

                #print(f'dist vals {d}')

                if d < prev_d and d < max_dist:
                    # print(f'dist {d}')
                    # print(f'prev dist {prev_d}')
                    # print(f'point list {point_list}')
                    next_point = point
                    prev_d = d
                
        if not(next_point in point_list):
            if len(next_point) > 0:
                point_list.append(next_point)
                next_point = ()
                prev_d = 100000
            
        d_x = (path_end_pt[0]-point_list[-1][1])**2
        d_y = (path_end_pt[1]-point_list[-1][0])**2
        d_to_goal = (d_x + d_y)**0.5

        if d_to_goal < 80:
            print(f'Point before goal {point_list[-1]}')
            print(f'Goal {goal_pt}')
            break
    
    #Add last points
    point_list.append((path_end_pt[1], path_end_pt[0]))
    point_list.append((goal_pt[1], goal_pt[0]))
    return point_list

def move_arm(points):
    world_xy = []

    for point in points:
        world_x, world_y = convertCoordinate(point[1], point[0], (640, 480))
        result = AK.setPitchRangeMoving((world_x, world_y, 3), -90, -90, 0)
        time.sleep(1)
        print(world_x, world_x)
    
    Board.setBusServoPulse(1, 100, 500)
    time.sleep(1)

if __name__ == '__main__':

    my_camera = Camera.Camera()
    my_camera.camera_open() 
    AK.setPitchRangeMoving((5, 5,  5), -90, -90, 0)
    Board.setBusServoPulse(1, 600, 500)
    time.sleep(2)

    set_points_window = 'set_points'

    cv2.namedWindow(set_points_window)
    cv2.setMouseCallback(set_points_window,get_mouse_xy)

    start_set = False
    path_start_set = False
    path_end_set = False
    goal_set = False

    # start_set = not start_set
    # path_start_set = not path_start_set
    # path_end_set = not path_end_set
    # goal_set = not goal_set

    # # Maze 1
    # start_pt = (258, 27)
    # path_start_pt = (206, 75)
    # path_end_pt = (507, 302)
    # goal_pt = (383, 430)

    while True:

        img = my_camera.frame

        if img is not None:

            if not start_set or not goal_set or not path_start_set or not path_end_set:

                frame = img.copy()

                cv2.imshow(set_points_window,frame)

                k = cv2.waitKey(20) & 0xFF

                if k == 27:
                    break

                elif k == ord('z'):
                    start_pt = (mouse_x, mouse_y)
                    start_set = True
                    print(f'Start Set {mouse_x, mouse_y}')
                
                elif k == ord('x'):
                    path_start_pt = (mouse_x, mouse_y)
                    path_start_set = True
                    print(f'Path Start Set {mouse_x, mouse_y}')
                
                elif k == ord('c'):
                    path_end_pt = (mouse_x, mouse_y)
                    path_end_set = True
                    print(f'Path End Set {mouse_x, mouse_y}')

                elif k == ord('v'):
                    goal_pt = (mouse_x, mouse_y)
                    goal_set = True
                    print(f'Goal Set {mouse_x, mouse_y}')
                
                if goal_set and start_set and path_start_set and path_end_set:
                    cv2.destroyWindow(set_points_window)

            else :

                frame = img.copy()
                wall_to_hug, points = get_wall_to_hug(frame)
                cv2.imshow('wall_to_hug', wall_to_hug)

                # img_with_corners = get_corners(wall_to_hug, frame)
                # cv2.imshow('img_with_corners', img_with_corners)

                point_list = order_points(points)
                img_with_points = plot_points(frame, point_list)
                cv2.imshow('img_with_points', img_with_points)

                key = cv2.waitKey(1)
                if key == 27:
                    break

    my_camera.camera_close()
    cv2.destroyAllWindows()

    time.sleep(3)

    print(point_list)

    move_arm(point_list)