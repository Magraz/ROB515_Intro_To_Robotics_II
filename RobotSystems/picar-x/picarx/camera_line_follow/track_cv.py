import cv2 as cv
import numpy as np
import sys
import math
import os
import logging
import geom_util as geom
import roi
import track_conf as tconf

    

#default gray threshold
T = tconf.threshold
Roi = roi.ROI()

# def balance_pic(image):
#     global T
#     ret = None
#     direction = 0
#     for i in range(0, tconf.th_iterations):

#         rc, gray = cv.threshold(image, T, 255, 0)
#         crop = Roi.crop_roi(gray)
        
#         nwh = cv.countNonZero(crop)
       
#         perc = int(100 * nwh / Roi.get_area())
#         #logging.debug(f"balance attempt, {i}, {T}, {perc}")
#         print(f"balance attempt, {i}, {T}, {perc}")
#         if perc > tconf.white_max:
#             if T > tconf.threshold_max:
#                 break
#             if direction == -1:
#                 ret = crop
#                 break
#             T += 10
#             direction = 1
#         elif perc < tconf.white_min:
#             if T < tconf.threshold_min:
#                 break
#             if  direction == 1:
#                 ret = crop
#                 break

#             T -= 10
#             direction = -1
#         else:
#             ret = crop
#             break 

#     return ret

def balance_pic(image, inv_polarity=False, threshold=50):
    polarity = 0

    if(inv_polarity):
        polarity = cv.THRESH_BINARY_INV

    _, gray = cv.threshold(image, threshold, 255, polarity)

    return Roi.crop_roi(gray)  


def adjust_brightness(img, level):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    b = np.mean(img[:,:,2])
    if b == 0:
        return img
    r = level / b
    c = img.copy()
    c[:,:,2] = c[:,:,2] * r
    return cv.cvtColor(c, cv.COLOR_HSV2BGR)

    

def prepare_pic(image, inv_polarity=False, threshold=50):
    global Roi
    global T
    height, width = image.shape[:2]

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (9, 9), 0)

    if Roi.get_area() == 0:
        Roi.init_roi(width, height)

    return balance_pic(blurred, inv_polarity, threshold), width, height


def find_main_countour(image):

    cnts, hierarchy = cv.findContours(image, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)

    C = None
    if cnts is not None and len(cnts) > 0:
         C = max(cnts, key = cv.contourArea)


    if C is None:
        return None, None

    rect = cv.minAreaRect(C)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    box = geom.order_box(box)
    return C, box

def handle_pic(image, draw=False, inv_polarity=False, threshold=50):
    
    cropped, w, h = prepare_pic(image, inv_polarity, threshold)
    if cropped is None:
        return None, None
    
    cont, box = find_main_countour(cropped)
    if cont is None:
        return None, None
    
    p1, p2 = geom.calc_box_vector(box)
    if p1 is None:
        return None, None
    
    angle = geom.get_vert_angle(p1, p2, w, h)
    shift = geom.get_horz_shift(p1[0], w)

    if draw:
        cv.drawContours(image, [cont], -1, (0,0,255), 3)
        cv.drawContours(image,[box],0,(255,0,0),2)
        cv.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 0), 3)
        
        cv.circle(image,(int(p1[0]), int(p1[1])), 10, (255,0, 0), -1)
        cv.circle(image,(int(p2[0]), int(p2[1])), 10, (255,255, 0), -1)

        msg_a = "Angle {0}".format(int(angle))
        msg_s = "Shift {0}".format(int(shift))

        cv.putText(image, msg_a, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv.putText(image, msg_s, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        
    return angle, shift

def prepare_pic2(image):
    height, width = image.shape[:2]
    crop = image[3 * height / 4: height, width / 4:3 * width/ 4]
    crop = adjust_brightness(crop, tconf.brightness)

    gray = cv.cvtColor(crop, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (9, 9), 0)

    rc, gray = cv.threshold(blurred, tconf.threshold, 255, 0)
    return gray, width / 2, height / 4



def handle_pic2(path, fout = None, show = False):
    image = cv.imread(path)
    if image is None:
        logging.warning(("File not found", path))
        return None, None
    height, width = image.shape[:2]
    cropped, w, h = prepare_pic2(image)
    if cropped is None:
        return None, None
    cont, box = find_main_countour(cropped)
    if cont is None:
        return None, None
    
    p1, p2 = geom.calc_box_vector(box)
    if p1 is None:
        return None, None

    angle = geom.get_vert_angle(p1, p2, w, h)
    shift = geom.get_horz_shift(p1[0], w)

    draw = fout is not None or show

    if draw:
        w_offset = (width - w) / 2
        h_offset = (height - h)
        dbox = geom.shift_box(box, w_offset, h_offset)

        cv.drawContours(image,[dbox],0,(255,0,0),2)
        dp1 = (p1[0] + w_offset, p1[1] + h_offset)
        dp2 = (p2[0] + w_offset, p2[1] + h_offset)
        cv.line(image, dp1, dp2, (0, 255, 0), 3)
        msg_a = "Angle {0}".format(int(angle))
        msg_s = "Shift {0}".format(int(shift))

        cv.putText(image, msg_a, (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv.putText(image, msg_s, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    if fout is not None:
        cv.imwrite(fout, image)

    if show:    
        cv.imshow("Image", image)
        cv.waitKey(0)
    return angle, shift




if __name__ == '__main__':
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    pic = "1"
    if len(sys.argv) > 1:
        pic = sys.argv[1]

    """
    fname = "photos/" + pic + ".jpg"
    angle, shift = handle_pic2(fname, fout="out.jpg", show=True)
    print "Angle", angle, "Shift", shift
    """
    for f in os.listdir("test"):
        a, s = handle_pic("test/" + f, show=True)
                                   