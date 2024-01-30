# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import track_cv as track
import cv2
import os
import sys
import time
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from picarx_improved import Picarx


class Sensing():
	def __init__(self):
		# initialize the camera and grab a reference to the raw camera capture
		self.camera = PiCamera()
		self.camera.resolution = (640, 480)
		self.camera.framerate = 32
		self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
		# allow the camera to warmup
		time.sleep(0.1)
		
	def get_stream(self):
		return self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

class Interpreter():
	def __init__(self):
		pass
	def get_vector(self, frame, draw=False):
		angle, shift = track.handle_pic(image=frame, draw=draw, inv_polarity=True, threshold=90)
		return angle, shift

if __name__ == "__main__":
	px = Picarx()
	px.set_cam_pan_angle(2)
	px.set_cam_tilt_angle(-15)
	time.sleep(1)
	sense = Sensing()
	interpret = Interpreter()
	# capture frames from the camera
	for frame in sense.get_stream():
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array

		angle, shift = interpret.get_vector(image, draw=True)
		
		norm_ang_diff = 1
		if((angle != 0) and (angle != None)):
			max_angle = 90
			angle_diff = angle-max_angle
			norm_ang_diff = -angle_diff/max_angle
		
		if((shift != 0) and (shift != None)):
			max_shift = 90
			norm_shift = shift/max_shift
			px.set_dir_servo_angle(30*(0.9*norm_shift + 0.1*norm_ang_diff))
			px.forward(50)
		else:
			px.stop()

		# if((angle != 0) and (angle != None)):
		# 	# max_angle = 90
		# 	max_shift = 90
			
		# 	# angle_diff = angle-max_angle
		# 	# norm_ang_diff = angle_diff/max_angle
		# 	norm_shift = shift/max_shift
			
		# 	# px.set_dir_servo_angle(40 * -norm_ang_diff + 30*norm_shift)
		# 	px.set_dir_servo_angle(30*norm_shift)
		# 	px.forward(40)

		# else:
		# 	px.stop()

		# show the frame
		#cv2.imshow("Image", image)
		key = cv2.waitKey(1) & 0xFF

		# clear the stream in preparation for the next frame
		sense.rawCapture.truncate(0)
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break
	px.stop()