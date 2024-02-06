# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import track_cv as track
import cv2
import os
import sys
from time import sleep
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(1, os.path.join(sys.path[0], '..'))

from picarx_improved import Picarx
from bus import Bus


class Sensing():
	def __init__(self):
		# initialize the camera and grab a reference to the raw camera capture
		self.camera = PiCamera()
		self.camera.resolution = (640, 480)
		self.camera.framerate = 32
		self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
		# allow the camera to warmup
		sleep(0.5)
		
	def get_image(self):
		frame = self.camera.capture(self.rawCapture, format="bgr")
		frame_cp = frame.array[:]
		self.rawCapture.truncate(0)
		return frame_cp

	def producer(self, bus: Bus, delay: int = 0):
		while True:
			bus.write(self.get_image())
			sleep(delay)

class Interpreter():
	def __init__(self):
		pass

	def get_vector(self, frame, draw=False):
		angle, shift = track.handle_pic(image=frame, draw=draw, inv_polarity=True, threshold=90)
		return [angle, shift]
	
	def consumer_producer(self, sense_bus:Bus, interpret_bus:Bus, delay:int):
		while True:
			interpret_bus.write(self.get_vector(frame=sense_bus.read()), draw=True)
			sleep(delay)

class Controller():
	def __init__(self):
		pass

	def follow_line(self, angle, shift):

		# capture frames from the camera
		norm_ang_diff = 1
		if((angle != 0) and (angle != None)):
			max_angle = 90
			angle_diff = angle-max_angle
			norm_ang_diff = -angle_diff/max_angle
		
		if((shift != 0) and (shift != None)):
			max_shift = 90
			norm_shift = shift/max_shift
			px.set_dir_servo_angle(30*(0.9*norm_shift + 0.1*norm_ang_diff))
			px.forward(35)
		else:
			px.stop()
	
	def consumer(self, interpret_bus:Bus, delay:int):
		while True:
			interpreted_arr = interpret_bus.read()
			self.follow_line(angle=interpreted_arr[0], shift=interpreted_arr[1])
			sleep(delay)

if __name__ == "__main__":
	px = Picarx()

	px.set_cam_pan_angle(2)
	px.set_cam_tilt_angle(-15)
	sleep(1)

	sensor = Sensing()
	interpreter = Interpreter()
	controller = Controller()

	sensor_delay = 0.01
	interpreter_delay = 0.01
	controller_delay = 0.05

	sensor_bus = Bus()
	interpreter_bus = Bus()

	with ThreadPoolExecutor(max_workers=6) as exec:
		eSensor = exec.submit(sensor.producer, sensor_bus, sensor_delay)
		eInterpreter = exec.submit(interpreter.consumer_producer, sensor_bus, interpreter_bus, interpreter_delay)
		eController = exec.submit(controller.consumer, interpreter_bus, controller_delay)
	
	eSensor.result()
	eInterpreter.result()
	eController.result()

	px.stop()