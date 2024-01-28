# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import track_cv as track


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
		angle, shift = track.handle_pic(image=frame, draw=draw)
		return angle, shift

sense = Sensing()
interpret = Interpreter()
# capture frames from the camera
i = 0
for frame in sense.get_stream():
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# show the frame
	angle, shift = interpret.get_vector(image, draw=True)

	cv2.imshow("Image", image)
	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	sense.rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break