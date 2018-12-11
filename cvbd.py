
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import math
from simple_pid import PID
import serial
from imutils.video import WebcamVideoStream
from imutils.video import FPS

#import threading
printstring = ""
jogcancel = False
speedold =0
jog_command = ""
dircheck = 0
enables = False
PID_Parameters = [180,2,30]#[250,50,80]
anglethresh = 70
andglethreshhigh = 110
anglemax = 10
angleoffset = .2#0.8
bufcnt = 0
sendthresh =100
sendold = 0
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

port = 'COM3'
baud = 500000#230400#115200

s = serial.Serial(port, baud, timeout=0)



blueLower = (101,103,51)#(29, 86, 6)
blueUpper = (222,224,212)#(64, 255, 255)
greenLower = (33, 75, 33)
greenUpper = (98, 255, 255)

skip_every = 4
count = 0

if not args.get("video", False):
	vs = VideoStream(src=0).start()


else:
	pass 


time.sleep(2.0)

framecounter = 0

pid = PID(PID_Parameters[0], PID_Parameters[1], PID_Parameters[2], setpoint=90,output_limits=(-1000,1000),sample_time = 0.01)
pid.sample_time = 0.1
angle =0.0
pidout = 0
grbl_out=""
#s = serial.Serial('com3',115200)
##s.write("\r\n\r\n".encode())
##time.sleep(2)   # Wait for grbl to initialize
##s.flushInput()  # Flush startup text in serial input
		# Collect events until released
scnt = 0
		
def Cos2Lt(o,a):
	if o > 0 and a > 0:
		theta = math.atan(o/a)
	else:
		theta = 0
	#print(a)
	return math.degrees(theta)
	#print(math.degrees(theta) +90.0)
enable = False
speed = 0
dirold=0	
# keep looping
def PIDVALP(kP):
	PID_Parameters[0] = kP#cv2.getTrackbarPos('kP','CV PID Settings')
	#print(pid.kp)
	#pid.kp = PID_Parameters[0]
def PIDVALI(kI):
	PID_Parameters[1] = kI
	#pid.ki = PID_Parameters[1]
def PIDVALD(kD):
	PID_Parameters[2] = kD
	#pid.kd = PID_Parameters[2]	
cv2.namedWindow('CV PID Settings')
cv2.createTrackbar('kP', 'CV PID Settings', 000, 1000, PIDVALP)	
cv2.setTrackbarPos('kP', 'CV PID Settings',PID_Parameters[0])

cv2.createTrackbar('kI', 'CV PID Settings', 000, 1000, PIDVALI)	
cv2.setTrackbarPos('kI', 'CV PID Settings',PID_Parameters[1])

cv2.createTrackbar('kD', 'CV PID Settings', 000, 1000, PIDVALD)	
cv2.setTrackbarPos('kD', 'CV PID Settings',PID_Parameters[2])


fps = FPS().start()

while True:
	# grab the current frame
	#if count % skip_every == 0:
		
	frame = vs.read()

	
	frame = frame[1] if args.get("video", False) else frame
	

	if frame is None:
		break
	# resize the frame, blur it, and convert it to the HSV
	# color space
	
	frame = imutils.resize(frame, width=600)
	ht, wh = frame.shape[:2]
	bcen = (int(wh/2),ht)
			
	

	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	maskgreen = cv2.inRange(hsv, greenLower, greenUpper)
	maskgreen = cv2.erode(maskgreen, None, iterations=2)
	maskgreen = cv2.dilate(maskgreen, None, iterations=2)
	
	maskblue = cv2.inRange(hsv, blueLower, blueUpper)
	maskblue = cv2.erode(maskblue, None, iterations=2)
	maskblue = cv2.dilate(maskblue, None, iterations=2)

	cntsgreen = cv2.findContours(maskgreen.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cntsgreen = cntsgreen[0] if imutils.is_cv2() else cntsgreen[1]
	centergreen = None
	
	cntsblue = cv2.findContours(maskblue.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
	cntsblue = cntsblue[0] if imutils.is_cv2() else cntsblue[1]
	centerblue = None

# only proceed if at least one contour was found

	if len(cntsgreen) > 0:

		cgreen = max(cntsgreen, key=cv2.contourArea)
		((xgreen, ygreen), radiusgreen) = cv2.minEnclosingCircle(cgreen)
		Mgreen = cv2.moments(cgreen)
		centergreen = (int(Mgreen["m10"] / Mgreen["m00"]), int(Mgreen["m01"] / Mgreen["m00"]))


		if radiusgreen > 10:

			cv2.circle(frame, (int(xgreen), int(ygreen)), int(radiusgreen),
				(0, 255, 255), 2)
			cv2.circle(frame, centergreen, 5, (0, 0, 255), -1)

	if len(cntsblue) > 0:
		cblue = max(cntsblue, key=cv2.contourArea)
		((xblue, yblue), radiusblue) = cv2.minEnclosingCircle(cblue)
		Mblue = cv2.moments(cblue)
		centerblue = (int(Mblue["m10"] / Mblue["m00"]), int(Mblue["m01"] / Mblue["m00"]))
		
	
		if radiusblue > 10:
			cv2.circle(frame, (int(xblue), int(yblue)), int(radiusblue),
					(0, 255, 255), 2)
			cv2.circle(frame, centerblue, 5, (0, 0, 255), -1)
		
	if len(cntsgreen) > 0 and len(cntsblue) > 0:
		if radiusgreen >10 and radiusblue>10:
			cv2.line(frame,centergreen,centerblue,(0,0,255),2)
			cv2.line(frame,(centergreen[0],centerblue[1]),(centerblue[0],centerblue[1]),(0,255,0),2)
			cv2.line(frame,(centergreen[0],centergreen[1]),(centergreen[0],centerblue[1]),(255,0,0),2)
			angle = Cos2Lt(abs(centergreen[0]-centerblue[0]),abs(centergreen[1]-centerblue[1]))
			angle += angleoffset
			
			if centergreen[0] > centerblue[0]:
				angle = 90 - angle
			else:
				angle +=90
		
			if angle > anglethresh and angle < andglethreshhigh:
				pidout=pid(angle)
			else:
				pidout=0
				
			cv2.putText(frame,str(angle)[:5]+"deg",(int(centergreen[0]),int(centergreen[1])),cv2.FONT_HERSHEY_PLAIN,1.5,(255,255,255),1)
				
			if pidout <= 0:
				dir=0
				speed = int(abs(pidout))
				
			else:
				dir=1
				speed = int(pidout)+1000
			
	
		
		
		if enable:

			#if framecounter % 2 == 0:	
			
			if s.out_waiting > 10:
				reset_output_buffer()
			else:#Send Update if PID has changed by threshold value
				if(speed+sendthresh) > sendold or  (speed-sendthresh) < sendold: 
					sendold = speed
					s.write((str(speed)+"\n").encode())
			f.write(str(angle)+","+str(pidout)+","+str(speed)+","+str(fps.fps())+","+str(fps.elapsed())+"\n")	
				
		else:
			if framecounter % 20 == 0:
				s.write("0\n".encode())
				s.flush()
				s.flushOutput()
				
			
			#PID_Parameters[0] = cv2.getTrackbarPos('kP', 'CV PID Settings')
			#PID_Parameters[1]= cv2.getTrackbarPos('ki', 'CV PID Settings')
			#PID_Parameters[2]	= cv2.getTrackbarPos('kd', 'CV PID Settings')		
	else:#Not Found
		angle = 0
		pidout = 0
		speed = 0
		dir = 0
	fps.update()
	fps.stop()	
	print("Angle(deg): {:+.2f}, PID: Kp{}, Ki{:.3f}, Kd{:.3f}, PIDOut{:+3.3f}, Speed_Command: {:+5}, Direction: {}, fps:{:.2f} , time:{:.2f}\r".format(angle,pid.Kp,pid.Ki,pid.Kd,pidout,speed,dir,fps.fps(),fps.elapsed()),end="")
	pid.tunings = (PID_Parameters[0], PID_Parameters[1], PID_Parameters[2])
	cv2.imshow("CV PID Pendulum", frame)
	
	
	
	key = cv2.waitKey(1) & 0xFF
	
	
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		s.write("0\n".encode())
		s.flush()	
		s.close()
		f.close()
		fps.stop()
		break
	if key == ord("s") and enable == False:
		f = open("data.txt","w")
		f.write("angle,pidout,speed,fps,elapsed\n")	
		enable = True
		#pid.auto_mode = True
		#print("enabled")

	if key == ord("k"):
		
		s.write("0\n".encode())
		s.flush()
		s.reset_output_buffer()
		s.reset_input_buffer()
		
		enable = False
		f.close()
		
		#pid.auto_mode = False
		
		#pid.kp = PID_Parameters[0]
		#pid.ki = PID_Parameters[1]
		#pid.kd = PID_Parameters[2]
		#print("Disabled")
	if key == ord("p"):
		pid.setpoint = angle
		print(pid.setpoint)
			
	
	framecounter +=1
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
s.close()