# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
import numpy as np
import cv2

def empty(x):
    pass
#cv2.createTrackbar('HH','Palette',0,255,empty)
#cv2.createTrackbar('HL','Palette',0,255,empty)
#cv2.createTrackbar('SH','Palette',0,255,empty)
#cv2.createTrackbar('SL','Palette',0,255,empty)
#cv2.createTrackbar('VH','Palette',0,255,empty)
#cv2.createTrackbar('VL','Palette',0,255,empty)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (480, 320)
camera.framerate = 90
rawCapture = PiRGBArray(camera, size=(480, 320))

# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
   #  hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

   # # HH = cv2.getTrackbarPos('HH','Palette')
   # # HL = cv2.getTrackbarPos('HL','Palette')
   # # SH = cv2.getTrackbarPos('SH','Palette')
   # # SL = cv2.getTrackbarPos('SL','Palette')
   # # VH = cv2.getTrackbarPos('VH','Palette')
   # # VL = cv2.getTrackbarPos('VL','Palette')

   #  HH = 120
   #  HL = 95
   #  SH = 255
   #  SL = 200
   #  VH = 255
   #  VL = 100

   #  lower = np.array ([HH,SH,VH])
   #  upper = np.array ([HL,SL,VL])

   #  threshold = cv2.inRange(hsv,upper,lower)
   #  threshold = cv2.medianBlur(threshold, 5)

   #  kernel = np.ones((5,5), np.uint8)
   #  erosion = cv2.erode(threshold, kernel, iterations = 1)
   #  dilation = cv2.dilate(erosion, kernel, iterations = 1)
   #  dilation, contours, hierchy = cv2.findContours(dilation.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

   #  BiggestArea = 0
   #  BiggestAreaCount = 0
    
   #  for n in range(0,len(contours)):
   #      ct = contours[n]
   #      print('ctArea[%d] = %d\n' %(n, cv2.contourArea(ct)))
   #      if cv2.contourArea(ct) > BiggestArea:
   #          BiggestArea = cv2.contourArea(ct)
   #          BiggestAreaCount = n
   #  print('\n')

   #  if (BiggestArea != 0):
   #      cv2.drawContours(image, contours, BiggestAreaCount, (0,255,0), 3)
   #      (x,y), radius = cv2.minEnclosingCircle(contours[BiggestAreaCount])
   #      center = (int(x),int(y))
   #      radius = int(radius)
   #      cv2.circle(image,center,radius,(255,0,0),2)

   #  cv2.imshow("Palette", dilation)
    cv2.imshow("Image", image)
    
    # print(last)
    # print("\n")
    # key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    # if key == ord("q"):
    #     break