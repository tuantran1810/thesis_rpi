import sys
import threading
import time
from time import sleep
import Queue
import numpy as np

import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera

import serial
import UARTCommand

import cv2

usleep = lambda x: time.sleep(x/1000000.0)
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Define variables
BAP_IMAGE_SIZE = 320 # Square image
BAP_IMAGE_SIZE_STR = '320x320'

BAP_THREADHOLD_VALUE = 50   # Value to put in threshold function, assign 255 to pixels that have lower value than this and 0 to the higher
BAP_RECORDING_TIME = 10    # Time for a recording sequence

BAP_MIN_EXPECTED_BALL_SIZE = 200    # Place camera about 50cm higher than the plate
BAP_MAX_EXPECTED_BALL_SIZE = 500
BAP_BALL_CENTER_OFFSET = 10

BAP_CAMERA_WARMUP_TIME = 2
BAP_CAMERA_MODE = 7
BAP_CAMERA_FRAMERATE = 90
BAP_CAMERA_COLOR_MODE = 'yuv'

BAP_QUEUE_SIZE = 10
BAP_QUEUE_MESS_SIZE = 50

BAP_OriginalImage = np.zeros((BAP_IMAGE_SIZE,BAP_IMAGE_SIZE))
BAP_OriginalImage_Mutex = threading.Lock()

BAP_BlurImage = np.zeros((BAP_IMAGE_SIZE,BAP_IMAGE_SIZE))
BAP_BlurImage_Mutex = threading.Lock()

BAP_PlateImage = np.zeros((BAP_IMAGE_SIZE,BAP_IMAGE_SIZE))
BAP_NewPlate_Flag = 0
BAP_PlateImage_Mutex = threading.Lock()

BAP_WarpPerspectiveMatrix = np.zeros((3,3))
BAP_WarpPerspectiveMatrix_Mutex = threading.Lock()
BAP_WarpPerspectiveMatrix_Sem = threading.Semaphore(1)
BAP_WarpPerspectiveMatrix_Sem.acquire()

BAP_RecvMsgQueue = Queue.Queue(BAP_QUEUE_SIZE)
BAP_RecvMsgQueueMutex = threading.Lock()
BAP_RecvMsgQueueSem = threading.Semaphore(1)

BAP_SendMsgQueue = Queue.Queue(BAP_QUEUE_SIZE)
BAP_SendMsgQueueMutex = threading.Lock()
BAP_SendMsgQueueSem = threading.Semaphore(1)

BAP_CmdBuffer = UARTCommand.CommandBuffer(BAP_QUEUE_MESS_SIZE, BAP_RecvMsgQueue, BAP_RecvMsgQueueMutex, BAP_RecvMsgQueueSem)

count = 0

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Image getting class 
class BAP_ImageGet_Handler(object):
    global BAP_IMAGE_SIZE, BAP_THREADHOLD_VALUE
    def write(self, buf):
        global BAP_BlurImage
        global BAP_BlurImage_Mutex

        global BAP_OriginalImage
        global BAP_OriginalImage_Mutex

        global BAP_PlateImage
        global BAP_PlateImage_Mutex

        global BAP_WarpPerspectiveMatrix
        global BAP_WarpPerspectiveMatrix_Mutex
        global BAP_WarpPerspectiveMatrix_Sem

        global count

        image = np.fromstring(buf, dtype=np.uint8, count=BAP_IMAGE_SIZE*BAP_IMAGE_SIZE).reshape(BAP_IMAGE_SIZE, BAP_IMAGE_SIZE)
        BAP_OriginalImage_Mutex.acquire()
        BAP_OriginalImage = image.copy()
        BAP_OriginalImage_Mutex.release()

        _,light_threshold = cv2.threshold(image, BAP_THREADHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)

        BAP_BlurImage_Mutex.acquire()
        BAP_BlurImage = cv2.blur(light_threshold,(3,3))
        blurimg = BAP_BlurImage.copy()
        BAP_BlurImage_Mutex.release()

        _, contours, hierarchy = cv2.findContours(blurimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if(len(contours) != 0):
            #find biggest area
            BiggestArea = 0
            BiggestAreaPos = 0
            pos = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if (area > BiggestArea):
                    BiggestArea = area
                    BiggestAreaPos = pos
                pos += 1

            BiggestContour = contours[BiggestAreaPos]

            epsilon = 0.1*cv2.arcLength(BiggestContour,True)
            approx = cv2.approxPolyDP(BiggestContour,epsilon,True)

            rect_points = np.zeros((4,2), dtype = "float32")
            xplusy = np.zeros((4,1), dtype = "float32")
            xminusy = np.zeros((4,1), dtype = "float32")
            if(len(approx) == 4):
                for i in range(0,4):
                    xplusy[i] = np.float32(approx[i][0][0]) + np.float32(approx[i][0][1])
                    xminusy[i] = np.float32(approx[i][0][0]) - np.float32(approx[i][0][1])

                rect_top_left = np.argmin(xplusy)
                rect_bottom_right = np.argmax(xplusy)
                rect_top_right = np.argmax(xminusy)
                rect_bottom_left = np.argmin(xminusy)

                if (rect_top_left != rect_bottom_right and rect_top_left != rect_top_right and rect_top_left != rect_bottom_left and
                    rect_bottom_right != rect_top_right and rect_bottom_right != rect_bottom_left and rect_top_right != rect_bottom_left):

                    rect_points[0][0] =  np.float32(approx[rect_top_left][0][0])
                    rect_points[0][1] =  np.float32(approx[rect_top_left][0][1])
                    rect_points[1][0] =  np.float32(approx[rect_top_right][0][0])
                    rect_points[1][1] =  np.float32(approx[rect_top_right][0][1])
                    rect_points[2][0] =  np.float32(approx[rect_bottom_right][0][0])
                    rect_points[2][1] =  np.float32(approx[rect_bottom_right][0][1])
                    rect_points[3][0] =  np.float32(approx[rect_bottom_left][0][0])
                    rect_points[3][1] =  np.float32(approx[rect_bottom_left][0][1])

                    dst = np.array([[0,0],[319,0],[319,319],[0,319]],np.float32)

                    BAP_WarpPerspectiveMatrix_Mutex.acquire()
                    BAP_WarpPerspectiveMatrix = cv2.getPerspectiveTransform(rect_points,dst)
                    BAP_WarpPerspectiveMatrix_Mutex.release()
                    BAP_WarpPerspectiveMatrix_Sem.release()
                else:
                    print "ERROR: Invalid Rectangle"
            else:
                print "ERROR: No plate found, approx len = ", len(approx)
        else:
            print "ERROR: No contour detected"

        count += 1
    def flush(self):
        pass

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Image image getting thread
class BAP_ImageGet_Thread(threading.Thread):
    global BAP_CAMERA_MODE, BAP_IMAGE_SIZE_STR, BAP_CAMERA_FRAMERATE
    def __init__(self, Mode = BAP_CAMERA_MODE, resolution = BAP_IMAGE_SIZE_STR, framerate = BAP_CAMERA_FRAMERATE):
      threading.Thread.__init__(self)
      self.Mode = Mode
      self.resolution = resolution
      self.framerate = framerate

    def run(self):
        with PiCamera(sensor_mode = self.Mode, resolution = self.resolution, framerate = self.framerate) as camera:
            global BAP_CAMERA_WARMUP_TIME, BAP_CAMERA_COLOR_MODE, count
            time.sleep(BAP_CAMERA_WARMUP_TIME)
            output = BAP_ImageGet_Handler()
            while True:
                camera.start_recording(output, BAP_CAMERA_COLOR_MODE)
                camera.wait_recording(10)
                camera.stop_recording()
                print(count)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Image Plate Detecting thread
class BAP_PlateDetecting_Thread(threading.Thread):
    global BAP_IMAGE_SIZE
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global count

        global BAP_BlurImage
        global BAP_BlurImage_Mutex

        global BAP_PlateImage
        global BAP_PlateImage_Mutex
        global BAP_NewPlate_Flag

        global BAP_WarpPerspectiveMatrix
        global BAP_WarpPerspectiveMatrix_Mutex
        global BAP_WarpPerspectiveMatrix_Sem

        while True:
            M = None

            BAP_WarpPerspectiveMatrix_Sem.acquire()
            BAP_WarpPerspectiveMatrix_Mutex.acquire()
            M = BAP_WarpPerspectiveMatrix.copy()
            BAP_WarpPerspectiveMatrix_Mutex.release()

            if M is not None:
                BAP_BlurImage_Mutex.acquire()
                src = BAP_BlurImage.copy()
                BAP_BlurImage_Mutex.release()

                src = cv2.warpPerspective(src,M,(BAP_IMAGE_SIZE,BAP_IMAGE_SIZE))
                BAP_PlateImage_Mutex.acquire()
                BAP_PlateImage = src.copy()
                BAP_NewPlate_Flag = 1
                BAP_PlateImage_Mutex.release()

            # count += 1

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Find Ball position and pass it through Kalman filter
class BAP_BallPos_Thread(threading.Thread):
    global BAP_MIN_EXPECTED_BALL_SIZE, BAP_MAX_EXPECTED_BALL_SIZE, BAP_IMAGE_SIZE, BAP_BALL_CENTER_OFFSET
    def __init__(self, SendMsgQueue, SendLock, SendSem):
        threading.Thread.__init__(self)
        self.SendMsgQueue = SendMsgQueue
        self.SendLock = SendLock
        self.SendSem = SendSem

    def run(self):
        global count

        global BAP_PlateImage
        global BAP_PlateImage_Mutex
        global BAP_NewPlate_Flag

        kalman = cv2.KalmanFilter(4, 2, 0)
        kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        kalman.transitionMatrix = np.array([[1,0,0.02,0],[0,1,0,0.02],[0,0,1,0],[0,0,0,1]],np.float32)
        kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 10
        kalman.measurementNoiseCov = np.array([[1,0],[0,1]],np.float32) * 0.00003

        BallX = 0
        BallY = 0

        RealBallX = 0
        RealBallY = 0

        test0 = 0

        while True:
            BAP_PlateImage_Mutex.acquire()
            img = BAP_PlateImage.copy()
            new_flag = BAP_NewPlate_Flag
            BAP_PlateImage_Mutex.release()

            if new_flag:
                _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if(len(contours) != 0):
                    # find expected ball area
                    num = 0

                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        if(area > BAP_MIN_EXPECTED_BALL_SIZE and area < BAP_MAX_EXPECTED_BALL_SIZE):
                            BallCnt = cnt
                            M = cv2.moments(BallCnt)
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])

                            if ((cX > BAP_BALL_CENTER_OFFSET) and (cX < BAP_IMAGE_SIZE - BAP_BALL_CENTER_OFFSET) and 
                                (cY > BAP_BALL_CENTER_OFFSET) and (cY < BAP_IMAGE_SIZE - BAP_BALL_CENTER_OFFSET)):
                                num += 1
                                BallX = cX
                                BallY = cY

                            if num == 2:
                                break

                    if(num == 1):
                        RealBallX = BallX
                        RealBallY = BallY
                    elif (num == 0):
                        print "No ball found"
                    else:
                        print "There are more than 1 object like ball on the plate"
                else:
                    print "Nothing found on plate"

            kalman.correct(np.array([[np.float32(RealBallX)],[np.float32(RealBallY)]]))
            tmp = kalman.predict()

            test1 = 70
            UARTStr = '[BPos][' + '%03d '%test0 + '%03d]'%test1
            length = len(UARTStr) + 14
            lenStr = 'UUUUUUUUUU[' + '%02d'%length + ']'
            SendStr = lenStr + UARTStr

            test0 += 1
            if(test0 == 1000):
                test0 = 0

            self.SendLock.acquire()
            self.SendMsgQueue.put(SendStr.ljust(40, '\x55'))
            self.SendLock.release()
            self.SendSem.release()
            # usleep(20000)

            # BAP_PlateImage_Mutex.acquire()
            # cv2.circle(BAP_PlateImage, (tmp[0], tmp[1]), 7, (255, 255, 255), -1)
            # BAP_PlateImage_Mutex.release()
            
            # count += 1

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Display thread (just use 1 thread to display)
class BAP_ImageDisplay_Thread(threading.Thread):
    def __init__(self, name, imagenum):
        threading.Thread.__init__(self)
        self.name = name
        self.imagenum = imagenum
        self.mutex = None

    def run(self):
        while True:
            if(self.imagenum == 0):
                image = BAP_OriginalImage
                self.mutex = BAP_OriginalImage_Mutex
            elif(self.imagenum == 1):
                image = BAP_BlurImage
                self.mutex = BAP_BlurImage_Mutex
            elif(self.imagenum == 2):
                image = BAP_PlateImage
                self.mutex = BAP_PlateImage_Mutex
            else:
                image = None
                self.mutex = None

            if ((self.name is not None) and (self.mutex is not None) and (image is not None)):
                self.mutex.acquire()
                cv2.imshow(self.name, image)
                self.mutex.release()
                cv2.waitKey(100)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Main code

BAP_RecvThread = UARTCommand.RecvThread(BAP_CmdBuffer)
BAP_SendThread = UARTCommand.SendThread(BAP_SendMsgQueue, BAP_SendMsgQueueMutex, BAP_SendMsgQueueSem)

BAP_ImageGet_Thread = BAP_ImageGet_Thread()
BAP_PlateDetecting_Thread = BAP_PlateDetecting_Thread()
BAP_BallPos_Thread = BAP_BallPos_Thread(BAP_SendMsgQueue, BAP_SendMsgQueueMutex, BAP_SendMsgQueueSem)
BAP_OriginalImageDisplay_Thread = BAP_ImageDisplay_Thread("Original", 0)
BAP_BlurImageDisplay_Thread = BAP_ImageDisplay_Thread("Blur", 1)
BAP_PlateImageDisplay_Thread = BAP_ImageDisplay_Thread("Plate", 2)

BAP_RecvThread.start()
BAP_SendThread.start()

BAP_ImageGet_Thread.start()
BAP_PlateDetecting_Thread.start()
BAP_BallPos_Thread.start()
# BAP_OriginalImageDisplay_Thread.start()
# BAP_BlurImageDisplay_Thread.start()
# BAP_PlateImageDisplay_Thread.start()
