import picamera
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import serial
import sys
import numpy as np
import cv2

class MyOutput(object):
    def write(self, buf):
        # write will be called once for each frame of output. buf is a bytes
        # object containing the frame data in YUV420 format; we can construct a
        # numpy array on top of the Y plane of this data quite easily:
        y_data = np.frombuffer(
            buf, dtype=np.uint8, count=(448*320)).reshape((320, 448))

        global count
        count = count + 1
        # new = time.time()
        # global oldtime
        # last = new - oldtime
        # oldtime = new
        # print(last*1000)
        # do whatever you want with the frame data here... I'm just going to
        # print the maximum pixel brightness:
        # print(y_data[:360, :480].max())

    def flush(self):
        global count
        print(count)
        # this will be called at the end of the recording; do whatever you want
        # here
        pass

with picamera.PiCamera(
        sensor_mode=7,
        resolution='400x300',
        framerate=90) as camera:
    time.sleep(2) # let the camera warm up and set gain/white balance
    output = MyOutput()

    oldtime = time.time()
    count = 0

    start = time.time()
    for i in range(1,5):
        camera.start_recording(output, 'rgb')
        camera.wait_recording(2) # record 10 seconds worth of data
        camera.stop_recording()

    end = time.time()

    print(end - start)