#!/usr/bin/python

import Queue
import threading
import serial
import time


# START_OF_TEXT_D = '\x02'
# END_OF_TEXT_D = '\x03'
START_OF_TEXT_D = '2'
END_OF_TEXT_D = '3'
QUEUE_SIZE = 100
QUEUE_MESS_SIZE = 1000
SERIAL_DEVICE = '/dev/ttyAMA0'
SERIAL_SPEED = 115200

usleep = lambda x: time.sleep(x/1000000.0)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Buffer for receiving UART command
class CommandBuffer:
    def __init__(self, BSize, CmdQueue, CmdQueueLock, CmdQueueSem):
        self.Queue = CmdQueue
        self.QueueLock = CmdQueueLock
        self.QueueSem = CmdQueueSem
        self.BufferArr = ''
        self.isEmpty = 1
        self.isFull = 0
        self.WrtPtr = 0
        self.Size = BSize
        self.CmdStart = 0
        self.CmdStartPtr = 0
        self.CmdEnd = 0
        self.CmdEndPtr = 0

    def isFull(self):
        return self.isFull

    def isEmpty(self):
        return self.isEmpty

    def WriteBuffer(self, char):
        if self.isFull == 0:
            self.BufferArr += char
            self.isEmpty = 0

            if self.WrtPtr == self.Size:
                self.isFull = 1

            if (char == START_OF_TEXT_D):
                if (self.CmdStart == 0):
                    self.CmdStartPtr = self.WrtPtr
                    self.CmdStart = 1
                else:
                    self.ResetBuffer()

            if (char == END_OF_TEXT_D):
                if (self.CmdEnd == 0 and self.CmdStart == 1):
                    self.CmdEndPtr = self.WrtPtr
                    self.CmdEnd = 1
                else:
                    self.ResetBuffer()

            if (self.CmdStart == 1 and self.CmdEnd == 1):
                self.ProcessCommand()
            else:
                self.WrtPtr += 1

    def ResetBuffer(self):
        self.BufferArr = ''
        self.isEmpty = 1
        self.isFull = 0
        self.WrtPtr = 0
        self.CmdStart = 0
        self.CmdStartPtr = 0
        self.CmdEnd = 0
        self.CmdEndPtr = 0
        self.CmdReady = 0

    def ReadBuffer(self):
        return self.BufferArr

    def ProcessCommand(self):
        if (self.CmdStart == 1) and (self.CmdEnd == 1):
            if (self.CmdStartPtr < self.CmdEndPtr):
                self.QueueLock.acquire()
                self.Queue.put(self.BufferArr[self.CmdStartPtr+1:self.CmdEndPtr])
                self.QueueLock.release()
                self.QueueSem.release()
        self.ResetBuffer()

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Command processing thread
class StringProcThread (threading.Thread):

   def __init__(self, MsgQueue, Lock, Sem):
      threading.Thread.__init__(self)
      self.MsgQueue = MsgQueue
      self.Lock = Lock
      self.Sem = Sem

   def run(self):
      while 1:
        self.Sem.acquire()
        self.Lock.acquire()
        if not self.MsgQueue.empty():
            data = self.MsgQueue.get()
            print 'Receive: ' + repr(data)
        self.Lock.release()

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# UART receiving thread
class RecvThread (threading.Thread):

   def __init__(self, CmdBuffer):
      threading.Thread.__init__(self)
      self.CmdBuffer = CmdBuffer

   def run(self):
      while 1:
        inData = ser.read()
        self.CmdBuffer.WriteBuffer(inData)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
# Command processing thread
class SendThread (threading.Thread):

   def __init__(self, MsgQueue, Lock, Sem):
      threading.Thread.__init__(self)
      self.MsgQueue = MsgQueue
      self.Lock = Lock
      self.Sem = Sem

   def run(self):
      while 1:
        self.Sem.acquire()
        self.Lock.acquire()
        if not self.MsgQueue.empty():
            data = self.MsgQueue.get()
            ser.write(data)
        self.Lock.release()

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

RecvMsgQueue = Queue.Queue(QUEUE_SIZE)
RecvMsgQueueMutex = threading.Lock()
RecvMsgQueueSem = threading.Semaphore(QUEUE_SIZE)
CmdBuffer = CommandBuffer(QUEUE_MESS_SIZE, RecvMsgQueue, RecvMsgQueueMutex, RecvMsgQueueSem)

SendMsgQueue = Queue.Queue(QUEUE_SIZE)
SendMsgQueueMutex = threading.Lock()
SendMsgQueueSem = threading.Semaphore(QUEUE_SIZE)

ser = serial.Serial(
    port = SERIAL_DEVICE,
    baudrate = SERIAL_SPEED,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = None
)

# Create new threads
RecvThread = RecvThread(CmdBuffer)
StringProcThread = StringProcThread(RecvMsgQueue, RecvMsgQueueMutex, RecvMsgQueueSem)
SendThread = SendThread(SendMsgQueue, SendMsgQueueMutex, SendMsgQueueSem)

# Start new Threads
RecvThread.start()
StringProcThread.start()
SendThread.start()
