from controller import Robot
import numpy as np
import threading

import IPython.display
import PIL.Image

def display(image):
  if image.dtype == np.dtype('uint8'):
    IPython.display.display(PIL.Image.fromarray(image))
  elif image.dtype == np.dtype('float64'):
    intimage = np.round(image/np.max(image)*255).astype('uint8')
    IPython.display.display(PIL.Image.fromarray(intimage))

__basicTimeStep__ = 10

def step(ms=__basicTimeStep__):
  global __robot__
  try:
    while __robot__.step(ms) != -1:
      pass
  except NameError:
    pass

class PioneerRobot:
  def __init__(self):
    if not '__robot__' in globals():
      global __robot__
      __robot__ = Robot()
      global __step__
      __step__ = threading.Thread(target=step, args=(10,))
      __step__.start()
    self.leftMotor = __robot__.getDevice("left wheel")
    self.leftMotor.setPosition(float('inf'))
    self.leftMotor.setVelocity(0)
    self.rightMotor = __robot__.getDevice("right wheel")
    self.rightMotor.setPosition(float('inf'))
    self.rightMotor.setVelocity(0)
    self.leftWheelSensor = __robot__.getDevice("left wheel sensor")
    self.leftWheelSensor.enable(__basicTimeStep__)
    self.rightWheelSensor = __robot__.getDevice("right wheel sensor")
    self.rightWheelSensor.enable(__basicTimeStep__)
    self.sonar = []
    for i in range(16):
      sonarDevice = __robot__.getDevice("so"+str(i))
      sonarDevice.enable(__basicTimeStep__)
      self.sonar.append(sonarDevice)

class PioneerKinect:
  def __init__(self):
    global __robot__
    self.__kinectColor__ = __robot__.getDevice("kinect color")
    self.__kinectColor__.enable(__basicTimeStep__*4)
    self.__kinectRange__ = __robot__.getDevice("kinect range")
    self.__kinectRange__.enable(__basicTimeStep__*4)
    self.tiltMotor = __robot__.getDevice("tilt motor")
  def getColorImage(self):
    data = self.__kinectColor__.getImage()
    height = self.__kinectColor__.getHeight()
    width = self.__kinectColor__.getWidth()
    image = np.frombuffer(data, np.uint8).reshape((height, width, 4))
    return image[:,:,[2,1,0]]
  def getRangeImage(self):
    data = self.__kinectRange__.getRangeImage()
    height = self.__kinectRange__.getHeight()
    width = self.__kinectRange__.getWidth()
    image =  np.array(data).reshape(height,width)
    return image

class PioneerGripper:
  def __init__(self):
    global __robot__
    self.__liftMotor__ = __robot__.getDevice("lift motor")
    self.__leftFingerMotor__ = __robot__.getDevice("left finger motor")
    self.__rightFingerMotor__ = __robot__.getDevice("right finger motor")
    self.__leftFingerMotor__.setVelocity(0.1)
    self.__rightFingerMotor__.setVelocity(0.1)
    self.__liftMotor__.setVelocity(0.1)
  def up(self):
    self.__liftMotor__.setPosition(0)
  def down(self):
    self.__liftMotor__.setPosition(0.05)
  def open(self):
    self.__leftFingerMotor__.setPosition(0.1)
    self.__rightFingerMotor__.setPosition(0.1)
  def close(self):
    self.__leftFingerMotor__.setPosition(0)
    self.__rightFingerMotor__.setPosition(0)

# def step(ms=__basicTimeStep__):
#   global __robot__
#   try:
#     __robot__.step(ms)
#   except NameError:
#     pass

