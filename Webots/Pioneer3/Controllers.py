from controller import Robot
import numpy as np
import threading
import time
import math

import IPython.display
import PIL.Image

def display(image, size=(300,200)):
  """Display the image in the notebook."""
  if image.dtype == np.dtype('uint8'):
    PILimage = PIL.Image.fromarray(image)
  elif image.dtype == np.dtype('float64'):
    intimage = np.round(image/np.max(image)*255).astype('uint8')
    PILimage = PIL.Image.fromarray(intimage)
  IPython.display.display(PILimage.resize(size))

__basicTimeStep__ = 10

def step(ms=__basicTimeStep__):
  global __robot__
  try:
    while __robot__.step(ms) != -1:
      pass
  except NameError:
    pass

def sleep(t):
  """Sleep t seconds in simulated time."""
  global __robot__
  start = __robot__.getTime()
  while __robot__.getTime()-start < t:
    time.sleep(__basicTimeStep__/1000)

class PioneerRobot:
  """Proxy to Pioneer3 robot in Webots."""

  def __init__(self):
    """Create proxy and enable devices:
       * leftMotor
       * rightMotor
       * leftWheelSensor
       * rightWheelSensor
       * sonar
    """
    if not '__robot__' in globals():
      global __robot__
      __robot__ = Robot()
      global __step__
      __step__ = threading.Thread(target=step, args=(10,))
      __step__.start()
    self.leftMotor = __robot__.getDevice("left wheel")
    """Motor of left wheel - instance variable"""
    self.leftMotor.setPosition(float('inf'))
    self.leftMotor.setVelocity(0)
    self.rightMotor = __robot__.getDevice("right wheel")
    """Motor of right wheel - instance variable"""
    self.rightMotor.setPosition(float('inf'))
    self.rightMotor.setVelocity(0)
    self.leftWheelSensor = __robot__.getDevice("left wheel sensor")
    """Encoder of left wheel - instance variable"""
    self.leftWheelSensor.enable(__basicTimeStep__)
    self.rightWheelSensor = __robot__.getDevice("right wheel sensor")
    """Encoder of right wheel - instance variable"""
    self.rightWheelSensor.enable(__basicTimeStep__)
    self.sonar = []
    """List of distance sensors - instance variable"""
    for i in range(16):
      sonarDevice = __robot__.getDevice("so"+str(i))
      sonarDevice.enable(__basicTimeStep__)
      self.sonar.append(sonarDevice)
    self.__gps__ = __robot__.getDevice("gps")
    self.__gps__.enable(__basicTimeStep__)
    self.__compass__ = __robot__.getDevice("compass")
    self.__compass__.enable(__basicTimeStep__)
    
  def getPose(self):
    vx, vy, vz = self.__compass__.getValues()
    theta = math.atan2(vz, vx)
    px, py, pz = self.__gps__.getValues()
    return pz, px, theta

class PioneerKinect:
  """Proxy to Kinect camera in Webots."""

  def __init__(self):
    """Create proxy and enable devices."""
    global __robot__
    self.__kinectColor__ = __robot__.getDevice("kinect color")
    self.__kinectColor__.enable(__basicTimeStep__*4)
    self.__kinectRange__ = __robot__.getDevice("kinect range")
    self.__kinectRange__.enable(__basicTimeStep__*4)
    self.tiltMotor = __robot__.getDevice("tilt motor")

  def getColorImage(self):
    """Return the color image."""
    data = self.__kinectColor__.getImage()
    height = self.__kinectColor__.getHeight()
    width = self.__kinectColor__.getWidth()
    image = np.frombuffer(data, np.uint8).reshape((height, width, 4))
    return image[:,:,[2,1,0]]

  def getRangeImage(self):
    """Return the range image."""
    data = self.__kinectRange__.getRangeImage()
    height = self.__kinectRange__.getHeight()
    width = self.__kinectRange__.getWidth()
    image =  np.array(data).reshape(height,width)
    return image

class PioneerGripper:
  """Proxy to Pioneer3 gripper in Webots."""

  def __init__(self):
    """Create proxy and enable motors."""
    global __robot__
    self.__liftMotor__ = __robot__.getDevice("lift motor")
    self.__leftFingerMotor__ = __robot__.getDevice("left finger motor")
    self.__rightFingerMotor__ = __robot__.getDevice("right finger motor")
    self.__leftFingerMotor__.setVelocity(0.1)
    self.__rightFingerMotor__.setVelocity(0.1)
    self.__liftMotor__.setVelocity(0.1)

  def up(self):
    """Move gripper up."""
    self.__liftMotor__.setPosition(0)

  def down(self):
    """Move gripper down."""
    self.__liftMotor__.setPosition(0.05)

  def open(self):
    """Open gripper."""
    self.__leftFingerMotor__.setPosition(0.1)
    self.__rightFingerMotor__.setPosition(0.1)

  def close(self):
    """Close gripper."""
    self.__leftFingerMotor__.setPosition(0)
    self.__rightFingerMotor__.setPosition(0)

