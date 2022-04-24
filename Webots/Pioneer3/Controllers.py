from controller import Robot
import numpy as np
import threading
import time
import math

__basicTimeStep__ = 10

def step(ms=__basicTimeStep__):
  global __robot__
  global __stop__
  __stop__ = False
  try:
    while __robot__.step(ms) != -1 and not __stop__:
      pass
  except NameError:
    pass

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
      if not sonarDevice is None:
        sonarDevice.enable(__basicTimeStep__)
        self.sonar.append(sonarDevice)
    self.__gps__ = __robot__.getDevice("gps")
    if not self.__gps__ is None:
      self.__gps__.enable(__basicTimeStep__)
    self.__compass__ = __robot__.getDevice("compass")
    if not self.__compass__ is None:
      self.__compass__.enable(__basicTimeStep__)
    self.laser = __robot__.getDevice("Hokuyo URG-04LX")
    if not self.laser is None:
      self.laser.enable(__basicTimeStep__)
    self.kinect = PioneerKinect()
    self.gripper = PioneerGripper()
    
  def disconnect(self):
    global __robot__
    global __step__
    global __stop__
    __stop__ = True
    __step__.join()
    __robot__ = None
    
  def getSonar(self, rear=False):
    d = []
    if rear:
      n = 16
    else:
      n = 8
    for i in range(n):
      d.append(self.sonar[i].getValue())
    return d
    
  def getPose(self):
    """Return pose of (x,y,theta) the robot."""
    if self.__compass__ is None or self.__gps__ is None:
      return None
    vx, vy, vz = self.__compass__.getValues()
    theta = math.atan2(vz, vx)
    px, py, pz = self.__gps__.getValues()
    return pz, px, theta

  def getEncoders(self):
    return (self.leftWheelSensor.getValue(), self.rightWheelSensor.getValue())
    
  def setSpeed(self, wl, wr):
    """Set wheel velocities."""
    self.leftMotor.setVelocity(wl)
    self.rightMotor.setVelocity(wr)

  def move(self, v, w):
    """Set linear and angular velocities."""
    r = 0.1953 / 2
    L = 0.33
    wl = (2*v - L*w) / (2*r)
    wr = (2*v + L*w) / (2*r)
    self.leftMotor.setVelocity(wl)
    self.rightMotor.setVelocity(wr)

  def stop(self):
    """Stop the robot."""
    self.move(0,0)
    
class PioneerKinect:
  """Proxy to Kinect camera in Webots."""

  def __init__(self):
    """Create proxy and enable devices."""
    global __robot__
    self.__kinectColor__ = __robot__.getDevice("kinect color")
    if not self.__kinectColor__ is None:
      self.__kinectColor__.enable(__basicTimeStep__*4)
    self.__kinectRange__ = __robot__.getDevice("kinect range")
    if not self.__kinectRange__ is None:
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
  
  def setTiltPosition(self, angle):
    self.tiltMotor.setPosition(angle)

class PioneerGripper:
  """Proxy to Pioneer3 gripper in Webots."""

  def __init__(self):
    """Create proxy and enable motors."""
    global __robot__
    self.__liftMotor__ = __robot__.getDevice("lift motor")
    self.__leftFingerMotor__ = __robot__.getDevice("left finger motor")
    self.__rightFingerMotor__ = __robot__.getDevice("right finger motor")
    if not self.__leftFingerMotor__ is None: 
        self.__leftFingerMotor__.setVelocity(0.1)
    if not self.__rightFingerMotor__ is None: 
        self.__rightFingerMotor__.setVelocity(0.1)
    if not self.__liftMotor__ is None: 
        self.__liftMotor__.setVelocity(0.1)

  def up(self):
    """Move gripper up."""
    if not self.__liftMotor__ is None: 
        self.__liftMotor__.setPosition(0)

  def down(self):
    """Move gripper down."""
    if not self.__liftMotor__ is None: 
        self.__liftMotor__.setPosition(0.05)

  def open(self):
    """Open gripper."""
    if not self.__leftFingerMotor__ is None: 
        self.__leftFingerMotor__.setPosition(0.1)
    if not self.__rightFingerMotor__ is None: 
        self.__rightFingerMotor__.setPosition(0.1)

  def close(self, position=0.065):
    """Close gripper."""
    if not self.__leftFingerMotor__ is None: 
        self.__leftFingerMotor__.setPosition(position)
    if not self.__rightFingerMotor__ is None: 
        self.__rightFingerMotor__.setPosition(position)

