from controller import Robot

__basicTimeStep__ = 64

class TurtleBot3Burger:
  def __init__(self):
    if not '__robot__' in globals():
      global __robot__
      __robot__ = Robot()
    self.leftWheelMotor = __robot__.getDevice("left wheel motor")
    self.leftWheelMotor.setPosition(float('inf'))
    self.leftWheelMotor.setVelocity(0)
    self.rightWheelMotor = __robot__.getDevice("right wheel motor")
    self.rightWheelMotor.setPosition(float('inf'))
    self.rightWheelMotor.setVelocity(0)
    self.leftWheelSensor = __robot__.getDevice("left wheel sensor")
    self.leftWheelSensor.enable(__basicTimeStep__)
    self.rightWheelSensor = __robot__.getDevice("right wheel sensor")
    self.rightWheelSensor.enable(__basicTimeStep__)
    self.lidar = __robot__.getDevice("LDS-01")
    self.lidar.enable(__basicTimeStep__)

def step(ms=__basicTimeStep__):
  global __robot__
  try:
    __robot__.step(ms)
  except NameError:
    pass

