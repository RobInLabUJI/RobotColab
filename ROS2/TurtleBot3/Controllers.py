
def sleep(t):
  """Sleep t seconds in ROS time."""
  #rospy.sleep(t)

class TurtleBot3Robot:
  """Proxy to TurtleBot 3 robot in ROS."""

  def __init__(self):
    pass

  def move(self, v, w):
    pass
    
  def scanCallback(self, data):
    pass

  def odomCallback(self, data):
    pass

  def imgCallback(self, data):
    pass

  def getPose(self):
    halfTheta = math.atan2(self.z, self.w)
    return self.x, self.y, 2 * halfTheta

  def getScan(self):
      return self.scan

  def getPointCloud(self):
      d = self.scan
      x = np.multiply(d,self.c)
      y = np.multiply(d,self.s)
      return x, y

  def getImage(self):
    return self.cv_image

