import math
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

def sleep(t):
  """Sleep t seconds in ROS time."""
  rospy.sleep(t)

class TurtleBot3Robot:
  """Proxy to TurtleBot 3 robot in ROS."""

  def __init__(self):
    rospy.init_node('controller', anonymous=True)
    self.pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    self.odomSub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.odomCallback)
    self.scanSub = rospy.Subscriber('scan', sensor_msgs.msg.LaserScan, self.scanCallback)
    a = np.linspace(0,360,360,False)*np.pi/180
    self.c = np.cos(a)
    self.s = np.sin(a)
    self.bridge = CvBridge()
    self.imgSub = rospy.Subscriber('/raspicam_node/image/compressed', sensor_msgs.msg.CompressedImage, self.imgCallback)

  def move(self, v, w):
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = v
    twist.angular.z = w
    self.pub.publish(twist)

  def scanCallback(self, data):
    self.scan = data.ranges

  def odomCallback(self, data):
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    self.w = data.pose.pose.orientation.w
    self.z = data.pose.pose.orientation.z

  def imgCallback(self, data):
    try:
      #self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
      np_arr = np.fromstring(data.data, np.uint8)
      bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      self.cv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
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

