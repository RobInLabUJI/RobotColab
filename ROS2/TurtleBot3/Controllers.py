import rclpy
from rclpy.node import Node
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import math
import numpy as np

class TurtleBot3Robot(Node):
  """Proxy to TurtleBot 3 robot in ROS 2."""

  def __init__(self):
    rclpy.init()
    super().__init__('tb3')
    
    self.odomSubscription = self.create_subscription(
        nav_msgs.msg.Odometry,
        '/odom',
        self.odomCallback,
        10)
    self.odomSubscription
    
    self.scanSubscription = self.create_subscription(
        sensor_msgs.msg.LaserScan,
        '/scan',
        self.scanCallback,
        10)
    self.scanSubscription
    a = np.linspace(0,360,360,False)*np.pi/180
    self.c = np.cos(a)
    self.s = np.sin(a)
    
    self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)
    self.scan = None
    
    rclpy.spin_once(self)

  def move(self, v, w):
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = float(v)
    twist.angular.z = float(w)
    self.publisher_.publish(twist)
    rclpy.spin_once(self)
    
  def scanCallback(self, data):
    self.scan = data.ranges

  def odomCallback(self, data):
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    self.w = data.pose.pose.orientation.w
    self.z = data.pose.pose.orientation.z

  def imgCallback(self, data):
    pass

  def getPose(self):
    rclpy.spin_once(self)
    halfTheta = math.atan2(self.z, self.w)
    return self.x, self.y, 2 * halfTheta

  def getScan(self):
    rclpy.spin_once(self)
    return self.scan

  def getPointCloud(self):
    rclpy.spin_once(self)
    d = self.scan
    x = np.multiply(d, self.c)
    y = np.multiply(d, self.s)
    return x, y

  def getImage(self):
    return self.cv_image

