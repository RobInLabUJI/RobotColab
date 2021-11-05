import math
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

def sleep(t):
  """Sleep t seconds in ROS time."""
  rospy.sleep(t)

class TurtleBotRobot:
  """Proxy to TurtleBot robot in ROS."""

  def __init__(self):
    rospy.init_node('controller', anonymous=True)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size=10)
    self.odomSub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.odomCallback)
    self.bridge = CvBridge()
    self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, self.imgCallback)

  def move(self, v, w):
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = v
    twist.angular.z = w
    self.pub.publish(twist)

  def odomCallback(self, data):
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    self.w = data.pose.pose.orientation.w
    self.z = data.pose.pose.orientation.z
    
  def getPose(self):
    halfTheta = math.atan2(self.z, self.w)
    return self.x, self.y, 2 * halfTheta

  def imgCallback(self, data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      pass

  def getImage(self):
    return self.cv_image

