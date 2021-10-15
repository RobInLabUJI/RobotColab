import math
import rospy
import geometry_msgs.msg
import nav_msgs.msg

def sleep(t):
  """Sleep t seconds in ROS time."""
  rospy.sleep(t)

class TurtleBotRobot:
  """Proxy to TurtleBot robot in ROS."""

  def __init__(self):
    rospy.init_node('controller', anonymous=True)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size=10)
    self.odomSub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.odomCallback)

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

