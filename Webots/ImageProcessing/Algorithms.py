import cv2, numpy as np
import IPython.display
import PIL.Image

from enum import Enum

class Color(Enum):
  RED = 1
  YELLOW = 2
  GREEN = 3
  CYAN = 4
  BLUE = 5
  MAGENTA = 6


def display(image, size=(300,200)):
  """Display the image in the notebook."""
  if image.dtype == np.dtype('uint8'):
    PILimage = PIL.Image.fromarray(image)
  elif image.dtype == np.dtype('float64'):
    intimage = np.round(image/np.max(image)*255).astype('uint8')
    PILimage = PIL.Image.fromarray(intimage)
  IPython.display.display(PILimage.resize(size))

def colorFilter(image, color):
  LOW_S = 160
  HIGH_S = 255
  LOW_V = 160
  HIGH_V = 255
  hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
  if color == Color.RED:
    LOW_H = 0
    HIGH_H = 10
  elif color == Color.YELLOW:
    LOW_H = 20
    HIGH_H = 40
  elif color == Color.GREEN:
    LOW_H = 50
    HIGH_H = 70
  elif color == Color.CYAN:
    LOW_H = 80
    HIGH_H = 100
  elif color == Color.BLUE:
    LOW_H = 110
    HIGH_H = 130
  elif color == Color.MAGENTA:
    LOW_H = 140
    HIGH_H = 160
  else:
    return None
  lower_hsv = np.array([LOW_H, LOW_S, LOW_V])
  upper_hsv = np.array([HIGH_H, HIGH_S, HIGH_V])
  mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
  return mask

def blobDetector(mask):
  M = cv2.moments(mask)
  if M['m00']==0:
    cx = -1
    cy = -1
  else:
    cx = (M['m10']/M['m00'])
    cy = (M['m01']/M['m00'])
  return cx, cy

