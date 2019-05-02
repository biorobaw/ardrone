#!/usr/bin/env python  

import rospy 
from math import cos, sin
import tf
import geometry_msgs.msg
from std_msgs.msg import Empty
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
import time

import numpy as np

map_frame = 'map'
marker_frame = 'ar_marker_0'
MAX_BLACKOUT = 2
TARGET_X = -1.27
TARGET_Y = 0
PROP_LINEAR = .75
D_LINEAR = 15
NORM_LIMIT = .1
THRS_ARRIVED = .1
THRS_VELS = .005

drone_land = None

prev_diff = None
hovering = 0

get_x = 320
get_y = 180
close = 1000
obst_x = 960
obst_y = 540
obst_size = 10

bridge = CvBridge()

img = Image()

def hover():
  #print(get_x)
  global get_x
  global get_y
  global close
  global obst_x
  global obst_y
  global obst_size
  msg = geometry_msgs.msg.Twist()
  msg.linear.x = 0
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
  """if (close > 13000 and get_x >= 320 and get_x < 641):
    print("strafe left")
    msg.linear.y = 0.075
    msg.angular.z = -0.1
    close = 10
  elif (close > 13000 and get_x < 320):
    print("strafe right")
    msg.linear.y = -0.075
    msg.angular.z = 0.2
    close = 10
  elif close <= 13000:"""
  if (obst_x >= 255 and obst_x <= 385):
    msg.angular.z = 0
  elif obst_x > 385:
    msg.angular.z = -0.2
  elif obst_x < 255:
    msg.angular.z = 0.2
  """if (obst_y >= 140 and obst_y <= 220):
    msg.linear.z = 0
  elif obst_y > 220:
    msg.linear.z = -0.2
  elif obst_y < 140:
    msg.linear.z = 0.2"""
  if (obst_size >= 5000 and obst_size <= 4000):
    msg.linear.x = 0
    drone_land.publish(Empty())
    exit()
  elif obst_size > 5000:
    msg.linear.x = 0
    drone_land.publish(Empty())
    exit()
  elif obst_size < 4000:
    msg.linear.x = 0.02
  return msg



def getContours(binary_image):

  _, contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  return contours


def draw_contours(image, contours, image_name):

  index = -1
  thickness = 2
  color = (255, 0, 255)
  cv2.drawContours(image, contours, index, color, thickness)
  cv2.imshow(image_name, image)


def process_contours(binary_image, rgb_image, contours):
  global get_x
  global get_y
  global close

  black_image = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

  for i in contours:
    area = cv2.contourArea(i)
    perimeter = cv2.arcLength(i, True)
    ((x, y), radius) = cv2.minEnclosingCircle(i)
    if (area > 50):
      #cv2.drawContours(rgb_image, [i], -1, (150, 250, 150), 1)
      cv2. drawContours(black_image, [i], -1, (150, 250, 150), 1)
      cx, cy = get_contour_center(i)
      #cv2.circle(rgb_image, (cx, cy), (int)(radius),(0, 0, 255), 1)
      cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
      drone_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
      msg = geometry_msgs.msg.Twist()
      get_x = cx
      get_y = cy
      close = area
      #print("Area: {}, Perimeter: {}".format(area, perimeter))
  #print("Number of Contours: {}".format(len(contours)))
  #cv2.imshow("RGB Image Contours", rgb_image)
  cv2.imshow("Black Image Contours", black_image)


def process_contours2(binary_image, rgb_image, contours):
  global obst_x
  global obst_y
  global obst_size

  black_image2 = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

  for i in contours:
    area = cv2.contourArea(i)
    perimeter = cv2.arcLength(i, True)
    ((x, y), radius) = cv2.minEnclosingCircle(i)
    if (area > 50):
      #cv2.drawContours(rgb_image, [i], -1, (150, 250, 150), 1)
      cv2. drawContours(black_image2, [i], -1, (150, 250, 150), 1)
      cx, cy = get_contour_center(i)
      #cv2.circle(rgb_image, (cx, cy), (int)(radius),(0, 0, 255), 1)
      cv2.circle(black_image2, (cx, cy), (int)(radius), (0, 0, 255), 1)
      drone_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
      msg = geometry_msgs.msg.Twist()
      obst_x = cx
      obst_y = cy
      obst_size = area
      #print("Area: {}, Perimeter: {}".format(area, perimeter))
  #print("Number of Contours: {}".format(len(contours)))
  #cv2.imshow("RGB Image Contours", rgb_image)
  cv2.imshow("Black Image Contours2", black_image2)


def get_contour_center(contour):

  M = cv2.moments(contour)
  ix = -1
  iy = -1
  if (M['m00'] != 0):
    ix = int(M['m10']/M['m00'])
    iy = int(M['m01']/M['m00'])
  return ix, iy


def filter_color(rgb_image, lower_bound_color, upper_bound_color):

  hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
  pinklower = (100, 100, 100)
  pinkupper = (180, 200, 200)

  mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
  return mask



def image_callback(ros_image):

  #print("got an image")
  global bridge
  try:
    obst_lower = (90, 90, 200)
    obst_upper = (180, 255, 255)
    goal_lower = (10, 110, 100)  #fishfood
    goal_upper = (100, 255, 255) #fishfood
    #pinklower = (20, 20, 10)
    #pinkupper = (180, 255, 255)
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    binary_image_mask = filter_color(cv_image, goal_lower, goal_upper)
    binary_image_mask2 = filter_color(cv_image, obst_lower, obst_upper)
    contours = getContours(binary_image_mask)
    contours2 = getContours(binary_image_mask2)
    process_contours(binary_image_mask, cv_image, contours)
    process_contours2(binary_image_mask2, cv_image, contours2)

  except CvBridgeError as e:
    print(e)

  (rows, cols, channels) = cv_image.shape
  cv2.waitKey(3)

def land():
  print "Landing"
  drone_land.publish(Empty())
  rospy.sleep(1)

if __name__ == "__main__":
  # Initialize TF listener
  rospy.init_node('hover')

  listener = tf.TransformListener()

  # Initialize cmd_vel publisher
  drone_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
  drone_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
  drone_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
  image_sub = rospy.Subscriber('/ardrone/front/image_raw', Image, image_callback)

  rospy.sleep(2)  

  # Take off
  print "Taking off"
  """drone_takeoff.publish(Empty())
  msg2 = geometry_msgs.msg.Twist()
  msg2.linear.x = 0
  msg2.linear.y = 0
  msg2.linear.z = 0
  msg2.angular.x = 0
  msg2.angular.y = 0
  msg2.angular.z = 0
  drone_vel.publish(msg2)
  time.sleep(3.5)"""

  runescape = 0

  while runescape < 70:
    #msg = hover()
    #drone_vel.publish(msg)
    time.sleep(0.3)
    runescape = runescape + 1
    print(runescape)

  #drone_land.publish(Empty())
  

  #rospy.on_shutdown(land)


  # Control loop
  rate = rospy.Rate(50.0)
