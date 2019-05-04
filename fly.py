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
TARobst_x = -1.27
TARobst_y = 0
PROP_LINEAR = .75
D_LINEAR = 15
NORM_LIMIT = .1
THRS_ARRIVED = .1
THRS_VELS = .005

drone_land = None

prev_diff = None
hovering = 0

obst_x = 320
obst_y = 180
obst_size = 1000
goal_x = 960
goal_y = 540
goal_size = 10

bridge = CvBridge()

img = Image()

def hover():

  global obst_x
  global obst_y
  global obst_size
  global goal_x
  global goal_y
  global goal_size
  msg = geometry_msgs.msg.Twist()
  msg.linear.x = 0
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
  if (obst_size > 13000 and obst_x >= 320 and obst_x < 641):
    msg.linear.y = 0.075
    msg.angular.z = -0.1
    obst_size = 10
  elif (obst_size > 13000 and obst_x < 320):
    msg.linear.y = -0.075
    msg.angular.z = 0.2
    obst_size = 10
  elif obst_size <= 13000:
  if (goal_x >= 255 and goal_x <= 385):
    msg.angular.z = 0
  elif goal_x > 385:
    msg.angular.z = -0.2
  elif goal_x < 255:
    msg.angular.z = 0.2
  if (goal_y >= 140 and goal_y <= 220):
    msg.linear.z = 0
  elif goal_y > 220:
    msg.linear.z = -0.2
  elif goal_y < 140:
    msg.linear.z = 0.2
  if (goal_size >= 5000 and goal_size <= 4000):
    msg.linear.x = 0
    drone_land.publish(Empty())
    exit()
  elif goal_size > 5000:
    msg.linear.x = 0
    drone_land.publish(Empty())
    exit()
  elif goal_size < 4000:
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
  global obst_x
  global obst_y
  global obst_size

  black_image = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

  for i in contours:
    area = cv2.contourArea(i)
    perimeter = cv2.arcLength(i, True)
    ((x, y), radius) = cv2.minEnclosingCircle(i)
    if (area > 50):
      cv2. drawContours(black_image, [i], -1, (150, 250, 150), 1)
      cx, cy = get_contour_center(i)
      cv2.circle(black_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
      drone_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
      msg = geometry_msgs.msg.Twist()
      obst_x = cx
      obst_y = cy
      obst_size = area
      #print("Area: {}, Perimeter: {}".format(area, perimeter))
  cv2.imshow("Black Image Contours", black_image)


def process_contours2(binary_image, rgb_image, contours):
  global goal_x
  global goal_y
  global goal_size

  black_image2 = np.zeros([binary_image.shape[0], binary_image.shape[1], 3], 'uint8')

  for i in contours:
    area = cv2.contourArea(i)
    perimeter = cv2.arcLength(i, True)
    ((x, y), radius) = cv2.minEnclosingCircle(i)
    if (area > 50):
      cv2. drawContours(black_image2, [i], -1, (150, 250, 150), 1)
      cx, cy = get_contour_center(i)
      cv2.circle(black_image2, (cx, cy), (int)(radius), (0, 0, 255), 1)
      drone_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
      msg = geometry_msgs.msg.Twist()
      goal_x = cx
      goal_y = cy
      goal_size = area
      #print("Area: {}, Perimeter: {}".format(area, perimeter))
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

  mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
  return mask



def image_callback(ros_image):

  global bridge
  try:
    goal_l = (90, 90, 200)
    goal_u = (180, 255, 255)
    obst_l = (10, 110, 100)
    obst_u = (100, 255, 255)
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    binary_image_mask = filter_color(cv_image, obst_l, obst_u)
    binary_image_mask2 = filter_color(cv_image, goal_l, goal_u)
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
  drone_takeoff.publish(Empty())

  move = 0

  while move < 70:
    msg = hover()
    drone_vel.publish(msg)
    time.sleep(0.3)
    move = move + 1
    print(move)

  drone_land.publish(Empty())
  

  rospy.on_shutdown(land)


  # Control loop
  rate = rospy.Rate(50.0)
