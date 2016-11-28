#!/usr/bin/env python  

import rospy 
from math import cos, sin
import tf
import geometry_msgs.msg
from std_msgs.msg import Empty

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

drone_land = None

prev_diff = None

def hover():
  print "Hovering"
  msg = geometry_msgs.msg.Twist()
  msg.linear.x = 0
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
  return msg

def goto_point(tx, ty, cx, cy, ct):
  global prev_diff
  global TARGET_X
  
  #print "Controlling", tx, ty, cx, cy, ct, -0.5*ct,
  msg = geometry_msgs.msg.Twist()

  rot = np.matrix([[cos(ct), -sin(ct)],[sin(ct), cos(ct)]])
  diff = np.matrix([[tx - cx],[ty - cy]])

  # No error derivative
  if prev_diff == None:
    prev_diff = diff
    
  d_diff = diff - prev_diff

  #print "Error diff change: "
  #print d_diff

  vels = PROP_LINEAR * np.dot(rot, diff) + D_LINEAR *  np.dot(rot, d_diff)
  
  print vels

  norm = np.linalg.norm(vels)
  if norm > NORM_LIMIT: 
    vels = vels / norm * NORM_LIMIT
    print "Normalizing"
  
    

  #print "Controlling", tx, ty, cx, cy, ct, -0.5*ct, vels[0], vels[1]
  
  msg.linear.x = vels[0]
  msg.linear.y = vels[1]
  msg.linear.z = 0
  msg.angular.x = 1
  msg.angular.y = 1
  msg.angular.z = -0.5*ct

  prev_diff = diff

  # Change target
  if np.linalg.norm(diff) < THRS_ARRIVED:
    TARGET_X = -TARGET_X

  return msg

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
  rospy.sleep(2)  

  # Take off
  print "Taking off"
  drone_takeoff.publish(Empty())

  

  rospy.on_shutdown(land)

  # Control loop
  rate = rospy.Rate(50.0)
  while not rospy.is_shutdown():
    try:
        # Get oldest timestamp for transform
        latest = listener.getLatestCommonTime(map_frame, marker_frame)
        #if (rospy.Time() - latest > rospy.Duration(MAX_BLACKOUT)):
        if (rospy.Time() - latest > rospy.Duration(MAX_BLACKOUT)):
          msg = hover()
        else: 
          # Obtain the transform
          (trans,rot) = listener.lookupTransform(map_frame, marker_frame, rospy.Time())
          
          cx = trans[0]
          cy = trans[1]
          euler = tf.transformations.euler_from_quaternion(rot)
          ct = euler[2]
          msg = goto_point(TARGET_X, TARGET_Y, cx, cy, ct)

        
          
    except (tf.Exception):
        msg = hover()

    drone_vel.publish(msg)

    rate.sleep()

  
