#!/usr/bin/env python

# Nicholas Conlon
# Basic program for occupancy grid mapping given a 
# 2D environment, a map, and some sensor readings.

import numpy as np
import rospy
import sys
import plot as plt

from point2d import point2d
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

# Some constatnts
SENSOR_CONE_DEGREES = 45
SCALE_FACTOR = 4
GRID_MAX_X = SCALE_FACTOR*60
GRID_MAX_Y = SCALE_FACTOR*16
OFFSET_X = 30
OFFSET_Y = 8

# Twist message
twist = Twist()

# Occupancy grid message
grid_msgs = OccupancyGrid()
grid_msgs.header.frame_id = '/map'
grid_msgs.info.resolution = 1./SCALE_FACTOR
grid_msgs.info.width = GRID_MAX_X
grid_msgs.info.height = GRID_MAX_Y
grid_msgs.data = range(GRID_MAX_X*GRID_MAX_Y)

class Data():
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta
    self.grid = np.array([[-1.0 for x in range(GRID_MAX_X)] for j in range(GRID_MAX_Y)], np.int8)

# Robot state
state = Data(None, None, 0.0)

# Publishers
pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)
occ = rospy.Publisher('/map',OccupancyGrid, queue_size=1)


#
# Callback for State truth data
#
def truthCallback(data):
  state.x = data.pose.pose.position.x
  state.y = data.pose.pose.position.y
  state.theta = get_rotation(data)
  return


#
# The callback for receiving sensor readings.
#
def sensorCallback(data):
  r1 = data.ranges[1] # to the right 45
  r2 = data.ranges[2] # straight ahead
  r3 = data.ranges[3] # to the left 45
  
  if(state.x != None and state.y != None): #so we don't try to map before we get state information
    x = point2d((state.x+OFFSET_X), (state.y+OFFSET_Y)) # current position
    occupancyGridMapping(x, r1, r2, r3)

  pub.publish(getTwistToPublish(r1, r2, r3))
  occ.publish(getGridToPublish(state.grid))
  return

#
# Fill in the grid message to publish
#
def getGridToPublish(msg):
  grid_msgs.data = np.ravel(msg, order='C')
  return grid_msgs


#
# Given some orientation message, return the yaw (heading estimate)
#
def get_rotation (msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

#
# Inverse sensor model
#
def inverseSensorModel(mp, cp, z):
  numerator = 1  #np.random.normal(
  denominator = 1  #1-numerator
  print np.log(numerator/denominator)
  return 1


#
# Return true if mapPoint is in the perceptual field of the robot given it's state
#
def inPerceptualField(mapPoint, curPoint, heading_deg, reading):
  d = mapPoint.getDistance(curPoint) 
  theta_l = heading_deg+SENSOR_CONE_DEGREES
  theta_r = heading_deg-SENSOR_CONE_DEGREES
  theta_point = np.arctan2(mapPoint.y-curPoint.y, mapPoint.x-curPoint.x)
  theta_point = np.rad2deg(theta_point)
  if(d <= reading and theta_point < theta_l and theta_point > theta_r):
    return True
  return False
   

# matrix l
# Point m
# Point x
# double heading angle z
def occupancyGridMapping(cp, r1, r2, r3):

  heading_deg = (np.rad2deg(state.theta)+360)%360
  for y in range(GRID_MAX_Y):
    for x in range(GRID_MAX_X):
      mp = point2d(x/SCALE_FACTOR+SCALE_FACTOR/2, y/SCALE_FACTOR+SCALE_FACTOR/2)
      if(inPerceptualField(mp, cp, heading_deg, r2)):
        state.grid[y][x]=state.grid[y][x]+inverseSensorModel(mp, cp, r2) # -lo ? const prior
  return



#
# Fill in the twist message based on current position data
#
def getTwistToPublish(r1, r2, r3):
  if(r1 < 2): # don't crash
    twist.linear.x = 0.0
    twist.angular.z = 10.0
  elif(r1 < r3): # don't go too far in either direction
    twist.angular.z = 2.0
    twist.linear.x = 5.0
  else:
    twist.angular.z = -2.0
    twist.linear.x = 5.0
  return twist


#
# The main loop for this bot.
#
def bot(argv):
  rospy.init_node('pybot', anonymous=True)
  sensor_sub = rospy.Subscriber('robot/base_scan', LaserScan, sensorCallback, queue_size=1)
  truth_sub = rospy.Subscriber('stage/base_pose_ground_truth', Odometry, truthCallback, queue_size=1) 
  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():

    # Do some zzz.
    rate.sleep()



#
# The thing that you do in python.
#
if __name__=='__main__':
  try:
    bot(sys.argv)
  except rospy.ROSInterruptException:
    pass
