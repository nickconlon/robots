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
SCALE_FACTOR = 2 
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
    self.l = np.array([[0.5 for x in range(GRID_MAX_X)] for j in range(GRID_MAX_Y)], np.float32)

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


#
# The callback for receiving sensor readings.
#
def sensorCallback(data):
  z = data.ranges
  c = 20 # (np.rad2deg(data.angle_max)+360)%360
  if(state.x != None and state.y != None): #so we don't try to map before we get state information
    x = point2d((state.x+OFFSET_X), (state.y+OFFSET_Y)) # current position
    occupancyGridMapping(x, z, c, data.range_max)

  pub.publish(getTwistToPublish(z))
  occ.publish(getGridToPublish(state.grid))


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
# Return true if mapPoint is in the perceptual field of the robot given it's state
#
def inPerceptualField(mp, cp, headingDeg, sensorConeDeg, maxRange):
  d = mp.getDistance(cp) 
  theta_l = ((headingDeg+sensorConeDeg)+360)%360
  theta_r = ((headingDeg-sensorConeDeg)+360)%360
  theta_point = ((np.rad2deg(np.arctan2(mp.y-cp.y, mp.x-cp.x))-headingDeg)+360)%360
  if(d <= maxRange and theta_point < theta_l and theta_point > theta_r):
    return True
  return False
   

#
#
#
def occupancyGridMapping(cp, z, sensorConeDeg, maxRange):
  h0 = (np.rad2deg(state.theta-np.pi/2)+360)%360 # right 90 deg
  h1 = (np.rad2deg(state.theta-np.pi/4)+360)%360 # right 45 deg
  h2 = (np.rad2deg(state.theta)+360)%360         # center
  h3 = (np.rad2deg(state.theta+np.pi/4)+360)%360 # left 45 deg
  h4 = (np.rad2deg(state.theta+np.pi/2)+360)%360 # left 90 deg
  thetas = [h0, h1, h2, h3, h4]

  for y in range(GRID_MAX_Y):
    for x in range(GRID_MAX_X):

      # center of mass of the cell we are looking at
      mp = point2d(x/SCALE_FACTOR+SCALE_FACTOR/2, y/SCALE_FACTOR+SCALE_FACTOR/2)
  
      if(inPerceptualField(mp, cp, h2, 90, maxRange)):
        state.l[y][x] = state.l[y][x] + inverseSensorModel(mp, cp, maxRange, z, thetas, sensorConeDeg) - 0.5
        prob = 1-(1.0/(1+np.exp(state.l[y][x])))
        print prob
        if prob > 0.90:
          state.grid[y][x] = 100
        elif prob > 0.25:
          state.grid[y][x] = 0
        else:
          state.grid[y][x] = -1
       
#      if(inPerceptualField(mp, cp, h1, sensorConeDeg, maxRange)):
#        state.grid[y][x] = inverseSensorModel(mp, cp, z[1])
#      if(inPerceptualField(mp, cp, h2, sensorConeDeg, maxRange)):
#        state.grid[y][x] = inverseSensorModel(mp, cp, z[2])
#      if(inPerceptualField(mp, cp, h3, sensorConeDeg, maxRange)):
#        state.grid[y][x] = inverseSensorModel(mp, cp, z[3])
#      if(inPerceptualField(mp, cp, h4, sensorConeDeg, maxRange)):
#        state.grid[y][x] = inverseSensorModel(mp, cp, z[4])
  
#
# Inverse sensor model
#
# Return 1 if mp is occluded, zero othewise
#
def inverseSensorModel(mp, cp, zmax, z, thetas, beamwidth):
  #pg 288 algorith 
  print "inverse sensor model" 
  alpha = 1.0/SCALE_FACTOR
  beta = beamwidth
  r = mp.getDistance(cp)
  phi = ((np.rad2deg(np.arctan2(mp.y-cp.y, mp.x-cp.x))-thetas[2])+360)%360
  k = argmin(phi, thetas)
  print "kay"
  if r > min(zmax, z[k]+alpha/2) or abs(phi-thetas[k]) > beta/2:
    return 0.5 # l_0
  elif z[k] < zmax:# and abs(r-z[k]) < alpha/2:
    return 1.0 # l_occ
  else:
    return 0.0 # l_free

  print "fail"



def argmin(phi, thetas):
  k = 360 #max degree difference
  i = 0
  for j in range(len(thetas)):
    arg = abs(phi-thetas[j])
    if(arg < k):
      k = arg
      i = j
  return i

      

#
# Fill in the twist message based on current position data
#
def getTwistToPublish(z):
  if(z[1] < 2): # don't crash
    twist.linear.x = 0.0
    twist.angular.z = 10.0
  elif(z[1] < z[3]): # don't go too far in either direction
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
