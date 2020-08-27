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
SCALE_FACTOR =1
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
pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=100)
occ = rospy.Publisher('/map',OccupancyGrid, queue_size=100)


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
  beammax = (np.rad2deg(data.angle_max)+360)%360
  beamwidth = beammax/len(data.ranges)
  if(state.x != None and state.y != None): #so we don't try to map before we get state information
    x = point2d((state.x+OFFSET_X), (state.y+OFFSET_Y)) # current position
    occupancyGridMapping(x, z, beamwidth, beammax, data.range_max)

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
def inPerceptualField(mp, cp, headingDeg, beammax, zmax):
  d = mp.getDistance(cp)
  theta_mp = (np.rad2deg(np.arctan2(mp.y-cp.y, mp.x-cp.x))+360)%360
  diff = angleDiff(headingDeg, theta_mp)
  if( d <= zmax and diff < beammax):
    return True
  return False
   

#
# Run the occupancy grid mapping algorithm
#
def occupancyGridMapping(cp, z, beamwidth, beammax, zmax):
  thetas = getSensorAngles()
  heading = thetas[2]

  for y in range(GRID_MAX_Y):
    for x in range(GRID_MAX_X):

      # the point we want to look at 
      mp = point2d(x/SCALE_FACTOR, y/SCALE_FACTOR)
       
      if(inPerceptualField(mp, cp, heading, beammax, zmax)):
        log = inverseSensorModel(mp, cp, zmax, z, thetas, beamwidth) - 0.5
        state.l[y][x] = state.l[y][x] + log 

        # extract the probability
        prob = 1-(1.0/(1+np.exp(state.l[y][x])))

        # fill in the grid for visulalization. these values require tweaking :(
        setGrid(prob, x, y)


#
# Inverse sensor model as seen on pg 288 of the textbook
#
def inverseSensorModel(mp, cp, zmax, z, thetas, beamwidth):
  alpha = 1.0/SCALE_FACTOR
  beta = beamwidth
  headingTheta = thetas[2]
  r = mp.getDistance(cp)
  phi = (np.rad2deg(np.arctan2(mp.y-cp.y, mp.x-cp.x))+360)%360
  k = argmin(phi, thetas)
  if r > min(zmax, z[k]+alpha/2) or angleDiff(phi, thetas[k]) > beta/2:
    return 0.5 # l_0
  elif z[k] < zmax and abs(r-z[k]) < alpha/2:
    return 1.0 # l_occ
  else:
    return 0.0 # l_free



#
# Find the argmin over j
#
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
# Find the difference between two angles in degrees
#
def angleDiff(a, b):
  a1 = (a+360)%360
  b1 = (b+360)%360
  return 180-abs(abs(a1 - b1)-180)

#
#
#
def getSensorAngles():
  h0 = (np.rad2deg(state.theta-np.pi/2)+360)%360 # right 90 deg
  h1 = (np.rad2deg(state.theta-np.pi/4)+360)%360 # right 45 deg
  h2 = (np.rad2deg(state.theta)+360)%360         # center
  h3 = (np.rad2deg(state.theta+np.pi/4)+360)%360 # left 45 deg
  h4 = (np.rad2deg(state.theta+np.pi/2)+360)%360 # left 90 deg
  thetas = [h0, h1, h2, h3, h4]
  return thetas

#
#
#
def setGrid(prob, x, y):
  if prob > 0.7:
    state.grid[y][x] = 100
  elif prob < 0.5:
    state.grid[y][x] = 0
  else:
    state.grid[y][x] = -1




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
