#!/usr/bin/env python

# Nicholas Conlon
# Basic program for localization using a particle filter given a 
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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time


SENSOR_CONE_DEGREES = 45
SCALE_FACTOR =1
GRID_MAX_X = SCALE_FACTOR*60
GRID_MAX_Y = SCALE_FACTOR*16
OFFSET_X = GRID_MAX_X/2
OFFSET_Y = GRID_MAX_Y/2

class Data():
  grey = -1#[155, 155, 155]
  black =0.5# [255, 255, 255]
  white =1# [1, 1, 1]
  
truth = Data()
truth.img = [[truth.grey for x in range(GRID_MAX_X)] for j in range(GRID_MAX_Y)]
truth.prob = [[0.5 for x in range(GRID_MAX_X)] for j in range(GRID_MAX_Y)]
truth.x = None
truth.y = None
truth.theta = 0


pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)



#
# The callback for receiving sensor readings.
#
def sensorCallback(data):
  #rospy.loginfo("[045] = %s", data.ranges[1])
  
  
  if(truth.x != None and truth.y != None):
    y = int(truth.y*SCALE_FACTOR+OFFSET_Y)
    x = int(truth.x*SCALE_FACTOR+OFFSET_X)
    #truth.img[y][x]=truth.white
    
    for l in range(int(data.ranges[2])): # forward
      x = int(truth.x*SCALE_FACTOR+OFFSET_X)
      y = int(truth.y*SCALE_FACTOR+OFFSET_Y)
      x = int(x+l*np.cos(truth.theta))
      y = int(y+l*np.sin(truth.theta))
      #truth.img[y][x]=truth.white
    

    curPoint = point2d((truth.x+OFFSET_X)/SCALE_FACTOR, (truth.y+OFFSET_Y)/SCALE_FACTOR)
    for i in range(GRID_MAX_X):# x
      for j in range(GRID_MAX_Y): # y
        mapPoint = point2d(i/SCALE_FACTOR+SCALE_FACTOR/2, j/SCALE_FACTOR+SCALE_FACTOR/2)
        if(inPerceptualField(mapPoint, curPoint, truth.theta, data.ranges[2])):
          truth.img[j][i]=truth.white

    
  pub.publish(getTwistToPublish(data.ranges[1], data.ranges[2], data.ranges[3]))
  plt.plot(truth.img) 
  return

def get_rotation (msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw


def truthCallback(data):
  truth.x = data.pose.pose.position.x
  truth.y = data.pose.pose.position.y
  truth.theta = get_rotation(data)
  return


def inverseSensorModel(m, x, z):
  numerator = 1  #p(m|z, x)
  denominator = 1  #1-p(m|z, x)
  return np.log(numerator/denominator)

def inPerceptualField(mapPoint, curPoint, heading, reading):
  d = mapPoint.getDistance(curPoint) 
  heading_deg = (np.rad2deg(heading)+360)%360
  theta_l = heading_deg+SENSOR_CONE_DEGREES
  theta_r = heading_deg-SENSOR_CONE_DEGREES
  theta_point = np.arctan2(mapPoint.y-curPoint.y, mapPoint.x-curPoint.x)
  theta_point = np.rad2deg(theta_point)
  if(d <= reading and theta_point < theta_l and theta_point > theta_r):
    return True

  return False
   

# Point x
# Point y
def getCentroid(x, y):
  return 


# matrix l
# Point m
# Point x
# double heading angle z
def occupancyGridMapping(l, x, z):
  for i in range(len(l)):
    if(inPerceptualField(m, x, z)):
      l[i] = l[i]+inverseSensorModel(m,x,z)-l0
    else:
      l[i] = l[i]
  return l




def getTwistToPublish(r1, r2, r3):
  twist = Twist()
  twist.linear.x = 5.0

  if(r1 < 2):
    twist.linear.x = 0.0
    twist.angular.z = 10.0
  elif(r1 < r3):
    twist.angular.z = 2.0
  else:
    twist.angular.z = -2.0

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
