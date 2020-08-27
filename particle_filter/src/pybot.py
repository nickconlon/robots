#!/usr/bin/env python

# Nicholas Conlon
# Basic program for localization using a particle filter given a 
# 2D environment, a map, and some sensor readings.

import numpy as np
import rospy
import sys
import sensing_model_and_map as prob
import plot as plt

from std_msgs.msg import String
from geometry_msgs.msg import Twist

doorReading = True

#
# The callback for receiving sensor readings.
#
def callback(data):
  rospy.loginfo("I heard %s", data.data)
  global doorReading
  if data.data == 'Door':
    doorReading = True
  else:
    doorReading = False

#
# The particle filter.
#
def particleFilter(Xt1, vel, door):
  xt = [0]*len(Xt1)
  wt = [0]*len(Xt1)
  J = len(Xt1)

  for j  in range(J): 
    # movement update
    xt[j] = np.random.normal(Xt1[j]+vel, abs(vel)) 
    
    # sensor update
    if door == True:
      wt[j] = prob.p_door(xt[j]/10)
    else:
      wt[j] = prob.p_wall(xt[j]/10)
 
  # normalize weights
  wt = norm(wt)

  # resampling 
  r = lowVarianceResampling(xt, wt)
  
  # updating particles and weights 
  X_ = [0]*len(xt)
  W_ = [0]*len(wt)

  for x in range(len(r)):
    X_[x] = xt[r[x]]
    W_[x] = wt[r[x]] 

  # print the histogram
  plt.test(X_, W_)

  return X_

#
# Apply low variance resampling 
#
def lowVarianceResampling(xt, wt):
  J = len(xt)
  X_ = [0]*J
  r = np.random.uniform(0, 1/float(J))
  c = wt[0]
  i = 0
  for j in range(J):
    U = r + (j)/float(J)
    while U > c:
      i = i + 1
      c = c + wt[i]
    X_[j] = i
  return X_


#
# Normalize the array.
#
def norm(array):
  n = sum(array)
  if n != 0:
    for x in range(len(array)):
      array[x]=array[x]/n
  return array


#
# The main loop for this bot.
#
def bot(argv):

  fail = False

  if len(argv) == 2:
    if argv[1] == 'west':
      position = 600
      velocity = -2
    elif argv[1] == 'east':
      position = 0
      velocity = 2
    else:
      fail = True
  else:
    fail = True


  if fail == True:
    print 'usage pybot.py <driving direction: \'east\' or \'west\'>'
    sys.exit()

  #initialize the pub's and sub's and ros things.
  pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('robot/wall_door_sensor', String, callback)
  rospy.init_node('pybot', anonymous=True)
  rate = rospy.Rate(10)
  
  #initialize the probelm specific things.
  mapSize = 600
  numSamples = 200
  particles = np.random.randint(mapSize, size=numSamples)

  while not rospy.is_shutdown():
 
    particles = particleFilter(particles, velocity*2, doorReading) 
    
    # Update the position.
    position = position + velocity
   
    # Send out some movement commands.
    twist = Twist()
    twist.linear.x = velocity
    pub.publish(twist)

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
