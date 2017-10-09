#!/usr/bin/env python

import numpy as np
import rospy
import sensing_model_and_map as prob
import plot as plt

from std_msgs.msg import String
from geometry_msgs.msg import Twist

doorReading = True

#
# The callback for receibing sensor readings
#
def callback(data): #todo delete me for speed!
  rospy.loginfo("I heard %s", data.data)
  global doorReading
  if data.data == 'Door':
    doorReading = True
  else:
    doorReading = False

#
# The bayes filter
#
def bayesFilter(bel, u, d):
  n = 0
  bel_ = [0]*600
  
  if d == 'z':
    for x in range(len(bel)):
      if doorReading == True: #door reading
        bel_[x]=prob.p_door(x/10)*bel[x]
        n = n + bel_[x]
      else: # wall reading
        bel_[x]=(1-prob.p_door(x/10))*bel[x]
        n = n + bel_[x]
    for x in range(len(bel)):
      bel_[x]=bel_[x]/n;

  if d == 'u':
    belShift = move(bel, 4)#shift(bel)
    for x in range(len(bel)):
      bel_[x]=belShift[x]*bel[x]

  return bel_

#
# Shift (motion model)
#
def shift(array):
  for x in range(len(array)-1):
    array[len(array)-x-1] = array[len(array)-x-2]
  array[0]=1/600.0
  return array

def move(array, m):
  n = len(array)
  for x in range(n):
    array[x] = array[(x-m)%n]
  return array

#
# The main loop
#
def bot():

  #initialize the pub's and sub's and ros things
  pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=100)
  sub = rospy.Subscriber('robot/wall_door_sensor', String, callback)
  rospy.init_node('pybot', anonymous=True)
  rate = rospy.Rate(10)
  
  #initialize the probelm specific things
  hist = [1/600.0]*600
  velocity = 4
  position = 0

  while not rospy.is_shutdown():
    
    hist = bayesFilter(hist, position, 'z')

    hist = shift(hist)
    if position %20 == 0:
      hist = bayesFilter(hist, position, 'u')
    
    position = position + 4
    print position
    
    plt.test(hist)
    twist = Twist()
    twist.linear.x = velocity
    pub.publish(twist)
    rate.sleep()



#
# The thing that you do in pythong
#
if __name__=='__main__':
  try:
    bot()
  except rospy.ROSInterruptException:
    pass
