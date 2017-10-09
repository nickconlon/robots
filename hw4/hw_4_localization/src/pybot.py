#!/usr/bin/env python

# Nicholas Conlon
# Basic probram for localization using a bayes filter given a 
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
# The bayes filter.
#
def bayesFilter(bel, u, d):
  if d == 'z':
    bel_ = sensorUpdate(bel)
  if d == 'u':
    bel_ = movementUpdate(bel, u)
  return norm(bel_)

#
# Apply a sensor update.
#
def sensorUpdate(bel):
  bel_ = [0]*len(bel)
  for x in range(len(bel)):
      if doorReading == True: #door reading
        bel_[x]=prob.p_door(x/10)*bel[x]
      else: # wall reading
        bel_[x]=(1-prob.p_door(x/10))*bel[x]
  return bel_

#
# Apply a movement update.
#
def movementUpdate(bel, m):
  rolled = np.roll(bel, m) 
  for x in range(abs(m)):
    rolled[x] = 1/600 #just use some small non-zero probability.
  return norm(rolled)

#
# Normalize the array.
#
def norm(bel):
  n = sum(bel)
  if n != 0:
    for x in range(len(bel)):
      bel[x]=bel[x]/n
  return bel

#
# The main loop for this bot.
#
def bot(argv):

  fail = False

  if len(argv) == 2:
    if argv[1] == 'east':
      position = 600
      velocity = -4
    if argv[1] == 'west':
      position = 0
      velocity = 4
    else:
      fail = True
  else:
    fail = True


  if fail == True:
    print 'usage pybot.py <driving direction: \'east\' or \'west\'>'
    sys.exit()

  #initialize the pub's and sub's and ros things.
  pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=100)
  sub = rospy.Subscriber('robot/wall_door_sensor', String, callback)
  rospy.init_node('pybot', anonymous=True)
  rate = rospy.Rate(10)
  
  #initialize the probelm specific things.
  hist = [1/600.0]*600
  
  while not rospy.is_shutdown():
   
    # Apply the bayes filter for a sensor reading. 
    hist = bayesFilter(hist, velocity/2, 'z')

    # Apply the beyes filter for a movement. 
    hist = bayesFilter(hist, velocity/2, 'u') 
    
    # Update the position.
    position = position + velocity
    
    # Plot the histogram.
    plt.test(hist)

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
