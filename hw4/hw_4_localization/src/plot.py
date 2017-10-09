#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


def test(data):
  plt.gcf().clear()
  plt.plot(range(0, 600), data)#.data) 
  plt.xlabel('location')
  plt.ylabel('Probability')
  plt.title('')
  plt.axis([0, 600, 0, 1.0])
  plt.grid(True)
  plt.ion()
  plt.pause(0.05)


def listener():

  rospy.init_node('listener', anonymous=True)
 # rospy.Subscriber("/robot/wall_door_sensor", String, callback)
  rospy.Subscriber("/robot/histogram", Float32MultiArray, test)
  rospy.spin()

if __name__=='__main__':
  listener()


