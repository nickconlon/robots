#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from matplotlib import collections as matcoll

def test(particles, weights):
  plt.gcf().clear()
 # s = 1000
 # x = np.random.randint(600, size=s)
 # y = [0]*s
 # for i in range(len(x)):
 #   y[i] = data[x[i]]
  
  lines = []
  for i in range(len(particles)):
    pair=[(particles[i], 0), (particles[i], weights[i])]
    lines.append(pair)

  linecoll = matcoll.LineCollection(lines)
  plt.axes().add_collection(linecoll)

  plt.scatter(particles, weights)
#  plt.plot(range(0, 600), data)#.data) 
  plt.xlabel('location')
  plt.ylabel('Probability')
  plt.title('')
  plt.axis([0, 600, 0, 0.3])
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


