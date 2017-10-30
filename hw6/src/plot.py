#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from matplotlib import collections as matcoll

def plot(data):
  plt.gcf().clear()

  plt.imshow(data, interpolation='none', origin='lower')
 
  plt.grid(True)
  plt.ion()
  plt.pause(0.000000005)

